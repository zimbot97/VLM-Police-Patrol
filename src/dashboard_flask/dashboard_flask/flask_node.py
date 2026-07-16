#!/usr/bin/env python3
"""
dashboard_flask.flask_node
---------------------------
Single ROS2 node that runs an rclpy spin loop in a background thread and a
Flask + Flask-SocketIO web server in the main thread.

Exposes:
  - GET  /                 dashboard page
  - GET  /video_feed       MJPEG stream of the latest camera image
  - GET  /holonomic        {enabled: bool} current drive mode, for initial page load
  - WS   cmd_vel            (client -> server) {linear, strafe, angular} -> published to /cmd_vel
  - WS   set_holonomic      (client -> server) {enabled} -> published to /holonomic_mode (Bool)
  - WS   holonomic_state    (server -> client) {enabled} broadcast on every mode change

  suspect_matcher integration:
  - POST /suspect/upload_reference   multipart 'image' -> saved as reference crop
  - POST /suspect/capture_candidate  sets detector save_basename + calls /capture_crop
  - POST /suspect/compare            calls /compare_images (first call = slow cold VLM load)
  - GET  /suspect/reference.jpg      current reference crop
  - GET  /suspect/candidate.jpg      current candidate crop
  - GET  /suspect/result             {match, detail} latest cached result
  - WS   match_result / match_detail (server -> client) pushed from result topics

Topics/services (override via ROS2 params):
  image_topic          (default: /camera/image_raw)      sensor_msgs/Image
  compressed_image_topic (default: /camera/image_raw/compressed)  sensor_msgs/CompressedImage
  use_compressed       (default: False)  subscribe to compressed topic and relay
                        JPEG bytes verbatim (near-zero CPU) instead of re-encoding
  stream_max_width     (default: 0=off)  downscale frames wider than this before streaming
  stream_fps           (default: 15)     MJPEG pacing cap
  jpeg_quality         (default: 70)     only used when (re)encoding raw/resized frames
  cmd_vel_topic        (default: /cmd_vel)               geometry_msgs/Twist
  holonomic_mode_topic (default: /holonomic_mode)        std_msgs/Bool
  capture_crop_service (default: /capture_crop)          std_srvs/Trigger
  compare_service      (default: /compare_images)        std_srvs/Trigger
  detector_node_name   (default: /yolo_detect_node)      (for save_basename param set)
  match_topic          (default: /suspect_feature_match)         std_msgs/Bool
  match_detail_topic   (default: /suspect_feature_match_detail)  std_msgs/String
  reference_image_path (default: /tmp/reference_crop.jpg)
  candidate_image_path (default: /tmp/candidate_crop.jpg)
  candidate_basename   (default: candidate)
  port                 (default: 5000)

NOTE: the map/AMCL tab was removed; map_topic/pose_topic subscriptions remain
in the node but are no longer surfaced in the UI.
"""

import io
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template, request, send_file
from flask_socketio import SocketIO
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def _find_template_folder():
    """Work both when run from source (colcon build --symlink-install /
    plain `python3 flask_node.py`) and when installed, where templates are
    copied to share/dashboard_flask/templates by setup.py's data_files."""
    local = os.path.join(os.path.dirname(__file__), 'templates')
    if os.path.isdir(local):
        return local
    try:
        return os.path.join(get_package_share_directory('dashboard_flask'), 'templates')
    except Exception:  # noqa: BLE001
        return local


def _find_static_folder():
    """Same source/installed resolution as templates, for the static/ dir.
    Under --merge-install, setup.py copies static/ to
    share/dashboard_flask/static/ — Flask must be told this path explicitly
    or the vendored socket.io.min.js 404s and the offline page breaks."""
    local = os.path.join(os.path.dirname(__file__), 'static')
    if os.path.isdir(local):
        return local
    try:
        return os.path.join(get_package_share_directory('dashboard_flask'), 'static')
    except Exception:  # noqa: BLE001
        return local


# ---------------------------------------------------------------------------
# ROS2 node — owns all subscriptions/publishers and the shared latest-state
# ---------------------------------------------------------------------------
class DashboardNode(Node):
    def __init__(self, socketio: SocketIO):
        super().__init__('dashboard_flask_node')
        self.socketio = socketio
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # latest cached state, guarded by self.lock
        self.latest_frame_jpeg = None      # bytes
        self.latest_map_png = None         # bytes
        self.latest_map_info = None        # dict: resolution, origin_x, origin_y, width, height
        self.latest_pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0}
        self.holonomic_mode = False

        # --- params -----------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('compressed_image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('holonomic_mode_topic', '/holonomic_mode')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('stream_max_width', 0)   # 0 = no downscale
        self.declare_parameter('stream_fps', 15)
        self.declare_parameter('port', 5000)

        # --- suspect_matcher integration params -------------------------
        self.declare_parameter('capture_crop_service', '/capture_crop')
        self.declare_parameter('compare_service', '/compare_images')
        self.declare_parameter('detector_node_name', '/yolo_detect_node')
        self.declare_parameter('match_topic', '/suspect_feature_match')
        self.declare_parameter('match_detail_topic', '/suspect_feature_match_detail')
        self.declare_parameter('suspect_pose_topic', '/suspect_pose')
        self.declare_parameter('reference_image_path', '/tmp/reference_crop.jpg')
        self.declare_parameter('candidate_image_path', '/tmp/candidate_crop.jpg')
        self.declare_parameter('candidate_basename', 'candidate')

        image_topic = self.get_parameter('image_topic').value
        compressed_image_topic = self.get_parameter('compressed_image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        map_topic = self.get_parameter('map_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        holonomic_topic = self.get_parameter('holonomic_mode_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.stream_max_width = self.get_parameter('stream_max_width').value
        self.stream_fps = max(1, int(self.get_parameter('stream_fps').value))
        self.port = self.get_parameter('port').value

        self.capture_crop_service = self.get_parameter('capture_crop_service').value
        self.compare_service = self.get_parameter('compare_service').value
        self.detector_node_name = self.get_parameter('detector_node_name').value
        match_topic = self.get_parameter('match_topic').value
        match_detail_topic = self.get_parameter('match_detail_topic').value
        suspect_pose_topic = self.get_parameter('suspect_pose_topic').value
        self.reference_image_path = self.get_parameter('reference_image_path').value
        self.candidate_image_path = self.get_parameter('candidate_image_path').value
        self.candidate_basename = self.get_parameter('candidate_basename').value

        # latest match result, guarded by self.lock
        self.latest_match = None          # bool or None
        self.latest_match_detail = None   # str or None
        self.latest_suspect = None        # {'x', 'y'} in map frame, or None

        # map/pose from Nav2 are typically published with TRANSIENT_LOCAL /
        # RELIABLE QoS — match it or you'll silently receive nothing.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Image source: compressed passthrough (cheap — the camera already did
        # the JPEG encode, we just relay bytes) vs raw (we encode each frame).
        # QoS: camera streams are usually BEST_EFFORT; match it so we don't
        # silently receive nothing from a sensor-data publisher.
        img_qos = QoSProfile(depth=1)
        img_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        if self.use_compressed:
            self.create_subscription(
                CompressedImage, compressed_image_topic, self.on_compressed_image,
                img_qos)
            self._image_source = compressed_image_topic
        else:
            self.create_subscription(Image, image_topic, self.on_image, img_qos)
            self._image_source = image_topic

        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)

        # laser scan: sensor stream -> BEST_EFFORT, throttled before pushing
        # to the browser (raw 10-30 Hz x hundreds of points is too heavy)
        scan_topic = self.get_parameter('scan_topic').value
        scan_qos = QoSProfile(depth=1)
        scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, scan_topic, self.on_scan, scan_qos)
        self._last_scan_emit = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.holonomic_pub = self.create_publisher(Bool, holonomic_topic, 10)
        initialpose_topic = self.get_parameter('initialpose_topic').value
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initialpose_topic, 10)

        # --- suspect_matcher: service clients + result subscriptions -----
        # Use a reentrant callback group so service calls made from the Flask
        # thread don't deadlock the single-threaded executor.
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()

        self.capture_client = self.create_client(
            Trigger, self.capture_crop_service, callback_group=self.cb_group)
        self.compare_client = self.create_client(
            Trigger, self.compare_service, callback_group=self.cb_group)
        self.set_param_client = self.create_client(
            SetParameters, f'{self.detector_node_name}/set_parameters',
            callback_group=self.cb_group)

        self.create_subscription(Bool, match_topic, self.on_match, 10,
                                  callback_group=self.cb_group)
        self.create_subscription(String, match_detail_topic, self.on_match_detail, 10,
                                  callback_group=self.cb_group)
        # Suspect location is published latched (TRANSIENT_LOCAL) so we still
        # get the last fix even if the dashboard subscribes late.
        self.create_subscription(PoseStamped, suspect_pose_topic, self.on_suspect_pose,
                                  map_qos, callback_group=self.cb_group)

        self.get_logger().info(
            f'Dashboard node up. image_source={self._image_source} '
            f'(compressed={self.use_compressed}) map={map_topic} '
            f'pose={pose_topic} cmd_vel={cmd_vel_topic} holonomic={holonomic_topic}'
        )

    # --- subscription callbacks ------------------------------------------
    def _maybe_downscale(self, cv_img):
        """Downscale to stream_max_width if set and the frame is wider."""
        if self.stream_max_width and cv_img.shape[1] > self.stream_max_width:
            scale = self.stream_max_width / cv_img.shape[1]
            new_h = int(cv_img.shape[0] * scale)
            cv_img = cv2.resize(cv_img, (self.stream_max_width, new_h),
                                 interpolation=cv2.INTER_AREA)
        return cv_img

    def on_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'image convert failed: {exc}')
            return
        cv_img = self._maybe_downscale(cv_img)
        ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if ok:
            with self.lock:
                self.latest_frame_jpeg = buf.tobytes()

    def on_compressed_image(self, msg: CompressedImage):
        # Fast path: the camera already produced compressed (usually JPEG)
        # bytes. If we don't need to downscale, relay them verbatim — no
        # decode, no re-encode, near-zero CPU. Only pay decode+encode cost
        # when a downscale is actually requested.
        data = bytes(msg.data)
        fmt = (msg.format or '').lower()
        if not self.stream_max_width and ('jpeg' in fmt or 'jpg' in fmt or not fmt):
            with self.lock:
                self.latest_frame_jpeg = data
            return
        # need to resize, or it's not JPEG (e.g. png) -> decode, scale, re-encode
        arr = np.frombuffer(data, np.uint8)
        cv_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            return
        cv_img = self._maybe_downscale(cv_img)
        ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if ok:
            with self.lock:
                self.latest_frame_jpeg = buf.tobytes()

    def on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # occupancy grid: -1 unknown, 0 free, 100 occupied -> grayscale image
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 128        # unknown -> gray
        img[data == 0] = 255         # free -> white
        img[data > 0] = 0            # occupied -> black
        img = np.flipud(img)         # ROS map origin is bottom-left

        ok, buf = cv2.imencode('.png', img)
        if ok:
            with self.lock:
                self.latest_map_png = buf.tobytes()
                self.latest_map_info = {
                    'resolution': msg.info.resolution,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'width': w,
                    'height': h,
                }
            self.socketio.emit('map_update', self.latest_map_info)

    def on_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        # yaw from quaternion (z-axis rotation only, standard planar-robot case)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw_deg': math.degrees(yaw),
        }
        with self.lock:
            self.latest_pose = pose
        self.socketio.emit('pose_update', pose)

    def on_scan(self, msg: LaserScan):
        # Throttle browser pushes to ~5 Hz regardless of the lidar rate
        now = time.time()
        if now - self._last_scan_emit < 0.2:
            return
        self._last_scan_emit = now

        with self.lock:
            pose = dict(self.latest_pose)
        px, py = pose['x'], pose['y']
        pyaw = math.radians(pose['yaw_deg'])
        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)

        # Downsample to <=240 points to keep the payload phone-friendly.
        n = len(msg.ranges)
        if n == 0:
            return
        step = max(1, n // 240)

        # NOTE: assumes the laser frame ~= base frame (no mounting offset /
        # rotation). Good enough for display; not for mapping math.
        pts = []
        angle = msg.angle_min
        for i in range(0, n, step):
            r = msg.ranges[i]
            a = msg.angle_min + i * msg.angle_increment
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                lx = r * math.cos(a)
                ly = r * math.sin(a)
                wx = px + lx * cos_y - ly * sin_y
                wy = py + lx * sin_y + ly * cos_y
                pts.append([round(wx, 3), round(wy, 3)])
        self.socketio.emit('scan_update', {'points': pts})

    def on_match(self, msg: Bool):
        with self.lock:
            self.latest_match = bool(msg.data)
        self.socketio.emit('match_result', {'match': bool(msg.data)})

    def on_match_detail(self, msg: String):
        with self.lock:
            self.latest_match_detail = msg.data
        self.socketio.emit('match_detail', {'detail': msg.data})

    def on_suspect_pose(self, msg: PoseStamped):
        # Map-frame suspect location; the browser projects it to map pixels the
        # same way it does the robot pose (via /map_info).
        suspect = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
        }
        with self.lock:
            self.latest_suspect = suspect
        self.socketio.emit('suspect_update', suspect)

    # --- called from Flask thread -----------------------------------------
    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def set_holonomic(self, enabled: bool) -> bool:
        self.holonomic_mode = bool(enabled)
        self.holonomic_pub.publish(Bool(data=self.holonomic_mode))
        self.get_logger().info(f'holonomic_mode -> {self.holonomic_mode}')
        return self.holonomic_mode

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        """Publish an AMCL initial pose estimate (same semantics as RViz's
        '2D Pose Estimate' tool). yaw in radians, map frame."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # RViz default initial-pose covariance: 0.25 m^2 on x/y,
        # ~0.068 rad^2 on yaw — gives AMCL a sane uncertainty to converge from
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'initialpose -> x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg')

    def get_holonomic(self) -> bool:
        return self.holonomic_mode

    def get_frame(self):
        with self.lock:
            return self.latest_frame_jpeg

    def get_map_png(self):
        with self.lock:
            return self.latest_map_png

    def get_map_info(self):
        with self.lock:
            return dict(self.latest_map_info) if self.latest_map_info else None

    def get_pose(self):
        with self.lock:
            return dict(self.latest_pose)

    def get_suspect(self):
        with self.lock:
            return dict(self.latest_suspect) if self.latest_suspect else None

    # --- suspect_matcher actions (called from Flask thread) ---------------
    def _set_detector_basename(self, basename: str, timeout_sec: float = 5.0) -> bool:
        """Set the detector node's save_basename param so the next /capture_crop
        writes <basename>_crop.jpg. Returns True on success."""
        if not self.set_param_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('set_parameters service unavailable')
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'save_basename'
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=basename)
        req.parameters = [p]
        future = self.set_param_client.call_async(req)
        # block the Flask worker thread (not the ROS executor) until done
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.02)
        if not future.done():
            return False
        res = future.result()
        return bool(res and res.results and all(r.successful for r in res.results))

    def capture_candidate(self, timeout_sec: float = 15.0) -> dict:
        """Set basename=candidate, then call /capture_crop. Returns
        {ok, message}."""
        if not self._set_detector_basename(self.candidate_basename):
            return {'ok': False, 'message': 'could not set detector save_basename'}

        if not self.capture_client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'message': f'{self.capture_crop_service} unavailable'}

        future = self.capture_client.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.02)
        if not future.done():
            return {'ok': False, 'message': 'capture_crop timed out'}
        res = future.result()
        return {'ok': bool(res.success), 'message': res.message}

    def run_compare(self, timeout_sec: float = 900.0) -> dict:
        """Call /compare_images. NOTE: first call triggers a cold VLM load that
        can take several minutes (see handoff §6). Returns {ok, message}."""
        # clear stale result so the UI only shows this run's outcome
        with self.lock:
            self.latest_match = None
            self.latest_match_detail = None

        if not self.compare_client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'message': f'{self.compare_service} unavailable'}

        future = self.compare_client.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)
        if not future.done():
            return {'ok': False, 'message': f'compare timed out after {timeout_sec:.0f}s'}
        res = future.result()
        return {'ok': bool(res.success), 'message': res.message}

    def get_match(self):
        with self.lock:
            return {'match': self.latest_match, 'detail': self.latest_match_detail}


# ---------------------------------------------------------------------------
# Flask + SocketIO app
# ---------------------------------------------------------------------------
app = Flask(__name__,
            template_folder=_find_template_folder(),
            static_folder=_find_static_folder())
app.config['SECRET_KEY'] = 'dashboard-flask-secret'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

node: DashboardNode = None  # set in main()

# a small watchdog: if no cmd_vel from the client for this many seconds
# while a "key" is held down, we rely on the browser sending zero explicitly
# (see index.html) rather than a server-side timeout, to keep this simple.


@app.route('/')
def index():
    return render_template('index.html')


def mjpeg_generator():
    boundary = b'--frame'
    period = 1.0 / node.stream_fps
    while True:
        frame = node.get_frame()
        if frame is not None:
            yield (boundary + b'\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(period)  # paced by stream_fps param (default 15)


@app.route('/video_feed')
def video_feed():
    return Response(mjpeg_generator(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/map.png')
def map_png():
    png = node.get_map_png()
    if png is None:
        return Response(status=204)
    return Response(io.BytesIO(png).read(), mimetype='image/png')


@app.route('/map_info')
def map_info():
    info = node.get_map_info()
    return info if info is not None else {}


@app.route('/pose')
def pose():
    return node.get_pose()


@app.route('/holonomic')
def holonomic():
    return {'enabled': node.get_holonomic()}


@socketio.on('cmd_vel')
def handle_cmd_vel(data):
    linear_x = float(data.get('linear', 0.0))
    linear_y = float(data.get('strafe', 0.0))
    angular_z = float(data.get('angular', 0.0))
    node.publish_cmd_vel(linear_x, linear_y, angular_z)


@socketio.on('set_holonomic')
def handle_set_holonomic(data):
    enabled = bool(data.get('enabled', False))
    new_state = node.set_holonomic(enabled)
    # broadcast so every open tab/browser stays in sync
    socketio.emit('holonomic_state', {'enabled': new_state})


@socketio.on('set_initial_pose')
def handle_set_initial_pose(data):
    node.publish_initial_pose(
        float(data.get('x', 0.0)),
        float(data.get('y', 0.0)),
        float(data.get('yaw', 0.0)))


# ---------------------------------------------------------------------------
# suspect_matcher routes (upload reference / capture candidate / compare)
# ---------------------------------------------------------------------------
@app.route('/suspect/upload_reference', methods=['POST'])
def upload_reference():
    """Save the uploaded image as the reference crop the comparator reads."""
    if 'image' not in request.files:
        return jsonify({'ok': False, 'message': 'no image field in request'}), 400
    file = request.files['image']
    try:
        data = file.read()
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return jsonify({'ok': False, 'message': 'could not decode image'}), 400
        cv2.imwrite(node.reference_image_path, img)
    except Exception as exc:  # noqa: BLE001
        return jsonify({'ok': False, 'message': f'save failed: {exc}'}), 500
    return jsonify({'ok': True, 'message': 'reference saved',
                    'path': node.reference_image_path})


@app.route('/suspect/capture_candidate', methods=['POST'])
def capture_candidate():
    """Trigger yolo_detect /capture_crop to save the candidate crop."""
    result = node.capture_candidate()
    code = 200 if result['ok'] else 502
    return jsonify(result), code


@app.route('/suspect/compare', methods=['POST'])
def compare():
    """Run /compare_images. First call can take minutes (cold VLM load)."""
    result = node.run_compare()
    code = 200 if result['ok'] else 502
    # match/detail arrive asynchronously over sockets, but also return the
    # latest cached values in case they landed before the service returned
    result.update(node.get_match())
    return jsonify(result), code


@app.route('/suspect/reference.jpg')
def reference_jpg():
    if not os.path.isfile(node.reference_image_path):
        return Response(status=204)
    return send_file(node.reference_image_path, mimetype='image/jpeg')


@app.route('/suspect/candidate.jpg')
def candidate_jpg():
    if not os.path.isfile(node.candidate_image_path):
        return Response(status=204)
    return send_file(node.candidate_image_path, mimetype='image/jpeg')


@app.route('/suspect/result')
def suspect_result():
    return jsonify(node.get_match())


@app.route('/suspect/pose')
def suspect_pose():
    # Last known map-frame suspect location, or null if none yet. Used by the
    # browser to bootstrap the marker on (re)load, since the socket event only
    # fires on a new match.
    return jsonify(node.get_suspect())


# ---------------------------------------------------------------------------
# entrypoint
# ---------------------------------------------------------------------------
def ros_spin_thread(executor):
    executor.spin()


def main():
    global node
    rclpy.init()
    node = DashboardNode(socketio)

    # Multi-threaded executor: the reentrant callback group + service calls
    # issued from the Flask worker threads need real concurrency, otherwise a
    # call_async waiting on a result would starve the callback that delivers it.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    spin_thread.start()

    try:
        socketio.run(app, host='0.0.0.0', port=node.port, allow_unsafe_werkzeug=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
