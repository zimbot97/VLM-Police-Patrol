#!/usr/bin/env python3
"""
suspect_localizer_node.py

Turns a suspect match into a map-frame location + marker + JSON.

Two placement modes (param 'location_source'):
  "amcl_pose" (DEFAULT) — the suspect is placed at the robot's own map-frame
      pose from /amcl_pose, frozen at the /capture_crop moment. No depth cloud,
      no tf math — cheapest. Assumes you drive up to whoever you're checking.
  "pointcloud" — the 3D-scan logic below: centroid of the depth points inside
      the YOLO person bbox, transformed into the map frame.

Both freeze the location at capture time and emit it only when the match is
true. The rest of this doc describes the "pointcloud" path in detail.


This is a SEPARATE, self-contained node — it does not modify or depend on the
internals of yolo_detect_node / attribute_compare_from_files_node, it only
consumes their published topics:

  in  <detections_topic>  (default /yolo/detections, ai_msgs/PerceptionTargets)
        The YOLO person boxes. yolo_detect publishes here exactly when
        /capture_crop runs, so EACH message is treated as a capture event: we
        take the LARGEST person box (the same "largest person" rule the
        detector uses to pick its crop) and freeze a 3D fix for that instant.

  in  <pointcloud_topic>  (default /camera/depth_registered/points,
        sensor_msgs/PointCloud2)
        Organized (height x width) XYZ+RGB cloud, one point per pixel, in the
        camera optical frame. Because it is organized, pixel (u, v) maps
        directly to point index v*width + u — so the YOLO pixel box selects a
        rectangular block of points with no projection math needed. This topic
        is subscribed ON DEMAND — only for the moment around a capture — and
        never streamed in steady state, so it costs nothing when idle.

  in  <match_topic>  (default /suspect_feature_match, std_msgs/Bool)
        The yes/no VLM match result from attribute_compare_from_files_node.

  in  <match_detail_topic> (default /suspect_feature_match_detail,
        std_msgs/String) — human-readable breakdown, copied into the JSON.

What it does:
  1. At each capture event: take the points inside the capture bbox from the
     time-matched cloud, drop NaN/inf (the cloud is is_dense=false — invalid
     depth pixels are NaN, encoded as 0x7FC00000), and average them ->
     centroid in the camera optical frame. This is the "centroid of the
     pointcloud within the bbox" (a lightweight PCL-style pass; we use numpy
     directly since this is a Python package and the cloud is organized).
  2. Transform that centroid into <target_frame> (default 'map') with tf2,
     using the cloud's own stamp so the pose reflects the robot's pose at the
     capture instant, and FREEZE it. The frozen fix is emitted later when the
     match arrives — important because the VLM can take minutes on a cold
     load, during which the robot/person may move; we must not re-sample.

  out (on <match_topic> == true only):
    - <output_json_path> (default /tmp/suspect_location.json): the map-frame
      centroid, the source camera-frame centroid, the bbox, point count,
      stamp, target frame, and the match detail string.
    - <marker_topic> (default /suspect_marker, visualization_msgs/Marker):
      a SPHERE at the map-frame centroid, published with transient_local
      durability so RViz picks it up even if it subscribes late.

Params (all overridable with --ros-args -p name:=value):
  detections_topic, pointcloud_topic, match_topic, match_detail_topic,
  target_frame, person_label, marker_topic, output_json_path,
  marker_scale, min_valid_points, tf_timeout_sec.
"""

import json
import threading

import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import (Point, PoseStamped,
                               PoseWithCovarianceStamped)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker


def quat_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a quaternion (x, y, z, w)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


class SuspectLocalizerNode(Node):

    def __init__(self):
        super().__init__("suspect_localizer_node")

        # How to place the suspect at capture time:
        #   "amcl_pose"  (default) -> use the robot's own map-frame pose from
        #                 /amcl_pose. Cheap: no cloud, no tf, no depth needed.
        #                 The suspect is reported AT the robot (good enough
        #                 when you drive up to whoever you're checking).
        #   "pointcloud" -> centroid of the depth points inside the YOLO bbox,
        #                 transformed to the map frame (the 3D-scan logic).
        self.declare_parameter("location_source", "amcl_pose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("pointcloud_topic",
                               "/camera/depth_registered/points")
        self.declare_parameter("match_topic", "/suspect_feature_match")
        self.declare_parameter("match_detail_topic",
                               "/suspect_feature_match_detail")
        self.declare_parameter("target_frame", "map")
        # Only average points from boxes with this Target.type / Roi.type.
        self.declare_parameter("person_label", "person")
        self.declare_parameter("marker_topic", "/suspect_marker")
        self.declare_parameter("pose_topic", "/suspect_pose")
        self.declare_parameter("output_json_path", "/tmp/suspect_location.json")
        self.declare_parameter("marker_scale", 0.3)
        # Guard against a box that lands entirely on invalid-depth pixels.
        self.declare_parameter("min_valid_points", 20)
        self.declare_parameter("tf_timeout_sec", 1.0)
        # pointcloud mode only: how long to wait for one fresh cloud after a
        # capture event before giving up. The cloud is subscribed ON DEMAND
        # (only around a capture), never streamed continuously.
        self.declare_parameter("cloud_wait_sec", 2.0)

        self.location_source = self.get_parameter("location_source").value
        if self.location_source not in ("amcl_pose", "pointcloud"):
            self.get_logger().warn(
                f"unknown location_source '{self.location_source}', "
                "falling back to 'amcl_pose'.")
            self.location_source = "amcl_pose"
        self.amcl_pose_topic = self.get_parameter("amcl_pose_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.match_topic = self.get_parameter("match_topic").value
        self.match_detail_topic = self.get_parameter("match_detail_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.person_label = self.get_parameter("person_label").value
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.output_json_path = self.get_parameter("output_json_path").value
        self.min_valid_points = int(
            self.get_parameter("min_valid_points").value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_sec").value))
        self.cloud_wait = float(self.get_parameter("cloud_wait_sec").value)

        self._lock = threading.Lock()
        self._latest_map_fix = None  # fix frozen at the last capture moment
        self._latest_detail = ""
        self._latest_amcl = None     # (stamp_ns, PoseWithCovarianceStamped)
        # On-demand single-cloud grab (pointcloud mode only).
        self._cloud_event = threading.Event()
        self._grabbed_cloud = None

        self._cb = ReentrantCallbackGroup()
        cb = self._cb
        self.create_subscription(
            PerceptionTargets, self.detections_topic, self._cb_detections, 10,
            callback_group=cb)
        self.create_subscription(
            String, self.match_detail_topic, self._cb_detail, 10,
            callback_group=cb)

        # tf is only needed for the pointcloud source; the cloud itself is NOT
        # streamed — it is subscribed on demand per capture (see _grab_cloud).
        # amcl_pose mode subscribes only to the (tiny) pose topic.
        self.tf_buffer = None
        if self.location_source == "pointcloud":
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self.amcl_pose_topic,
                self._cb_amcl, 10, callback_group=cb)
        self.create_subscription(
            Bool, self.match_topic, self._cb_match, 10, callback_group=cb)

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter("marker_topic").value, latched)
        # Plain map-frame point, latched so late subscribers (e.g. the Flask
        # dashboard) still get the last known suspect location.
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, latched)

        src_in = (self.amcl_pose_topic if self.location_source == "amcl_pose"
                  else self.pointcloud_topic)
        self.get_logger().info(
            f"Suspect localizer ready [location_source={self.location_source}]."
            f" capture<-{self.detections_topic}, pose<-{src_in}, "
            f"match<-{self.match_topic}. On match=true -> writes "
            f"{self.output_json_path} and publishes a marker in "
            f"'{self.target_frame}'.")

    # ---------------- inputs ----------------

    def _cb_detail(self, msg: String):
        with self._lock:
            self._latest_detail = msg.data

    def _cb_amcl(self, msg: PoseWithCovarianceStamped):
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec))
        with self._lock:
            self._latest_amcl = (stamp_ns, msg)

    def _cb_detections(self, msg: PerceptionTargets):
        """A detections message == a /capture_crop event.

        yolo_detect publishes on this topic when /capture_crop runs (and, in
        live_view, per frame). We freeze the fix at THIS moment and nothing
        recomputes afterwards, so it reflects exactly the frame that produced
        the compared crop. How the point is placed depends on location_source:
          amcl_pose  -> the robot's current /amcl_pose (already map frame).
          pointcloud -> centroid of depth points in the largest person bbox.
        """
        best = None
        best_area = 0
        for t in msg.targets:
            for roi in t.rois:
                label = roi.type or t.type
                if label != self.person_label:
                    continue
                r = roi.rect
                area = int(r.width) * int(r.height)
                if area > best_area:
                    best_area = area
                    best = (int(r.x_offset), int(r.y_offset),
                            int(r.x_offset) + int(r.width),
                            int(r.y_offset) + int(r.height))
        if best is None:
            return

        if self.location_source == "amcl_pose":
            fix = self._fix_from_amcl(best)
        else:
            fix = self._fix_from_pointcloud(best, msg.header)
        if fix is None:
            return

        with self._lock:
            self._latest_map_fix = fix
        p = fix["position_map"]
        self.get_logger().info(
            f"Capture fix frozen ({fix['location_source']}): map "
            f"({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}).")

    def _fix_from_amcl(self, bbox):
        with self._lock:
            amcl = self._latest_amcl
        if amcl is None:
            self.get_logger().warn(
                "capture event but no /amcl_pose received yet; skipping.")
            return None
        _, msg = amcl
        pos = msg.pose.pose.position
        return {
            "location_source": "amcl_pose",
            "frame_id": msg.header.frame_id or self.target_frame,
            "stamp": {"sec": msg.header.stamp.sec,
                      "nanosec": msg.header.stamp.nanosec},
            "position_map": {"x": float(pos.x), "y": float(pos.y),
                             "z": float(pos.z)},
            "bbox_xyxy": list(bbox),
        }

    def _fix_from_pointcloud(self, bbox, det_header):
        cloud = self._grab_cloud()
        if cloud is None:
            self.get_logger().warn(
                f"no cloud on {self.pointcloud_topic} within "
                f"{self.cloud_wait}s of capture; skipping.")
            return None

        centroid_cam = self._centroid_in_bbox(cloud, bbox)
        if centroid_cam is None:
            self.get_logger().warn(
                "capture bbox had too few valid depth points; skipping.")
            return None

        return self._to_target_frame(centroid_cam, cloud, bbox)

    def _grab_cloud(self):
        """Subscribe to the cloud ON DEMAND, take ONE message, unsubscribe.

        The cloud topic is never streamed in steady state; we only pay the
        (bandwidth-heavy) subscription for the moment around a capture.
        """
        self._grabbed_cloud = None
        self._cloud_event.clear()
        sub = self.create_subscription(
            PointCloud2, self.pointcloud_topic, self._on_grab_cloud, 1,
            callback_group=self._cb)
        try:
            got = self._cloud_event.wait(timeout=self.cloud_wait)
        finally:
            self.destroy_subscription(sub)
        return self._grabbed_cloud if got else None

    def _on_grab_cloud(self, msg: PointCloud2):
        if not self._cloud_event.is_set():
            self._grabbed_cloud = msg
            self._cloud_event.set()

    # ---------------- point cloud math ----------------

    def _centroid_in_bbox(self, cloud: PointCloud2, bbox):
        """Mean XYZ of valid points inside the pixel box (camera frame)."""
        if cloud.height <= 1:
            # Unorganized cloud: pixel->index mapping is undefined here.
            self.get_logger().warn(
                "point cloud is not organized (height<=1); cannot map the "
                "pixel bbox to points.", once=True)
            return None

        # Read x, y, z (float32 at offsets 0/4/8) as an organized HxW grid,
        # regardless of the full point_step (padding/rgb are ignored).
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [0, 4, 8],
            "itemsize": cloud.point_step,
        })
        flat = np.frombuffer(cloud.data, dtype=dtype,
                             count=cloud.height * cloud.width)
        grid = flat.reshape(cloud.height, cloud.width)

        x1, y1, x2, y2 = bbox
        x1 = max(0, min(x1, cloud.width))
        x2 = max(0, min(x2, cloud.width))
        y1 = max(0, min(y1, cloud.height))
        y2 = max(0, min(y2, cloud.height))
        if x2 <= x1 or y2 <= y1:
            return None

        block = grid[y1:y2, x1:x2]
        xs = block["x"].reshape(-1).astype(np.float64)
        ys = block["y"].reshape(-1).astype(np.float64)
        zs = block["z"].reshape(-1).astype(np.float64)

        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        n = int(valid.sum())
        if n < self.min_valid_points:
            return None

        return (float(xs[valid].mean()),
                float(ys[valid].mean()),
                float(zs[valid].mean()),
                n)

    def _to_target_frame(self, centroid_cam, cloud: PointCloud2, bbox):
        cx, cy, cz, n = centroid_cam
        src_frame = cloud.header.frame_id
        # Prefer the cloud's own stamp so the pose reflects the robot's pose at
        # the exact capture instant; fall back to latest if tf can't
        # interpolate that far back (e.g. short tf buffer).
        cloud_time = rclpy.time.Time.from_msg(cloud.header.stamp)
        tf = None
        for when in (cloud_time, rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, src_frame, when,
                    timeout=self.tf_timeout)
                break
            except ExtrapolationException:
                continue
            except (LookupException, ConnectivityException) as e:
                self.get_logger().warn(
                    f"tf {src_frame}->{self.target_frame} unavailable: {e}",
                    throttle_duration_sec=5.0)
                return None
        if tf is None:
            self.get_logger().warn(
                f"tf {src_frame}->{self.target_frame} not available at the "
                "capture stamp or latest; skipping.",
                throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        p = R.dot(np.array([cx, cy, cz])) + np.array([t.x, t.y, t.z])

        return {
            "location_source": "pointcloud",
            "frame_id": self.target_frame,
            "stamp": {"sec": cloud.header.stamp.sec,
                      "nanosec": cloud.header.stamp.nanosec},
            "source_frame": src_frame,
            "position_map": {"x": float(p[0]), "y": float(p[1]),
                             "z": float(p[2])},
            "centroid_camera": {"x": cx, "y": cy, "z": cz},
            "bbox_xyxy": list(bbox),
            "num_valid_points": n,
        }

    # ---------------- match trigger ----------------

    def _cb_match(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            fix = self._latest_map_fix
            # Consume the fix: it belongs to exactly one match. This stops a
            # repeated match=true from re-emitting the same location, and stops
            # a later match from firing on a fix left over from a previous
            # capture that produced no new one.
            self._latest_map_fix = None
            detail = self._latest_detail
        if fix is None:
            self.get_logger().warn(
                "match=true but no fresh map-frame fix available "
                "(no bbox, no valid depth in box, or tf missing). "
                "Nothing saved/published.")
            return

        fix = dict(fix)
        fix["match_detail"] = detail
        self._write_json(fix)
        self._publish_marker(fix)
        self._publish_pose(fix)

    def _write_json(self, fix):
        try:
            with open(self.output_json_path, "w") as f:
                json.dump(fix, f, indent=2)
            n = fix.get("num_valid_points")
            src = (f"{n} points" if n is not None
                   else fix.get("location_source", "unknown"))
            self.get_logger().info(
                f"Suspect located at map ({fix['position_map']['x']:.2f}, "
                f"{fix['position_map']['y']:.2f}, "
                f"{fix['position_map']['z']:.2f}) from "
                f"{src} -> {self.output_json_path}")
        except OSError as e:
            self.get_logger().error(
                f"could not write {self.output_json_path}: {e}")

    def _publish_marker(self, fix):
        m = Marker()
        m.header.frame_id = fix.get("frame_id", self.target_frame)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "suspect"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        pos = fix["position_map"]
        m.pose.position = Point(x=pos["x"], y=pos["y"], z=pos["z"])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.marker_scale
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        # 0 = never auto-delete; the last known suspect fix stays in RViz.
        m.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(m)

    def _publish_pose(self, fix):
        ps = PoseStamped()
        ps.header.frame_id = fix.get("frame_id", self.target_frame)
        ps.header.stamp = self.get_clock().now().to_msg()
        pos = fix["position_map"]
        ps.pose.position = Point(x=pos["x"], y=pos["y"], z=pos["z"])
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = SuspectLocalizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
