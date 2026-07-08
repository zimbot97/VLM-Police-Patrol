#!/usr/bin/env python3
"""
yoloworld_detect_node.py

Loads the YOLO-World .bin model on the RDK X5 BPU (via the rdk_model_zoo
sample's own YOLOWorldDetect wrapper), subscribes to a camera image topic,
and:
  - on a SERVICE CALL, detects + crops the largest person and saves it to disk
    (feeding attribute_compare_from_files_node.py); and
  - optionally (live_view:=true) runs detection on every frame and publishes
    an annotated image (bounding boxes) for live viewing in rqt_image_view /
    the hobot websocket display.

  camera/color/image_raw --> [cache latest frame]
        |                              |
        | (live_view:=true)            | ros2 service call /capture_crop
        v                              v
  detect every frame            detect on latest frame,
  publish annotated image       crop largest, save <basename>_crop.jpg
  on <annotated_topic>

IMPORTANT trade-off:
  live_view runs BPU inference on EVERY frame, which competes for the BPU and
  raises power/heat. If you only need the crop-on-demand behavior, leave
  live_view:=false (default) and the node just caches frames cheaply, running
  inference only when /capture_crop is called. Turn live_view on when you
  actually want to watch boxes in real time (e.g. aiming the camera).

  live_view detection also throttles to 'view_max_fps' so it doesn't try to
  infer faster than the BPU/CPU can sustain.

This wraps the sample's tested inference/decode (yoloworld_det.py); it mirrors
the sample runtime/python/main.py usage:
    config = YOLOWorldConfig(model_path=..., vocab_file=..., score_thres=...,
                             nms_thres=...)
    model = YOLOWorldDetect(config)
    model.set_scheduling_params(priority=..., bpu_cores=[...])
    boxes, scores, cls_ids = model.predict(image, prompts)   # boxes in orig px

Usage:
  # crop-on-demand only (cheap; no continuous inference)
  ros2 run suspect_matcher yoloworld_detect --ros-args \
    -p camera_topic:=/camera/color/image_raw

  # with live annotated view
  ros2 run suspect_matcher yoloworld_detect --ros-args \
    -p camera_topic:=/camera/color/image_raw \
    -p live_view:=true

  ros2 service call /capture_crop std_srvs/srv/Trigger {}

  # view the annotated stream (default topic /yoloworld/image_annotated):
  ros2 run rqt_image_view rqt_image_view /yoloworld/image_annotated
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets, Target, Roi
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from std_srvs.srv import Trigger


class YoloWorldDetectNode(Node):

    def __init__(self):
        super().__init__("yoloworld_detect_node")

        sample_root = "/home/sunrise/rdk_model_zoo/samples/vision/yoloworld"

        self.declare_parameter(
            "model_path", os.path.join(sample_root, "model/yolo_world.bin"))
        self.declare_parameter(
            "vocab_file",
            os.path.join(sample_root, "test_data/offline_vocabulary_embeddings.json"))
        self.declare_parameter(
            "sample_runtime_dir", os.path.join(sample_root, "runtime/python"))
        self.declare_parameter("repo_root", "/home/sunrise/rdk_model_zoo")

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("save_dir", "/tmp")
        self.declare_parameter("save_basename", "candidate")
        self.declare_parameter("prompt", "person")
        self.declare_parameter("score_thres", 0.05)
        self.declare_parameter("nms_thres", 0.45)
        self.declare_parameter("keep_conf", 0.25)
        self.declare_parameter("bpu_cores", [0])
        self.declare_parameter("priority", 0)
        self.declare_parameter("crop_padding_frac", 0.1)
        self.declare_parameter("detections_topic", "/yoloworld/detections")

        # --- live view ---
        # If true, run inference on every incoming frame (throttled) and
        # publish an annotated image with boxes drawn. Costs continuous BPU.
        self.declare_parameter("live_view", False)
        self.declare_parameter("annotated_topic", "/yoloworld/image_annotated")
        self.declare_parameter("view_max_fps", 5.0)

        self.model_path = self.get_parameter("model_path").value
        self.vocab_file = self.get_parameter("vocab_file").value
        self.sample_runtime_dir = self.get_parameter("sample_runtime_dir").value
        self.repo_root = self.get_parameter("repo_root").value
        self.save_dir = self.get_parameter("save_dir").value
        self.save_basename = self.get_parameter("save_basename").value
        self.prompt = self.get_parameter("prompt").value
        self.score_thres = self.get_parameter("score_thres").value
        self.nms_thres = self.get_parameter("nms_thres").value
        self.keep_conf = self.get_parameter("keep_conf").value
        self.bpu_cores = list(self.get_parameter("bpu_cores").value)
        self.priority = self.get_parameter("priority").value
        self.crop_padding_frac = self.get_parameter("crop_padding_frac").value
        self.live_view = self.get_parameter("live_view").value
        self.view_max_fps = self.get_parameter("view_max_fps").value
        camera_topic = self.get_parameter("camera_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value

        self.bridge = CvBridge()
        self._infer_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._latest_frame = None  # (bgr, header)
        self._last_view_time = 0.0

        self._load_model()

        cb_group = ReentrantCallbackGroup()

        self.det_pub = self.create_publisher(
            PerceptionTargets, detections_topic, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.sub = self.create_subscription(
            Image, camera_topic, self._cb_image, 10, callback_group=cb_group)
        self.srv = self.create_service(
            Trigger, "capture_crop", self._handle_capture,
            callback_group=cb_group)

        view_note = (
            f"live_view ON (annotating -> '{annotated_topic}', "
            f"<= {self.view_max_fps} fps)"
            if self.live_view else
            "live_view OFF (frames cached only; inference on /capture_crop)")
        self.get_logger().info(
            f"YOLO-World detector ready (prompt='{self.prompt}'). {view_note}. "
            "Call 'ros2 service call /capture_crop std_srvs/srv/Trigger {}' to "
            f"save the largest crop to "
            f"'{os.path.join(self.save_dir, self.save_basename)}_crop.jpg'.")

    # ---------------- model load ----------------

    def _load_model(self):
        for p in (self.sample_runtime_dir, self.repo_root):
            if p and p not in sys.path:
                sys.path.insert(0, p)

        try:
            from yoloworld_det import YOLOWorldConfig, YOLOWorldDetect
        except Exception as e:
            self.get_logger().error(
                "Could not import YOLOWorldConfig/YOLOWorldDetect from "
                f"'{self.sample_runtime_dir}'. Check the 'sample_runtime_dir' "
                f"and 'repo_root' params. Import error: {e}")
            raise

        config = YOLOWorldConfig(
            model_path=self.model_path,
            vocab_file=self.vocab_file,
            score_thres=self.score_thres,
            nms_thres=self.nms_thres,
        )
        self.model = YOLOWorldDetect(config)
        self.model.set_scheduling_params(
            priority=self.priority, bpu_cores=self.bpu_cores)
        self.get_logger().info(f"Loaded YOLO-World BPU model: {self.model_path}")

    # ---------------- inference ----------------

    def _run_inference(self, bgr):
        with self._infer_lock:
            boxes, scores, cls_ids = self.model.predict(bgr, [self.prompt])
        return boxes, scores, cls_ids

    def _collect_boxes(self, bgr, boxes, scores, cls_ids):
        """Filter by keep_conf, clamp to image, return list of
        (x1,y1,x2,y2,score) and build a PerceptionTargets message."""
        h, w = bgr.shape[:2]
        kept = []
        det_msg = PerceptionTargets()
        for box, score, _cid in zip(boxes, scores, cls_ids):
            score = float(score)
            if score < self.keep_conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in box]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            kept.append((x1, y1, x2, y2, score))

            t = Target()
            t.type = self.prompt
            roi = Roi()
            roi.type = self.prompt
            r = RegionOfInterest()
            r.x_offset, r.y_offset = x1, y1
            r.width, r.height = x2 - x1, y2 - y1
            roi.rect = r
            roi.confidence = score
            t.rois = [roi]
            det_msg.targets.append(t)
        return kept, det_msg

    def _draw(self, bgr, kept):
        out = bgr.copy()
        for (x1, y1, x2, y2, score) in kept:
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(out, f"{self.prompt}: {score:.2f}",
                        (x1, max(y1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2, cv2.LINE_AA)
        return out

    # ---------------- ROS callbacks ----------------

    def _cb_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return
        with self._frame_lock:
            self._latest_frame = (bgr, msg.header)

        # Read live_view fresh so `ros2 param set ... live_view true` takes
        # effect at runtime without restarting the node.
        if not self.get_parameter("live_view").value:
            return

        # Throttle live inference to view_max_fps.
        now = time.time()
        if self.view_max_fps > 0 and \
                (now - self._last_view_time) < (1.0 / self.view_max_fps):
            return
        self._last_view_time = now

        try:
            boxes, scores, cls_ids = self._run_inference(bgr)
            kept, det_msg = self._collect_boxes(bgr, boxes, scores, cls_ids)
        except Exception as e:
            self.get_logger().warn(f"live inference failed: {e}")
            return

        det_msg.header = msg.header
        self.det_pub.publish(det_msg)

        annotated = self._draw(bgr, kept)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.annotated_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"annotated publish failed: {e}")

    def _handle_capture(self, request, response):
        with self._frame_lock:
            cached = self._latest_frame

        if cached is None:
            response.success = False
            response.message = "no camera frame received yet"
            return response

        bgr, header = cached

        try:
            boxes, scores, cls_ids = self._run_inference(bgr)
        except Exception as e:
            response.success = False
            response.message = f"inference failed: {e}"
            return response

        kept, det_msg = self._collect_boxes(bgr, boxes, scores, cls_ids)
        det_msg.header = header
        self.det_pub.publish(det_msg)

        # Also publish an annotated snapshot so you can see what was captured.
        try:
            annotated = self._draw(bgr, kept)
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = header
            self.annotated_pub.publish(out_msg)
        except Exception:
            pass

        if not kept:
            response.success = False
            response.message = (
                f"no '{self.prompt}' detected above conf {self.keep_conf}")
            return response

        # Largest by area.
        best = max(kept, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        x1, y1, x2, y2, score = best
        h, w = bgr.shape[:2]
        pad = int(self.crop_padding_frac * (y2 - y1))
        yy1, yy2 = max(0, y1 - pad), min(h, y2 + pad)
        xx1, xx2 = max(0, x1 - pad), min(w, x2 + pad)
        crop = bgr[yy1:yy2, xx1:xx2]
        if crop.size == 0:
            response.success = False
            response.message = "crop was empty after padding/clamping"
            return response

        out_path = os.path.join(self.save_dir, f"{self.save_basename}_crop.jpg")
        if not cv2.imwrite(out_path, crop):
            response.success = False
            response.message = f"cv2.imwrite failed for {out_path}"
            return response

        response.success = True
        response.message = (
            f"saved largest '{self.prompt}' (conf {score:.2f}, "
            f"{len(kept)} found) -> {out_path}")
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldDetectNode()
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
