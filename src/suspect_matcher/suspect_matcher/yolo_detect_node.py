#!/usr/bin/env python3
"""
yolo_detect_node.py

FAST person detector: drop-in alternative to yoloworld_detect_node.py that
uses a plain closed-vocabulary Ultralytics YOLO detector (yolo11n / yolov8n
detect) instead of open-vocabulary YOLO-World.

Why this is faster: you only ever detect "person". YOLO-World carries text-
embedding machinery so it can detect arbitrary prompt words -- flexibility you
don't use. A plain COCO detector where person = class 0 runs roughly an order
of magnitude faster on the X5 BPU (yolo11n/yolov8n detect benchmark in the
hundreds of FPS vs YOLO-World's ~30-47 FPS). Detection was already the fast
part of the pipeline (the VLM dominates end-to-end time), so this mainly makes
the live view smoother and lowers BPU load -- it won't dramatically shorten a
full capture->compare cycle.

Behavior is otherwise IDENTICAL to yoloworld_detect_node.py:
  - subscribes to a camera topic, caches the latest frame
  - /capture_crop (std_srvs/Trigger) -> detect on latest frame, crop largest
    person, save <save_dir>/<save_basename>_crop.jpg
  - publishes ai_msgs/PerceptionTargets on detections_topic
  - optional live_view:=true -> annotated stream on annotated_topic
So attribute_compare_from_files_node.py consumes its output unchanged.

Wraps the rdk_model_zoo sample's UltralyticsYOLODetect wrapper, mirroring
runtime/python/main.py's detect path:
    cfg = UltralyticsYOLODetectConfig(model_path=..., classes_num=80,
             score_thres=..., nms_thres=..., reg=16, resize_type=1,
             strides=[8,16,32])
    model = UltralyticsYOLODetect(cfg)
    model.set_scheduling_params(priority=..., bpu_cores=[...])
    boxes, scores, cls_ids = model.predict(img)   # cls_id 0 == person (COCO)

Usage:
  ros2 run suspect_matcher yolo_detect --ros-args \
    -p model_path:=/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo/model/yolo11n_detect_bayese_640x640_nv12.bin \
    -p camera_topic:=/camera/color/image_raw \
    -p save_basename:=candidate

  ros2 service call /capture_crop std_srvs/srv/Trigger {}
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

# COCO class index for "person".
PERSON_CLASS_ID = 0


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__("yolo_detect_node")

        sample_root = "/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo"

        self.declare_parameter(
            "model_path",
            os.path.join(sample_root,
                         "model/yolo11n_detect_bayese_640x640_nv12.bin"))
        self.declare_parameter(
            "sample_runtime_dir", os.path.join(sample_root, "runtime/python"))
        self.declare_parameter("repo_root", "/home/sunrise/rdk_model_zoo")

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("save_dir", "/tmp")
        self.declare_parameter("save_basename", "candidate")
        # COCO id to keep. 0 = person. Change if you retrained/relabeled.
        self.declare_parameter("target_class_id", PERSON_CLASS_ID)
        self.declare_parameter("class_label", "person")

        # Detector config (mirrors main.py detect defaults).
        self.declare_parameter("classes_num", 80)
        self.declare_parameter("score_thres", 0.25)
        self.declare_parameter("nms_thres", 0.70)
        self.declare_parameter("reg", 16)
        self.declare_parameter("resize_type", 1)   # 1 = letterbox
        self.declare_parameter("strides", [8, 16, 32])

        self.declare_parameter("keep_conf", 0.25)
        self.declare_parameter("bpu_cores", [0])
        self.declare_parameter("priority", 0)
        self.declare_parameter("crop_padding_frac", 0.1)
        self.declare_parameter("detections_topic", "/yolo/detections")

        # Live view
        self.declare_parameter("live_view", False)
        self.declare_parameter("annotated_topic", "/yolo/image_annotated")
        self.declare_parameter("view_max_fps", 10.0)

        self.model_path = self.get_parameter("model_path").value
        self.sample_runtime_dir = self.get_parameter("sample_runtime_dir").value
        self.repo_root = self.get_parameter("repo_root").value
        self.save_dir = self.get_parameter("save_dir").value
        self.save_basename = self.get_parameter("save_basename").value
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.class_label = self.get_parameter("class_label").value
        self.classes_num = int(self.get_parameter("classes_num").value)
        self.score_thres = self.get_parameter("score_thres").value
        self.nms_thres = self.get_parameter("nms_thres").value
        self.reg = int(self.get_parameter("reg").value)
        self.resize_type = int(self.get_parameter("resize_type").value)
        self.strides = [int(s) for s in self.get_parameter("strides").value]
        self.keep_conf = self.get_parameter("keep_conf").value
        self.bpu_cores = list(self.get_parameter("bpu_cores").value)
        self.priority = self.get_parameter("priority").value
        self.crop_padding_frac = self.get_parameter("crop_padding_frac").value
        self.view_max_fps = self.get_parameter("view_max_fps").value
        camera_topic = self.get_parameter("camera_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value

        self.bridge = CvBridge()
        self._infer_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._latest_frame = None
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
            if self.get_parameter("live_view").value else
            "live_view OFF (frames cached; inference on /capture_crop)")
        self.get_logger().info(
            f"Fast YOLO detector ready (keeping class {self.target_class_id}="
            f"'{self.class_label}'). {view_note}. Call 'ros2 service call "
            "/capture_crop std_srvs/srv/Trigger {}' to save the largest crop "
            f"to '{os.path.join(self.save_dir, self.save_basename)}_crop.jpg'.")

    # ---------------- model load ----------------

    def _load_model(self):
        for p in (self.sample_runtime_dir, self.repo_root):
            if p and p not in sys.path:
                sys.path.insert(0, p)

        try:
            from ultralytics_yolo_det import (
                UltralyticsYOLODetect, UltralyticsYOLODetectConfig)
        except Exception as e:
            self.get_logger().error(
                "Could not import UltralyticsYOLODetect/Config from "
                f"'{self.sample_runtime_dir}'. Check 'sample_runtime_dir' and "
                f"'repo_root' params. Import error: {e}")
            raise

        cfg = UltralyticsYOLODetectConfig(
            model_path=self.model_path,
            classes_num=self.classes_num,
            score_thres=self.score_thres,
            nms_thres=self.nms_thres,
            reg=self.reg,
            resize_type=self.resize_type,
            strides=self.strides,
        )
        self.model = UltralyticsYOLODetect(cfg)
        self.model.set_scheduling_params(
            priority=self.priority, bpu_cores=self.bpu_cores)
        self.get_logger().info(f"Loaded YOLO detect BPU model: {self.model_path}")

    # ---------------- inference ----------------

    def _run_inference(self, bgr):
        with self._infer_lock:
            boxes, scores, cls_ids = self.model.predict(bgr)
        return boxes, scores, cls_ids

    def _collect_boxes(self, bgr, boxes, scores, cls_ids):
        """Keep only target class above keep_conf; clamp; build detections."""
        h, w = bgr.shape[:2]
        kept = []
        det_msg = PerceptionTargets()
        for box, score, cid in zip(boxes, scores, cls_ids):
            if int(cid) != self.target_class_id:
                continue
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
            t.type = self.class_label
            roi = Roi()
            roi.type = self.class_label
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
            cv2.putText(out, f"{self.class_label}: {score:.2f}",
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

        if not self.get_parameter("live_view").value:
            return

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
                f"no '{self.class_label}' detected above conf {self.keep_conf}")
            return response

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
            f"saved largest '{self.class_label}' (conf {score:.2f}, "
            f"{len(kept)} found) -> {out_path}")
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
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
