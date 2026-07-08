#!/usr/bin/env python3
"""
attribute_compare_from_files_node.py

Suspect appearance-matching node (Option B), operating on two static image
FILES rather than a live camera feed. Intended to be used after an upstream
detector (e.g. the YOLO-World sample in rdk_model_zoo) has already cropped
the "person" region out of a frame and saved it as a JPG/PNG.

This node does NOT run any detection itself — it assumes cropping already
happened upstream (in your yoloworld script) and just re-reads whatever file
paths its parameters currently point at each time you trigger a comparison.
That keeps this node decoupled from the detector's specific API, so you can
swap detectors later without touching this file.

Pipeline per trigger:
  1. cv2.imread(reference_image_path) -> crop A
  2. cv2.imread(candidate_image_path) -> crop B
  3. Query hobot_llamacpp for structured attributes (clothing_color,
     clothing_type, hairstyle) on crop A, wait for its response, then query
     crop B. hobot_llamacpp only handles one image+prompt per inference
     cycle, so this is inherently sequential, not parallel. Completion is
     detected deterministically: hobot_llamacpp publishes exactly one
     ai_msgs/PerceptionTargets message per finished response on its
     ai_msg_pub_topic_name (default /llama_cpp_node), with the generated
     text in targets[0].type. No silence-timeout heuristic or inter-query
     delay is needed — the node blocks on that single message per query.
  4. Compare the two attribute dicts with plain Python token overlap.
  5. Publish the match result and a breakdown (see Outputs below).

Outputs (two separate things — don't confuse them):
  - Service response (std_srvs/Trigger):
      success = did the PIPELINE run without error (files read, model
                responded, attributes parsed). NOT the match result.
      message = the human-readable summary, or the error reason if
                success is False.
  - Topics:
      <result_topic>  (default /suspect_feature_match, std_msgs/Bool)
          = the actual yes/no MATCH result. Published only when the
            pipeline succeeded. THIS is where you read match true/false.
      <detail_topic>  (default /suspect_feature_match_detail, std_msgs/String)
          = the same human-readable breakdown as the service message.

  So: check response.success to know the comparison actually ran, then
  read /suspect_feature_match for the boolean match. A "no match" is
  success=True (pipeline ran fine) with /suspect_feature_match = false;
  a broken pipeline is success=False and nothing published on the topic.

Usage:
  1. Launch hobot_llamacpp separately (feed_type:=1), same as before, with a
     system_prompt instructing concise, format-following answers.

  2. Run this node, pointing at your two crop files:
       ros2 run <your_pkg> attribute_compare_from_files_node.py \
         --ros-args -p reference_image_path:=/tmp/reference_crop.jpg \
                     -p candidate_image_path:=/tmp/candidate_crop.jpg

  3. In a separate terminal, watch the boolean match result:
       ros2 topic echo /suspect_feature_match
       # prints "data: true" or "data: false" on each successful comparison

  4. Trigger a comparison:
       ros2 service call /compare_images std_srvs/srv/Trigger {}
       # response.success = pipeline ran OK; the match bool is on the topic above

     To compare a NEW pair without restarting the node, just update the
     parameters and call the service again:
       ros2 param set /attribute_compare_from_files_node candidate_image_path /tmp/new_crop.jpg
       ros2 service call /compare_images std_srvs/srv/Trigger {}

     (Paths are re-read from disk fresh on every trigger — nothing is cached
     from a previous run.)

Caveats:
  - This is clothing/hairstyle/build matching only — not face recognition or
    confirmed identity. Treat "yes" as "worth a closer look", not proof.
  - Crop quality matters a lot here: a loose/tight crop, motion blur, or a
    partially-occluded person will degrade the VLM's attribute extraction.
    If results look unreliable, first check /tmp saved crops visually before
    assuming the VLM or the comparison logic is at fault.
  - Cold-start: this node does NOT warm the model up on its own — it starts
    idle and only queries hobot_llamacpp when you call /compare_images.
    hobot_llamacpp itself doesn't load the model at ITS startup either (with
    feed_type:=1, it loads on the first image it receives), so whichever
    /compare_images call happens to be first will simply take a long time
    (observed 5-11 minutes for InternVL2_5-1B on RDK X5) while it blocks
    waiting for that cold load + first inference. That's expected, not a
    hang — just let it sit. 'response_timeout_sec' defaults to 900s to
    accommodate this. Every call after the first should be fast (a few
    seconds each), since the model stays loaded in hobot_llamacpp's process.
  - Total latency for a real comparison (once warm) is roughly 2x a single
    VLM query, since it runs two sequential inferences. If a query ever
    exceeds 'response_timeout_sec' (once warm), raise it — but note the
    default 900s is set high for the cold first call, so this is unlikely.
"""

import re
import threading
import time

import cv2
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

MIN_FIELD_MATCHES = 2

FIELDS = ["clothing_color", "clothing_type", "hairstyle"]

ATTRIBUTE_PROMPT = (
    "Look at the person in this photo. Report exactly three lines in this "
    "exact format, with no other text:\n"
    "clothing_color: <one or two words>\n"
    "clothing_type: <one or two words, e.g. jacket, t-shirt, dress, jeans>\n"
    "hairstyle: <one or two words, e.g. short black hair, long ponytail, bald>"
)

STOPWORDS = {"a", "an", "the", "colored", "color", "coloured", "with", "and"}

FIELD_PATTERNS = {
    field: re.compile(rf"{field}\s*[:\-]\s*([^\n,]+)", re.IGNORECASE)
    for field in FIELDS
}


def normalize_tokens(value: str):
    value = value.lower().strip()
    tokens = re.findall(r"[a-z]+", value)
    return {t for t in tokens if t not in STOPWORDS}


def parse_attributes(text: str) -> dict:
    result = {}
    for field, pattern in FIELD_PATTERNS.items():
        m = pattern.search(text)
        value = m.group(1).strip() if m else ""
        # Strip llama.cpp end-of-turn / special tokens that leak into the
        # generated text (e.g. "short black hair</s>").
        value = re.sub(r"</?s>|<\|.*?\|>", "", value).strip()
        result[field] = value
    return result


def compare_attributes(ref: dict, cand: dict):
    per_field = {}
    n_matched = 0
    for field in FIELDS:
        ref_tokens = normalize_tokens(ref.get(field, ""))
        cand_tokens = normalize_tokens(cand.get(field, ""))
        matched = bool(ref_tokens) and bool(cand_tokens) and \
            bool(ref_tokens & cand_tokens)
        per_field[field] = matched
        if matched:
            n_matched += 1
    return (n_matched >= MIN_FIELD_MATCHES), per_field, n_matched


class AttributeCompareFromFilesNode(Node):

    def __init__(self):
        super().__init__("attribute_compare_from_files_node")

        self.declare_parameter("reference_image_path", "")
        self.declare_parameter("candidate_image_path", "")
        self.declare_parameter("hobot_image_topic", "/image")
        self.declare_parameter("hobot_prompt_topic", "/prompt_text")
        # hobot_llamacpp's final-result topic (ai_msg_pub_topic_name). It
        # publishes ONE ai_msgs/PerceptionTargets per finished response, with
        # the generated text in targets[0].type — a real completion signal,
        # so no silence-timeout guessing is needed.
        self.declare_parameter("hobot_result_topic", "/llama_cpp_node")
        self.declare_parameter("result_topic", "/suspect_feature_match")
        self.declare_parameter("detail_topic", "/suspect_feature_match_detail")
        # Per-query timeout. Must cover a COLD model load on the first ever
        # call (observed 5-11 minutes for InternVL2_5-1B on RDK X5) since this
        # node doesn't warm the model up on startup.
        self.declare_parameter("response_timeout_sec", 900.0)

        self.response_timeout_sec = self.get_parameter("response_timeout_sec").value

        self.bridge = CvBridge()

        # One complete response per PerceptionTargets message; use an Event
        # to hand it from the subscriber callback to the waiting query call.
        self._response_event = threading.Event()
        self._response_text = ""

        cb_group = ReentrantCallbackGroup()

        hobot_image_topic = self.get_parameter("hobot_image_topic").value
        hobot_prompt_topic = self.get_parameter("hobot_prompt_topic").value
        hobot_result_topic = self.get_parameter("hobot_result_topic").value
        result_topic = self.get_parameter("result_topic").value
        detail_topic = self.get_parameter("detail_topic").value

        self.image_pub = self.create_publisher(Image, hobot_image_topic, 10)
        self.prompt_pub = self.create_publisher(String, hobot_prompt_topic, 10)
        self.result_pub = self.create_publisher(Bool, result_topic, 10)
        self.detail_pub = self.create_publisher(String, detail_topic, 10)

        self.result_sub = self.create_subscription(
            PerceptionTargets, hobot_result_topic, self._cb_result, 10,
            callback_group=cb_group)

        self.srv = self.create_service(
            Trigger, "compare_images", self._handle_compare,
            callback_group=cb_group)

        self.get_logger().info(
            "Ready. Set 'reference_image_path' and 'candidate_image_path' "
            "params (file paths), then call "
            "'ros2 service call /compare_images std_srvs/srv/Trigger {}'. "
            "Note: the FIRST call will include hobot_llamacpp's model load "
            f"time (can take several minutes) — response_timeout_sec is "
            f"set to {self.response_timeout_sec}s to accommodate that.")

    # ---------- hobot_llamacpp result ----------

    def _cb_result(self, msg: PerceptionTargets):
        # One PerceptionTargets = one finished response. The generated text
        # is carried in targets[0].type.
        if not msg.targets:
            return
        self._response_text = msg.targets[0].type
        self._response_event.set()

    # ---------- single-image VLM query ----------

    def _query_attributes(self, cv_image, timeout_sec=None) -> dict:
        if timeout_sec is None:
            timeout_sec = self.response_timeout_sec

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()

        self._response_text = ""
        self._response_event.clear()

        self.prompt_pub.publish(String(data=ATTRIBUTE_PROMPT))
        time.sleep(0.05)
        self.image_pub.publish(image_msg)

        # Block until exactly one complete response arrives, or timeout.
        got = self._response_event.wait(timeout=timeout_sec)
        if not got:
            return {}
        return parse_attributes(self._response_text)

    # ---------- service handler ----------

    def _handle_compare(self, request, response):
        # Response semantics:
        #   response.success = did the inference pipeline RUN without error
        #                      (files read, model responded, attributes parsed).
        #                      It does NOT mean "matched".
        #   response.message = human-readable summary or the error reason.
        # The actual yes/no MATCH result is a separate std_msgs/Bool published
        # on the 'result_topic' (default /suspect_feature_match), and is only
        # published when success is True.
        ref_path = self.get_parameter("reference_image_path").value
        cand_path = self.get_parameter("candidate_image_path").value

        if not ref_path or not cand_path:
            response.success = False
            response.message = ("set 'reference_image_path' and "
                                 "'candidate_image_path' parameters first")
            return response

        ref_img = cv2.imread(ref_path)
        if ref_img is None:
            response.success = False
            response.message = f"could not read reference image: {ref_path}"
            return response

        cand_img = cv2.imread(cand_path)
        if cand_img is None:
            response.success = False
            response.message = f"could not read candidate image: {cand_path}"
            return response

        ref_attrs = self._query_attributes(ref_img)
        if not any(ref_attrs.values()):
            response.success = False
            response.message = "could not extract attributes from reference image"
            return response

        cand_attrs = self._query_attributes(cand_img)
        if not any(cand_attrs.values()):
            response.success = False
            response.message = "could not extract attributes from candidate image"
            return response

        overall_match, per_field, n_matched = compare_attributes(ref_attrs, cand_attrs)

        breakdown = ", ".join(
            f"{field}: {'match' if per_field[field] else 'mismatch'} "
            f"(ref='{ref_attrs[field]}' cand='{cand_attrs[field]}')"
            for field in FIELDS
        )
        summary = (f"{'yes' if overall_match else 'no'} "
                   f"({n_matched}/{len(FIELDS)} fields match) — {breakdown}")

        # Match result goes on its own topic; service success just reports
        # that the pipeline completed.
        self.result_pub.publish(Bool(data=overall_match))
        self.detail_pub.publish(String(data=summary))

        response.success = True
        response.message = summary
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AttributeCompareFromFilesNode()
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
