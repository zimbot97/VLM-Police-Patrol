# suspect_matcher

Person detection, VLM appearance-matching, and map localization for the VLM-Police-Patrol robot (RDK X5).

## Overview

`suspect_matcher` detects people in the camera feed, captures a cropped image of the largest person on demand, and compares that crop against a reference "suspect" photo by asking the on-device `hobot_llamacpp` VLM to extract structured surface attributes (clothing color, clothing type, hairstyle) from each image and then comparing them. When a match is declared, a companion node freezes and reports the suspect's map-frame location. Detection runs on the RDK X5 BPU (a fast closed-vocabulary Ultralytics YOLO detector, or an open-vocabulary YOLO-World detector), while the VLM attribute reasoning runs through `hobot_llamacpp`. This is **surface-feature triage only — clothing/hairstyle/build matching, NOT face recognition or confirmed identity**; a "yes" means "worth a closer look", not proof.

## Nodes / executables

| Executable | Node | Source file | Role |
|---|---|---|---|
| `yolo_detect` | `yolo_detect_node` | [suspect_matcher/yolo_detect_node.py](suspect_matcher/yolo_detect_node.py) | Fast closed-vocab YOLO person detector on BPU; crop-on-demand capture |
| `yoloworld_detect` | `yoloworld_detect_node` | [suspect_matcher/yoloworld_detect_node.py](suspect_matcher/yoloworld_detect_node.py) | Open-vocabulary YOLO-World detector on BPU; crop-on-demand capture |
| `attribute_compare` | `attribute_compare_from_files_node` | [suspect_matcher/attribute_compare_from_files_node.py](suspect_matcher/attribute_compare_from_files_node.py) | VLM attribute extraction + comparison of two image files via `hobot_llamacpp` |
| `suspect_localizer` | `suspect_localizer_node` | [suspect_matcher/suspect_localizer_node.py](suspect_matcher/suspect_localizer_node.py) | Fuses detection bbox + amcl pose or depth cloud into a map-frame suspect location |

## Node details

### yolo_detect (`yolo_detect_node`)

Wraps the `rdk_model_zoo` `UltralyticsYOLODetect` wrapper. Caches the latest camera frame; on `/capture_crop` it runs inference on that frame, keeps the largest box of the target class, and saves a padded crop to `<save_dir>/<save_basename>_crop.jpg`. With `live_view:=true` it also runs (throttled) inference on every frame and publishes an annotated image.

Parameters:

| Name | Type | Default |
|---|---|---|
| `model_path` | string | `/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo/model/yolo11n_detect_bayese_640x640_nv12.bin` |
| `sample_runtime_dir` | string | `<sample_root>/runtime/python` |
| `repo_root` | string | `/home/sunrise/rdk_model_zoo` |
| `camera_topic` | string | `/camera/color/image_raw` |
| `save_dir` | string | `/tmp` |
| `save_basename` | string | `candidate` |
| `target_class_id` | int | `0` (COCO person) |
| `class_label` | string | `person` |
| `classes_num` | int | `80` |
| `score_thres` | double | `0.25` |
| `nms_thres` | double | `0.70` |
| `reg` | int | `16` |
| `resize_type` | int | `1` (letterbox) |
| `strides` | int[] | `[8, 16, 32]` |
| `keep_conf` | double | `0.25` |
| `bpu_cores` | int[] | `[0]` |
| `priority` | int | `0` |
| `crop_padding_frac` | double | `0.1` |
| `detections_topic` | string | `/yolo/detections` |
| `live_view` | bool | `False` |
| `annotated_topic` | string | `/yolo/image_annotated` |
| `view_max_fps` | double | `10.0` |

- **Subscribed:** `camera_topic` (`sensor_msgs/Image`)
- **Published:** `detections_topic` (`ai_msgs/PerceptionTargets`), `annotated_topic` (`sensor_msgs/Image`)
- **Services:** `capture_crop` (server, `std_srvs/Trigger`)

### yoloworld_detect (`yoloworld_detect_node`)

Same crop-on-demand / live-view behavior as `yolo_detect`, but wraps the `rdk_model_zoo` `YOLOWorldDetect` open-vocabulary wrapper (text-prompt detection via an offline vocabulary embedding file). Detects whatever the `prompt` param names (default `person`).

Parameters:

| Name | Type | Default |
|---|---|---|
| `model_path` | string | `/home/sunrise/rdk_model_zoo/samples/vision/yoloworld/model/yolo_world.bin` |
| `vocab_file` | string | `<sample_root>/test_data/offline_vocabulary_embeddings.json` |
| `sample_runtime_dir` | string | `<sample_root>/runtime/python` |
| `repo_root` | string | `/home/sunrise/rdk_model_zoo` |
| `camera_topic` | string | `/camera/color/image_raw` |
| `save_dir` | string | `/tmp` |
| `save_basename` | string | `candidate` |
| `prompt` | string | `person` |
| `score_thres` | double | `0.05` |
| `nms_thres` | double | `0.45` |
| `keep_conf` | double | `0.25` |
| `bpu_cores` | int[] | `[0]` |
| `priority` | int | `0` |
| `crop_padding_frac` | double | `0.1` |
| `detections_topic` | string | `/yoloworld/detections` |
| `live_view` | bool | `False` |
| `annotated_topic` | string | `/yoloworld/image_annotated` |
| `view_max_fps` | double | `5.0` |

- **Subscribed:** `camera_topic` (`sensor_msgs/Image`)
- **Published:** `detections_topic` (`ai_msgs/PerceptionTargets`), `annotated_topic` (`sensor_msgs/Image`)
- **Services:** `capture_crop` (server, `std_srvs/Trigger`)

### attribute_compare (`attribute_compare_from_files_node`)

Runs no detection itself. On `/compare_images` it reads two image files fresh from disk, queries `hobot_llamacpp` for structured attributes on each (sequentially — the VLM handles one image+prompt per cycle), parses `clothing_color` / `clothing_type` / `hairstyle`, and compares by normalized token overlap. A match requires at least 2 of 3 fields to overlap (`MIN_FIELD_MATCHES = 2`). The VLM prompt is published as text and the image as an `Image`; completion is detected deterministically from the single `PerceptionTargets` the VLM emits per response (generated text in `targets[0].type`).

Service response semantics: `success` = the pipeline ran without error (not the match result); the boolean match is published separately on `result_topic`.

Parameters:

| Name | Type | Default |
|---|---|---|
| `reference_image_path` | string | `""` |
| `candidate_image_path` | string | `""` |
| `hobot_image_topic` | string | `/image` |
| `hobot_prompt_topic` | string | `/prompt_text` |
| `hobot_result_topic` | string | `/llama_cpp_node` |
| `result_topic` | string | `/suspect_feature_match` |
| `detail_topic` | string | `/suspect_feature_match_detail` |
| `response_timeout_sec` | double | `900.0` (high to cover cold model load, 5–11 min for InternVL2_5-1B) |

- **Subscribed:** `hobot_result_topic` (`ai_msgs/PerceptionTargets`)
- **Published:** `hobot_image_topic` (`sensor_msgs/Image`), `hobot_prompt_topic` (`std_msgs/String`), `result_topic` (`std_msgs/Bool`), `detail_topic` (`std_msgs/String`)
- **Services:** `compare_images` (server, `std_srvs/Trigger`)

### suspect_localizer (`suspect_localizer_node`)

Consumes the detector's published topics (it does not depend on their internals). Treats each `detections_topic` message as a capture event, takes the largest person box, and freezes a map-frame fix. Two placement modes via `location_source`:

- **`amcl_pose`** (default): places the suspect at the robot's own `/amcl_pose` map position (cheap; no cloud/tf).
- **`pointcloud`**: subscribes to the depth cloud on demand, averages valid (finite) XYZ points inside the bbox from the organized cloud to get a camera-frame centroid, then transforms it to `target_frame` with tf2 (using the cloud stamp, falling back to latest).

The frozen fix is emitted only when `match_topic` goes true (the fix is consumed per match). Output is a JSON file, a latched RViz `Marker` sphere, and a latched `PoseStamped`.

> **Limitation:** we run in `amcl_pose` mode because the RDK X5 CPU already sits ~90 % under the full stack + VLM (see [docs/technical.md](../../docs/technical.md#3-known-issues)), so the heavier `pointcloud` centroid+TF path is avoided. The suspect is therefore reported at the robot's own pose — drive up to whoever you're checking.

Parameters:

| Name | Type | Default |
|---|---|---|
| `location_source` | string | `amcl_pose` (`amcl_pose` or `pointcloud`) |
| `amcl_pose_topic` | string | `/amcl_pose` |
| `detections_topic` | string | `/yolo/detections` |
| `pointcloud_topic` | string | `/camera/depth_registered/points` |
| `match_topic` | string | `/suspect_feature_match` |
| `match_detail_topic` | string | `/suspect_feature_match_detail` |
| `target_frame` | string | `map` |
| `person_label` | string | `person` |
| `marker_topic` | string | `/suspect_marker` |
| `pose_topic` | string | `/suspect_pose` |
| `output_json_path` | string | `/tmp/suspect_location.json` |
| `marker_scale` | double | `0.3` |
| `min_valid_points` | int | `20` |
| `tf_timeout_sec` | double | `1.0` |
| `cloud_wait_sec` | double | `2.0` |

- **Subscribed:** `detections_topic` (`ai_msgs/PerceptionTargets`), `match_detail_topic` (`std_msgs/String`), `match_topic` (`std_msgs/Bool`), `amcl_pose_topic` (`geometry_msgs/PoseWithCovarianceStamped`, amcl_pose mode only), `pointcloud_topic` (`sensor_msgs/PointCloud2`, pointcloud mode, subscribed on demand)
- **Published:** `marker_topic` (`visualization_msgs/Marker`, latched/transient-local), `pose_topic` (`geometry_msgs/PoseStamped`, latched)
- **Services:** none
- **Also writes:** `output_json_path` (JSON) on match

## Launch files

### [launch/compare.launch.py](launch/compare.launch.py)

Starts **only** the `attribute_compare` node (not `hobot_llamacpp`, which must be launched separately). Launch arguments:

| Arg | Default |
|---|---|
| `reference_image_path` | `/tmp/reference_crop.jpg` |
| `candidate_image_path` | `/tmp/candidate_crop.jpg` |
| `response_timeout_sec` | `900.0` |

## Data flow

detect (`yolo_detect` / `yoloworld_detect`) → on `/capture_crop`, save the largest person crop to disk and publish the bbox on `/yolo/detections` → `attribute_compare` reads the reference + candidate crops, queries the `hobot_llamacpp` VLM per image (`/prompt_text` + `/image`, reading completions from `/llama_cpp_node`), parses and compares attributes → publishes the match boolean on `/suspect_feature_match` and a breakdown on `/suspect_feature_match_detail` → `suspect_localizer` freezes a map-frame fix at the capture event and, on match=true, writes `/tmp/suspect_location.json` and publishes `/suspect_marker` and `/suspect_pose`. See the pipeline diagram in [../../docs/technical.md](../../docs/technical.md).

## Dependencies

From `package.xml`: `rclpy`, `sensor_msgs`, `std_msgs`, `std_srvs`, `ai_msgs`, `cv_bridge`, `python3-opencv`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `python3-numpy`.

Notable imports / runtime deps: `cv_bridge` + OpenCV (`cv2`), `numpy`, `ai_msgs/PerceptionTargets`, `tf2_ros` (Buffer/TransformListener). Detector nodes import the RDK `rdk_model_zoo` sample wrappers at runtime (`ultralytics_yolo_det.UltralyticsYOLODetect`, `yoloworld_det.YOLOWorldDetect`) from `sample_runtime_dir` / `repo_root`. The VLM is provided by the external `hobot_llamacpp` node (started separately).

## Build & run

```bash
cd ~/ros2_ws
colcon build --packages-select suspect_matcher
source install/setup.bash
```

Start the VLM (see [sh/llamacpp.sh](../../sh/llamacpp.sh)) — uses InternVL2_5-1B (`vit_model_int16_v2.bin` + `Qwen2.5-0.5B-Instruct-Q4_0.gguf`) with `feed_type:=1 model_type:=0` and `system_prompt:=config/system_prompt.txt`:

```bash
ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
  -p feed_type:=1 -p model_type:=0 \
  -p model_file_name:=/home/sunrise/models/internvl2_5_1b/vit_model_int16_v2.bin \
  -p llm_model_name:=/home/sunrise/models/internvl2_5_1b/Qwen2.5-0.5B-Instruct-Q4_0.gguf \
  -p system_prompt:="config/system_prompt.txt" --log-level warn
```

Run the detector (see [sh/yolo.sh](../../sh/yolo.sh)):

```bash
ros2 run suspect_matcher yolo_detect --ros-args \
  -p model_path:=/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo/model/yolo11n_detect_bayese_640x640_nv12.bin \
  -p camera_topic:=/camera/color/image_raw -p live_view:=true -p keep_conf:=0.7
# capture a crop:
ros2 service call /capture_crop std_srvs/srv/Trigger {}
```

Run the comparison (see [sh/suspect_matcher.sh](../../sh/suspect_matcher.sh)):

```bash
ros2 launch suspect_matcher compare.launch.py \
  reference_image_path:=/tmp/reference_crop.jpg \
  candidate_image_path:=/tmp/candidate_crop.jpg
# trigger it:
ros2 service call /compare_images std_srvs/srv/Trigger {}
ros2 topic echo /suspect_feature_match
```

Run the localizer (see [sh/suspect_localize.sh](../../sh/suspect_localize.sh)):

```bash
ros2 run suspect_matcher suspect_localizer --ros-args \
  -p location_source:=amcl_pose -p min_valid_points:=20 -p tf_timeout_sec:=1.0
# depth mode instead: -p location_source:=pointcloud -p cloud_wait_sec:=2.0
```

## Files

```
suspect_matcher/
├── setup.py                                          # entry points (4 executables)
├── setup.cfg                                         # ament_python script dirs
├── package.xml                                       # deps + metadata
├── config/system_prompt.txt                          # VLM system prompt (concise attribute extraction)
├── launch/compare.launch.py                          # launches attribute_compare only
└── suspect_matcher/
    ├── yolo_detect_node.py                            # fast closed-vocab YOLO detector + crop capture
    ├── yoloworld_detect_node.py                       # open-vocab YOLO-World detector + crop capture
    ├── attribute_compare_from_files_node.py           # VLM attribute extraction + comparison
    └── suspect_localizer_node.py                      # amcl/pointcloud -> map-frame suspect fix
```

---

See [../../README.md](../../README.md) for the full VLM-Police-Patrol robot and [../../docs/technical.md](../../docs/technical.md) for the system architecture and pipeline diagram.
