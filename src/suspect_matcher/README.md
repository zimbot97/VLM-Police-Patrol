# suspect_matcher

ROS2 (Humble / TROS) package for **suspect appearance matching** on the RDK X5.

Two nodes:

1. **`yoloworld_detect`** — loads the YOLO-World `.bin` model on the BPU
   (open-vocabulary), subscribes to a camera topic, detects people, and saves
   the largest person crop to disk (feeding node 3).
2. **`yolo_detect`** — FASTER alternative to node 1. Uses a plain
   closed-vocabulary Ultralytics YOLO detector (yolo11n/yolov8n) filtered to
   COCO class 0 = person. Same interface/outputs as node 1; ~10x faster on the
   X5 BPU because it skips YOLO-World's open-vocabulary overhead. Use this if
   you only ever detect people.
3. **`attribute_compare`** — compares two saved person-crop images by asking
   the [`hobot_llamacpp`](https://github.com/D-Robotics/hobot_llamacpp) VLM to
   extract clothing/hairstyle attributes from each, then comparing them.

> **Scope / caveat:** this is surface-feature triage only — clothing, hair,
> build. It is **not** face recognition or confirmed identity. Clothing and
> hairstyle can coincidentally match between different people or change day to
> day. Treat a "match" as "worth a human taking a closer look", not proof.

## How it works

1. You trigger the `/compare_images` service.
2. The node reads the two image files from disk (paths are node parameters).
3. It queries `hobot_llamacpp` once per image for a 3-line attribute report,
   waiting for the single `ai_msgs/PerceptionTargets` message that
   `hobot_llamacpp` publishes per finished response (the generated text is in
   `targets[0].type`).
4. It compares the two attribute sets with simple token overlap.
5. It publishes the result.

### Outputs

- **Service response** (`std_srvs/Trigger`): `success` = did the pipeline run
  without error (files read, model responded, attributes parsed). It does
  **not** mean "matched". `message` = the summary, or the error reason.
- **`/suspect_feature_match`** (`std_msgs/Bool`): the actual yes/no match.
  Published only on a successful run. This is where you read match true/false.
- **`/suspect_feature_match_detail`** (`std_msgs/String`): human-readable
  per-field breakdown.

| Outcome | service `success` | `/suspect_feature_match` |
|---|---|---|
| Match | `True` | `true` |
| No match | `True` | `false` |
| Model gave no parseable attributes | `False` | (nothing) |
| Bad file path / params unset | `False` | (nothing) |

## Build

Place this package in your workspace `src/`, then:

```bash
cd ~/your_ws
colcon build --merge-install --packages-select suspect_matcher
source install/setup.bash
```

## Run

### 0. YOLO-World detector — capture person crops

```bash
ros2 run suspect_matcher yoloworld_detect --ros-args \
  -p model_path:=/home/sunrise/rdk_model_zoo/samples/vision/yoloworld/model/yolo_world.bin \
  -p vocab_file:=/home/sunrise/rdk_model_zoo/samples/vision/yoloworld/test_data/offline_vocabulary_embeddings.json \
  -p camera_topic:=/camera/color/image_raw \
  -p save_dir:=/tmp \
  -p save_basename:=candidate
```

This wraps the rdk_model_zoo sample's own `YOLOWorldDetect` class (from
`yoloworld_det.py`), so preprocessing/decode/NMS are the sample's tested code,
not a re-implementation. It's open-vocabulary: the node prompts the model with
the word `person` (change via `-p prompt:=...`), so every returned box is that
class — no COCO class-index mapping needed.

It caches the latest camera frame and saves a crop only when you call its
service. Each call runs detection on the most recent frame and writes the
largest detected person to `<save_dir>/<save_basename>_crop.jpg` (overwriting):

```bash
ros2 service call /capture_crop std_srvs/srv/Trigger {}
# response.message e.g. "saved largest 'person' (conf 0.82, 2 found) -> /tmp/candidate_crop.jpg"
```

To capture a reference image, change the basename first and call again:
```bash
ros2 param set /yoloworld_detect_node save_basename reference
ros2 service call /capture_crop std_srvs/srv/Trigger {}
```

Service responses: `success=True` with the save path on success;
`success=False` if no frame received yet, no person detected, or inference
failed (message says which).

#### Live annotated view (optional)

By default the node just caches frames and only runs inference on
`/capture_crop` (cheap). To watch bounding boxes in real time — useful when
aiming the camera — enable `live_view`:

```bash
ros2 run suspect_matcher yoloworld_detect --ros-args \
  -p camera_topic:=/camera/color/image_raw \
  -p live_view:=true \
  -p view_max_fps:=5.0
```

Then view the annotated stream:
```bash
ros2 run rqt_image_view rqt_image_view /yoloworld/image_annotated
```
(or point the hobot websocket display at `/yoloworld/image_annotated`).

> **Cost note:** `live_view:=true` runs BPU inference on *every* frame (up to
> `view_max_fps`, default 5), competing for the BPU and raising power/heat.
> Leave it off for normal crop-on-demand use. Even with it off, each
> `/capture_crop` still publishes one annotated snapshot to
> `/yoloworld/image_annotated` so you can see what was captured.

You can toggle it at runtime:
```bash
ros2 param set /yoloworld_detect_node live_view true
```

> **Import paths:** the node adds the sample's `runtime/python` dir (for
> `yoloworld_det`) and the repo root (so the wrapper's own
> `import utils.py_utils...` resolves) to `sys.path`. If your checkout isn't at
> `/home/sunrise/rdk_model_zoo`, override `-p sample_runtime_dir:=...` and
> `-p repo_root:=...` to match.

Requires `bpu_infer_lib_x5` (used internally by the sample wrapper):
```bash
pip install bpu_infer_lib_x5 -i http://sdk.d-robotics.cc:8080/simple/ \
  --trusted-host sdk.d-robotics.cc
```

### 0b. Faster alternative — plain YOLO person detector

If you only ever detect people, `yolo_detect` is a drop-in, ~10x-faster
replacement for `yoloworld_detect`. It uses a closed-vocabulary
yolo11n/yolov8n model and keeps only COCO class 0 (person). Same
`/capture_crop` service, same crop output, same optional `live_view`.

```bash
ros2 run suspect_matcher yolo_detect --ros-args \
  -p model_path:=/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo/model/yolo11n_detect_bayese_640x640_nv12.bin \
  -p camera_topic:=/camera/color/image_raw \
  -p save_basename:=candidate \
  -p live_view:=true
```

Download a detect model first if you don't have one:
```bash
cd ~/rdk_model_zoo/samples/vision/ultralytics_yolo/model
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/ultralytics_YOLO/yolo11n_detect_bayese_640x640_nv12.bin
```

Use the **n** (nano) model for max speed. Its annotated stream is on
`/yolo/image_annotated` and detections on `/yolo/detections` (distinct from
the yoloworld topics, so either detector can run without collision). It filters
to `target_class_id:=0` (person) by default.

> **Reality check:** this speeds up detection, which was already the fast part.
> A full capture→compare cycle is dominated by the VLM step, so this mainly
> gives smoother live view and lower BPU load — not dramatically faster
> comparisons.

### 1. Start hobot_llamacpp (separately)

`hobot_llamacpp` needs its `config/` directory (with the model files) in its
working directory, so stage it first per its own README:

```bash
cp -r install/lib/hobot_llamacpp/config/ .
ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
  -p feed_type:=1 -p model_type:=0 \
  -p model_file_name:=vit_model_int16_v2.bin \
  -p llm_model_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf \
  -p system_prompt:="config/system_prompt.txt" \
  --log-level warn
```

(A concise `system_prompt.txt` is included in this package under `config/`;
copy it into hobot_llamacpp's `config/` if you want to use it, or point
`-p system_prompt:=` at it directly.)

### 2. Watch the match result

```bash
ros2 topic echo /suspect_feature_match
```

### 3. Launch this node

```bash
ros2 launch suspect_matcher compare.launch.py \
  reference_image_path:=/tmp/reference_crop.jpg \
  candidate_image_path:=/tmp/candidate_crop.jpg
```

or run it directly:

```bash
ros2 run suspect_matcher attribute_compare --ros-args \
  -p reference_image_path:=/tmp/reference_crop.jpg \
  -p candidate_image_path:=/tmp/candidate_crop.jpg
```

### 4. Trigger a comparison

```bash
ros2 service call /compare_images std_srvs/srv/Trigger {}
```

The **first** call includes hobot_llamacpp's cold model load (observed
5–11 minutes for InternVL2.5-1B on the X5) — it will block, that's expected.
Subsequent calls are fast.

To compare a new pair without restarting, update the parameter and call again:

```bash
ros2 param set /attribute_compare_from_files_node candidate_image_path /tmp/new_crop.jpg
ros2 service call /compare_images std_srvs/srv/Trigger {}
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `reference_image_path` | `""` | Path to reference person-crop image. |
| `candidate_image_path` | `""` | Path to candidate person-crop image. |
| `hobot_image_topic` | `/image` | hobot_llamacpp image input topic. |
| `hobot_prompt_topic` | `/prompt_text` | hobot_llamacpp prompt input topic. |
| `hobot_result_topic` | `/llama_cpp_node` | hobot_llamacpp result topic (PerceptionTargets). |
| `result_topic` | `/suspect_feature_match` | Bool match output. |
| `detail_topic` | `/suspect_feature_match_detail` | String breakdown output. |
| `response_timeout_sec` | `900.0` | Per-query timeout (high to cover cold load). |

## Tuning match strictness

Edit `MIN_FIELD_MATCHES` at the top of
`suspect_matcher/attribute_compare_from_files_node.py` (default 2 of 3 fields
must agree). Raise to 3 for stricter matching. If the model uses different
words for the same thing ("navy" vs "dark blue"), extend `normalize_tokens()`
with a small synonym map rather than expecting the VLM to be consistent.
