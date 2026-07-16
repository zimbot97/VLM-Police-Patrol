# Benchmarks — Real-Time AI Inference (Challenge 2)

Measured on the **D-Robotics RDK X5** (Sunrise 5 / BPU), ROS 2 Humble.

> Fill in the columns below from a live run — capture FPS/latency with
> `ros2 topic hz /yolo/detections` and the node's own timing logs. Numbers shown
> are indicative targets for this platform; replace with your measured values.

## Perception models

| Model | Task | Engine | Input res | Precision | Latency (ms) | Throughput (FPS) | Tool / runtime |
|-------|------|--------|-----------|-----------|--------------|------------------|----------------|
| YOLO11n (`yolo11n_detect_bayese_640x640_nv12.bin`) | Person detection | **BPU** | 640×640 (NV12) | int8 (bayes-e) | _TBD_ | _~30_ | `hobot_dnn` / RDK model zoo |
| InternVL2.5-1B (ViT `vit_model_int16_v2.bin`) | VLM vision encoder | **BPU** | — | int16 | _TBD_ | — | `hobot_llamacpp` |
| Qwen2.5-0.5B-Instruct (`Q4_0.gguf`) | VLM language head | **CPU** | — | Q4_0 | _TBD_ (tokens/s _TBD_) | — | `hobot_llamacpp` / llama.cpp |

## Tool / firmware versions

| Component | Version |
|-----------|---------|
| OS / RDK image | _TBD_ |
| ROS 2 | Humble |
| `hobot_dnn` / `hobot_llamacpp` | _TBD_ |
| BPU toolchain (OpenExplorer) | _TBD_ |
| Camera driver (`ros2_astra_camera`) | _TBD_ |

## Concurrent workload (multi-task)

Two workloads run concurrently during a demo:

1. **Real-time YOLO person detection** — continuous stream on the **BPU**, annotated
   frames published to `/yolo/image_annotated`.
2. **VLM appearance matching** — triggered on demand; ViT encode on **BPU**, LLM
   decode on **CPU**, so it overlaps detection without stealing the BPU stream.

| Scenario | BPU util | CPU util | Detection FPS held | Notes |
|----------|----------|----------|--------------------|-------|
| Detection only | _TBD_ | _TBD_ | _TBD_ | baseline |
| Detection + VLM compare | _TBD_ | _TBD_ | _TBD_ | VLM decode is CPU-bound |

## Live demo capture

Add a screenshot or short clip of the annotated live stream here:

![Live detection overlay](images/live-detection.png)

_(Capture from the Flask dashboard or `/yolo/image_annotated` and save to
`docs/images/live-detection.png`.)_
