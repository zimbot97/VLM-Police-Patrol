# Benchmarks — Real-Time AI Inference (Challenge 2)

**Platform:** D-Robotics **RDK X5** — 8× Cortex-A55 @ 1.5 GHz, 8 GB LPDDR4
(~5.2 GB visible after the BPU ION carve-out), 10 TOPS BPU. ROS 2 Humble.

System-level CPU/load figures below are **measured** (htop snapshot, 2026-07-14, full
stack running). Per-model FPS/latency are not yet instrumented — capture with
`ros2 topic hz /yolo/detections` and the nodes' timing logs, then replace the _TBD_ cells.
Full profiling write-up: [system_benchmark_watchdog_analysis.md](system_benchmark_watchdog_analysis.md).

## Perception models

| Model | Task | Engine | Input res | Precision | Latency (ms) | Throughput (FPS) | Tool / runtime |
|-------|------|--------|-----------|-----------|--------------|------------------|----------------|
| YOLO11n (`yolo11n_detect_bayese_640x640_nv12.bin`) | Person detection | **BPU** | 640×640 (NV12) | int8 (bayes-e) | _TBD_ | **≈ camera rate (~30)** | `hobot_dnn` / RDK model zoo |
| InternVL2.5-1B (ViT `vit_model_int16_v2.bin`) | VLM vision encoder | **BPU** | — | int16 | _TBD_ | — | `hobot_llamacpp` |
| Qwen2.5-0.5B-Instruct (`Q4_0.gguf`) | VLM language head | **CPU** | — | Q4_0 | _TBD_ (tokens/s _TBD_) | — | `hobot_llamacpp` / llama.cpp |

> **Detection FPS ≈ camera FPS.** The pipeline is camera-bound, not BPU-bound: YOLO11n on
> the BPU keeps up with the `ros2_astra_camera` stream and runs at effectively the same
> frame rate as `/camera/color/image_raw`. The camera runs at the Astra / LeTMC-520 driver
> default (VGA ~30 fps; [camera.launch.py](../src/police_patrol_bot/launch/camera.launch.py)
> sets no explicit rate). Confirm both with `ros2 topic hz /camera/color/image_raw` and
> `ros2 topic hz /yolo/detections` — they should read nearly identical.

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

Even at this load, teleop stays smooth because motion control lives on the RP2040 Pico,
not the X5 CPU.

## System profiling — measured baseline

Snapshot: full patrol stack (camera, YOLO, VLM matcher armed, AMCL/Nav2, EKF, Flask,
OLED, micro-ROS agent) during active patrol.

| Metric | Value | Assessment |
|--------|-------|------------|
| Load average (1 / 5 / 15 min) | **20.75 / 22.05 / 19.22** | 🔴 ~2.6× the 8 cores — sustained saturation |
| CPU utilization (all 8 cores) | **86.6 – 93.6 %** | 🔴 No headroom |
| Memory | 2.65 G / 5.20 G (~51 %) | 🟡 OK; RAM is not the bottleneck |
| Swap | 0 / 0 | 🟡 None configured — OOM kill is the only relief valve |
| Tasks / threads / running | 155 / 469 / 8 | Run queue == core count |

**Top CPU consumers (aggregated per process):**

| Process | Approx. CPU | Note |
|---------|-------------|------|
| `astra_camera_node` | **~290 % (≈3 cores)** | Largest single consumer — RGBD + depth cloud |
| `suspect_localizer` (pointcloud path) | **~150 % (≈1.5 cores)** | Depth/point-cloud lookup — a key reason we run `location_source:=amcl_pose` instead (see below) |
| `ekf_node` | ~110 % | High for an EKF — check input rates |
| `yolo_detect` | ~95 % (≈1 core) | BPU detection + crop |
| `robot_state_publisher` | ~75 % | Likely high-rate `/joint_states` |
| `nav2_amcl` | ~65 % | Normal under active localization |
| `pointcloud_to_laserscan` | ~55 % | |
| `dashboard_flask` | ~22 % | MJPEG + Socket.IO |
| `oled_status` | ~16 % | 15 fps PIL redraws |
| `micro_ros_agent` | ~7 % | Healthy |

**Top RAM:** `suspect_localizer` 635 M · `astra_camera_node` 247 M · `yolo_detect` 190 M
· `dashboard_flask` 136 M. (`hobot_llamacpp` / InternVL weights sit in the ION region,
outside normal RES accounting.)

### Why the localizer runs in `amcl_pose` mode

The profile above is exactly why: the `pointcloud` placement path costs ~1.5 cores on an
already-saturated CPU, so the localizer defaults to `amcl_pose` (report the suspect at the
robot's own pose) — see the limitation in [technical.md §3](technical.md#3-known-issues).

### Optimization backlog (biggest wins first)

From the [full analysis](system_benchmark_watchdog_analysis.md#4-findings--recommendations-priority-order):

1. Cut `astra_camera_node` cost — publish `/camera/depth_registered/points` only if
   needed; drop color/depth to 640×480@15.
2. Replace `pointcloud_to_laserscan` with `depthimage_to_laserscan` (skips full cloud).
3. Audit EKF / `robot_state_publisher` input rates (~1.8 cores combined is suspicious).
4. Boot headless — `sudo systemctl set-default multi-user.target` (drops Xorg overhead).
5. Add a 1–2 G zram swap as an OOM safety net.
6. Verify no duplicate camera/localizer instances from a leftover backgrounded launch.
7. Re-benchmark after each change; record load-avg after 5 min steady patrol.

### Re-benchmark template

Add an "after" row per optimization to quantify each win:

| Config | Load avg (1/5/15) | CPU/core | Mem | `/camera/color/image_raw` Hz | `/scan` Hz | Notes |
|--------|-------------------|----------|-----|------------------------------|-----------|-------|
| Baseline (full stack + VLM armed) | 20.75 / 22.05 / 19.22 | 86.6–93.6 % | 2.65 G / 5.20 G | _TBD_ | _TBD_ | this snapshot |
| _after: camera @640×480×15_ | | | | | | |
| _after: headless (no Xorg)_ | | | | | | |

## Watchdog thresholds (proposed)

For a monitoring node publishing `diagnostic_msgs/DiagnosticArray`:

| Signal | WARN | CRITICAL | Action on CRITICAL |
|--------|------|----------|--------------------|
| Load avg (1 min) / cores | > 1.5 (load 12) | > 2.5 (load 20) | Alert dashboard + OLED; optionally pause VLM |
| Per-core CPU sustained 30 s | > 85 % | > 95 % | Same |
| `astra_camera_node` CPU | > 200 % | > 300 % | Restart camera / drop resolution |
| `suspect_localizer` RES | > 700 M | > 900 M | Restart node (leak suspicion) |
| Free RAM (excl. cache) | < 800 M | < 400 M | Refuse new VLM query; alert |
| `/camera/color/image_raw` rate | < 20 Hz | < 10 Hz | Flag camera stall |
| TF `map→base_link` age | > 0.5 s | > 2 s | Flag localization stall |
| Run queue (`procs_running`) | > 10 | > 16 | Alert |

> The baseline snapshot **trips CRITICAL** on load average and per-core CPU — this is the
> motivation for the CPU-reduction backlog above.
