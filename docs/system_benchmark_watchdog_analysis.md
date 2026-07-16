# VLM Police Patrol — System Benchmark & Watchdog Analysis

**Platform:** RDK X5 (8× Cortex-A55 @ 1.5 GHz, 8 GB LPDDR4, 10 TOPS BPU)
**Snapshot source:** htop, 2026-07-14 01:37 (uptime 03:45:18)
**Stack state:** Full patrol stack running (camera, YOLO, VLM suspect matcher, AMCL/Nav2, EKF, Flask dashboard, OLED, micro-ROS agent)

---

## 1. Headline Metrics

| Metric | Value | Assessment |
|---|---|---|
| Load average (1 min) | **20.75** | 🔴 Critical — 2.6× core count |
| Load average (5 min) | 22.05 | 🔴 Sustained, not a spike |
| Load average (15 min) | 19.22 | 🔴 Chronic saturation |
| CPU utilization (all 8 cores) | 86.6 – 93.6% | 🔴 No headroom |
| Memory | 2.65 G / 5.20 G (~51%) | 🟡 OK, but ION carve-out reduces visible total |
| Swap | 0 K / 0 K | 🟡 No swap configured — OOM kill is the only pressure valve |
| Tasks / threads / running | 155 / 469 / 8 running | 🟡 Run queue exactly matches core count |

**Interpretation:** the CPU is the bottleneck, not RAM. A load average of ~21 on 8 cores means roughly 13 threads are runnable-but-waiting at any instant. Everything on the box (TF timing, AMCL updates, MJPEG frame pacing, micro-ROS agent latency) degrades under this condition. Note the visible 5.2 G total (not 8 G) reflects the ION memory carve-out for the BPU (~2.5–3 G reserved), so effective RAM headroom is tighter than the percentage suggests.

---

## 2. Per-Process Breakdown

> ⚠️ **Thread caveat:** htop was displaying threads (H not pressed). Rows sharing a command line with identical RES/MEM% are threads of one process — memory is shared, **not additive**. CPU% per row *is* additive across threads of the same process.

### CPU ranking (aggregated by process)

| Rank | Process | Approx. total CPU | Notes |
|---|---|---|---|
| 1 | `astra_camera_node` | **~290%+ (≈3 cores)** | Main thread 176% + sibling threads at 41%, 32%, 25%, 25%, 21%, 8.5%, 8.5%, 7.9%, 7.3%. Single largest consumer on the system. |
| 2 | `suspect_localizer` (python3) | **~150%+ (≈1.5 cores)** | 95.1% peak thread + 28%, 7.3%, 6.1×3, 5.5%, 4.9×2. Point-cloud/depth lookup work. |
| 3 | `yolo_detect` (suspect_matcher) | **~95% (≈1 core)** | 72.6% + 9.1% + 7.9% + 4.9×3. |
| 4 | `ekf_node` (robot_localization) | **~110%** | 54.9% + 42.1% + 30.5% + 12.2%. High for an EKF — see §4. |
| 5 | `pointcloud_to_laserscan` | **~55%** | 29.3% + 16.5% + 4.9% + 2.2-class threads. |
| 6 | `robot_state_publisher` | **~75%** | 42.1% + 32.3% — unusually high, likely high-rate joint states. |
| 7 | `nav2_amcl` | **~65%** | 36.6% + 25.0% + 5.5%. Normal under active localization. |
| 8 | `dashboard_flask` | **~22%** | 17.1% + 5.5%. MJPEG re-encode/passthrough + Socket.IO. |
| 9 | `oled_status` | ~16.5% | 15 fps PIL redraws — expected. |
| 10 | `Xorg` | ~22% | Desktop session running on the robot — pure overhead in deployment. |
| 11 | `wheel_joint_state_publisher.py` | ~14% | |
| 12 | `micro_ros_agent` | ~7.3% | Healthy. |
| 13 | `lifecycle_manager` | ~5.5% | |

### Memory ranking (per process, threads collapsed)

| Rank | Process | RES | MEM% | Notes |
|---|---|---|---|---|
| 1 | `suspect_localizer` | **635 M** | 11.9% | Largest single footprint — cloud buffer + Python + model-adjacent buffers |
| 2 | `astra_camera_node` | 247 M | 4.6% | RGBD driver buffers |
| 3 | `yolo_detect` | 190 M | 3.6% | |
| 4 | `dashboard_flask` | 136 M | 2.6% | |
| 5 | `Xorg` | 128 M | 2.4% | Removable in headless deployment |
| 6 | `nav2_amcl` | ~35 M | 0.7% | |
| 7 | `oled_status` | ~66 M | 1.2% | |

(Not visible in this snapshot: `hobot_llamacpp` / InternVL2.5-1B resident weights sit largely in the ION region, outside normal RES accounting.)

---

## 3. Watchdog Thresholds (proposed)

Suggested trip levels for a monitoring node (e.g. a `system_watchdog` publishing `diagnostic_msgs/DiagnosticArray`):

| Signal | WARN | CRITICAL | Action on CRITICAL |
|---|---|---|---|
| Load avg (1 min) / cores | > 1.5 (load 12) | > 2.5 (load 20) | Publish alert to dashboard + OLED; optionally pause VLM queries |
| Per-core CPU sustained 30 s | > 85% | > 95% | Same |
| `astra_camera_node` CPU | > 200% | > 300% | Restart camera node / drop resolution profile |
| `suspect_localizer` RES | > 700 M | > 900 M | Restart node (leak suspicion) |
| Free RAM (excl. cache) | < 800 M | < 400 M | Refuse new VLM query; alert |
| Swap usage | any (none configured) | — | N/A until zram added |
| `/camera/color/image_raw` rate | < 20 Hz | < 10 Hz | Flag camera stall |
| TF `map→base_link` age | > 0.5 s | > 2 s | Flag localization stall |
| Run queue (`procs_running`) | > 10 | > 16 | Alert |

Current snapshot **trips CRITICAL** on load average and per-core CPU, and WARN-to-CRITICAL on `astra_camera_node` CPU.

---

## 4. Findings & Recommendations (priority order)

1. **`astra_camera_node` (~3 cores) is the top target.**
   - If depth-registered point clouds are enabled at full rate, that's where the cost is. Publish `/camera/depth_registered/points` only if a consumer needs it; the `suspect_localizer` pixel-aligned XYZ path can use the aligned depth image + intrinsics instead of PointCloud2, which is far cheaper.
   - Reduce color/depth resolution or FPS in `astra_pro.launch.xml` (e.g. 640×480@15 instead of @30) and re-measure.

2. **`pointcloud_to_laserscan` + point cloud generation is a double tax.** If the laserscan is derived from the depth cloud, consider `depthimage_to_laserscan` directly from the depth image — it skips the full cloud allocation and typically halves this pipeline's cost.

3. **EKF + robot_state_publisher at ~1.8 cores combined is suspicious.** Check publish rates: wheel odom / IMU input rates and `ekf` `frequency` param. An EKF at 30–50 Hz should be a few percent, not 110%. High-rate `/joint_states` (from `wheel_joint_state_publisher.py`) would also explain the hot `robot_state_publisher`.

4. **Kill the desktop for deployment.** Xorg + GNOME shell overhead (~0.3+ cores, ~130 M) buys nothing on a patrol robot. Boot to multi-user target: `sudo systemctl set-default multi-user.target`.

5. **Add zram swap as an OOM safety net.** With 0 swap and ION carve-out, a cold VLM load spike can OOM-kill a node with no warning. A 1–2 G zram device is nearly free and turns hard kills into graceful slowdowns.

6. **Duplicate camera/localizer instances check.** The table shows many `astra_camera_node` and `suspect_localizer` rows; verify with `H` (hide threads) that only *one* process of each exists. A leftover backgrounded `bringup.sh` previously caused duplicate `map_server`/`lifecycle_manager` — the same failure mode would explain part of this load.

7. **Re-benchmark after each change.** Capture a consistent baseline: `htop` with threads hidden, plus `vmstat 1 30` and `ros2 topic hz` on `/camera/color/image_raw`, `/scan`, `/odom`. Record load avg after 5 min of steady patrol per configuration.

---

## 5. Benchmark Baseline (this snapshot)

```
Config: full stack, active patrol + VLM matcher armed
Load avg:        20.75 / 22.05 / 19.22
CPU per core:    86.6–93.6%
Mem:             2.65 G / 5.20 G, swap 0
Tasks:           155 procs, 469 threads, 8 running
Top CPU:         astra_camera_node ≈ 3.0 cores
Top RAM:         suspect_localizer 635 M RES
```

Use this table as the "before" row; add an "after" row per optimization from §4 to quantify each win.
