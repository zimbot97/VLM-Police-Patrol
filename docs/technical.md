# Technical Documentation

Engineering reference for the VLM Police Patrol robot: system architecture,
calibration procedures, and known issues. For quick-start / launch instructions see the
top-level [README.md](../README.md); for inference numbers see
[benchmarks.md](benchmarks.md).

---

## 1. Architecture

### 1.1 Compute & network topology

```
   ┌──────────────────────────────┐        Wi-Fi hotspot "police_patrol"
   │  RDK X5  (on-board compute)   │◄──────────────(pass 123456789)──────────────┐
   │  • ros2_astra_camera          │                                             │
   │  • yolo_detect (BPU)          │        ┌───────────────────────────┐        │
   │  • hobot_llamacpp VLM         │        │  SLAM laptop               │        │
   │    (ViT=BPU, LLM=CPU)         │◄──ROS2─┤  slam_gmapping_Humble      │◄───────┘
   │  • suspect_matcher / localize │  (same │  (off-board mapping)       │
   │  • dashboard_flask + OLED     │  DOMAIN)└───────────────────────────┘
   │  • micro_ros_agent            │
   └──────────────┬───────────────┘        ┌───────────────────────────┐
                  │ USB CDC /dev/ttyACM0    │  Operator browser          │
                  ▼                         │  (Flask dashboard)         │◄──hotspot
   ┌──────────────────────────────┐        └───────────────────────────┘
   │  RP2040 Pico (mecanum_base)   │
   │  • mecanum kinematics         │
   │  • 4× motor + encoder control │
   │  • MPU9250 IMU (I2C1)         │
   └──────────────────────────────┘
```

- **RDK X5** runs all perception, the VLM, localization, the dashboard, and the
  micro-ROS agent. It creates the `police_patrol` Wi-Fi hotspot.
- **SLAM laptop** runs `slam_gmapping` off-board on the same ROS 2 network to keep the
  X5's CPU/BPU free — all machines share one `ROS_DOMAIN_ID`.
- **RP2040 Pico** owns real-time motion: mecanum kinematics, motor/encoder loops, IMU
  sampling. It talks to the X5 over USB CDC (`/dev/ttyACM0`) via micro-ROS.

### 1.2 Software layers

| Layer | Runs on | Components |
|-------|---------|------------|
| Perception (BPU) | RDK X5 | `yolo_detect` (YOLO11n), `hobot_llamacpp` ViT encoder |
| Reasoning (CPU) | RDK X5 | `hobot_llamacpp` Qwen LLM head, `attribute_compare` |
| Localization | X5 + laptop | `ekf_filter_node`, `amcl`, `suspect_localizer`, `slam_gmapping` (laptop) |
| HMI | RDK X5 | `dashboard_flask`, `oled_status` |
| Bridge | RDK X5 | `micro_ros_agent` |
| Real-time control | RP2040 | `mecanum_base` firmware |

### 1.3 Frames

- TF tree: `map → odom → base_footprint → base_link → {wheels, camera, imu}`.
- `ekf_filter_node` publishes `odom → base_footprint`; `amcl` (or SLAM) provides
  `map → odom`; the URDF fixes `base_footprint → base_link`.

### 1.4 Package inventory

| Package | Type | Location | Status |
|---|---|---|---|
| `suspect_matcher` | ament_python (ours) | [src/suspect_matcher/](../src/suspect_matcher/) | Active |
| `dashboard_flask` | ament_python (ours) | [src/dashboard_flask/](../src/dashboard_flask/) | Active |
| `hobot_llamacpp` | 3rd-party (external) | [src/hobot_llamacpp/](../src/hobot_llamacpp/) | Placeholder dir — installed separately |
| `hobot_dnn` | 3rd-party | [src/hobot_dnn/](../src/hobot_dnn/) | Empty placeholder |
| `hobot_msgs` / `ai_msgs` | 3rd-party msgs | [src/hobot_msgs/](../src/hobot_msgs/) | Empty placeholder |
| `ros2_astra_camera` | 3rd-party driver | [src/ros2_astra_camera/](../src/ros2_astra_camera/) | Empty placeholder (camera source) |
| `micro_ros_setup` / `uros` | micro-ROS agent | [src/uros/](../src/uros/) | Empty placeholder |
| `mecanum_base` (firmware) | Pico C / micro-ROS | [pico_firware/mecanum_base/](../pico_firware/mecanum_base/) | Active |

Only `suspect_matcher`, `dashboard_flask`, and the Pico firmware contain our source; the
empty `src/*` dirs are placeholders for vendor packages installed on the device.

#### `suspect_matcher` nodes

| Executable | Role |
|---|---|
| `attribute_compare` | Sends reference + candidate crops to the VLM, compares attributes, publishes match |
| `yolo_detect` | On-board YOLO person detector + crop capture |
| `yoloworld_detect` | YOLO-World (open-vocab, prompt-driven) detector + crop capture |

- **`attribute_compare`** — pubs `/image`, `/prompt_text` → VLM; subs `/llama_cpp_node`;
  pubs `/suspect_feature_match` (Bool), `/suspect_feature_match_detail` (String); serves
  `/compare_images` (Trigger).
- **`yolo_detect` / `yoloworld_detect`** — subs `/camera/color/image_raw`; pubs
  `/yolo/detections` · `/yoloworld/detections` + `*_image_annotated`; serves
  `/capture_crop` (Trigger) → writes crop to `/tmp`.

#### `dashboard_flask` (`flask_node`)

- Subs `/camera/image_raw`, `/map`, `/amcl_pose`, `/suspect_feature_match(_detail)`;
  pubs `/cmd_vel`, `/holonomic_mode`; service clients `/capture_crop`, `/compare_images`
  + `set_parameters` on `/yolo_detect_node`.

#### Pico `mecanum_base`

- micro-ROS over USB CDC → `micro_ros_agent` (`/dev/ttyACM0`). Subs `cmd_vel`; pubs
  `odom`, `wheel_speeds`, `imu/data_raw`.

### 1.5 Cross-package relationship map

```
                    ┌──────────────────────────┐
                    │   dashboard_flask         │  (web UI + joystick)
                    │   flask_node              │
                    └──────────────────────────┘
        cmd_vel /        │  ▲ camera/image_raw       │ srv: /capture_crop
        holonomic_mode   │  │ match + match_detail    │      /compare_images
                         ▼  │                          ▼
  ┌────────────┐   ┌──────────────────┐   ┌─────────────────────────────┐
  │ Pico fw    │   │ ros2_astra_camera│   │ suspect_matcher             │
  │ mecanum_   │◄──┤  (camera driver) ├──►│  yolo_detect / yoloworld    │
  │ base       │   │ /camera/color/…  │   │   → /capture_crop → /tmp    │
  │ (micro-ROS)│   └──────────────────┘   │  attribute_compare          │
  │            │                          │   /compare_images           │
  │ odom, imu, │                          └──────────────┬──────────────┘
  │ wheel_spds │                          /image,/prompt_text │ ▲ /llama_cpp_node
  └────────────┘                                              ▼ │
   ▲ cmd_vel                                        ┌──────────────────┐
   └─(via micro_ros_agent /dev/ttyACM0)             │ hobot_llamacpp   │
                                                    │  (VLM node)      │
                                                    └──────────────────┘
```

**End-to-end flow**
1. `ros2_astra_camera` publishes `/camera/color/image_raw` (detectors) and
   `/camera/image_raw` (dashboard preview).
2. Dashboard uploads a reference photo, then calls `/capture_crop` → a `yolo(world)_detect`
   node saves the current person crop to `/tmp`.
3. Dashboard calls `/compare_images` → `attribute_compare` pushes both crops + prompts to
   `hobot_llamacpp` (`/image`, `/prompt_text`), reads `/llama_cpp_node`.
4. `attribute_compare` publishes `/suspect_feature_match` (+ `_detail`) → dashboard shows
   the result.
5. Driving: dashboard publishes `/cmd_vel` → `micro_ros_agent` → Pico `mecanum_base`
   moves the wheels and returns `odom` / `imu/data_raw` / `wheel_speeds`.

**Topic / service connection table**

| Producer | Interface | Type | Consumer |
|---|---|---|---|
| ros2_astra_camera | `/camera/color/image_raw` | Image | yolo_detect, yoloworld_detect |
| ros2_astra_camera | `/camera/image_raw` | Image | flask_node |
| flask_node | `/capture_crop` (call) | std_srvs/Trigger | yolo(world)_detect (server) |
| flask_node | `/compare_images` (call) | std_srvs/Trigger | attribute_compare (server) |
| attribute_compare | `/image`, `/prompt_text` | Image / String | hobot_llamacpp |
| hobot_llamacpp | `/llama_cpp_node` | (VLM result) | attribute_compare |
| attribute_compare | `/suspect_feature_match(_detail)` | Bool / String | flask_node |
| flask_node | `/cmd_vel`, `/holonomic_mode` | Twist / Bool | mecanum_base (via agent) |
| mecanum_base | `odom`, `wheel_speeds`, `imu/data_raw` | Odometry / … | (nav / consumers) |

---

## 2. Calibration

### 2.1 Chassis geometry (firmware constants)

Set in [`mecanum_base.cpp`](../pico_firware/mecanum_base/mecanum_base.cpp) — **measure
against the real chassis before tuning odometry**:

| Constant | Value | Meaning |
|----------|-------|---------|
| `WHEEL_R` | 0.04 m | Wheel radius |
| `LX` | 0.0625 m | Half wheelbase (front–rear); full 125 mm |
| `LY` | 0.100 m | Half track width (left–right); full 200 mm |
| `ENC_CPR` | 1560 | Counts/rev = 13 pulses × 4 (quadrature) × 30:1 gear |
| `MAX_W` | 20.0 rad/s | Max wheel angular speed (100 % duty) |

**Odometry check:** command a known straight 1 m and a 360° in-place rotation; compare
`/odom` (and `/odometry/filtered`) to reality. If linear distance is off, re-check
`WHEEL_R` and `ENC_CPR`; if rotation is off, re-check `LX`/`LY`.

### 2.2 IMU (MPU9250)

- I2C1 on GP26/GP27 @ 400 kHz, address `0x68`.
- Keep the robot **stationary at boot** — gyro bias is sampled at startup; movement
  during init corrupts the bias estimate.
- The EKF ([`ekf.yaml`](../src/police_patrol_bot/config/ekf.yaml)) uses IMU yaw-rate +
  linear accel with `imu0_remove_gravitational_acceleration: true`; orientation is not
  fused (raw IMU has no absolute orientation).

### 2.3 Sensor fusion (EKF)

- 30 Hz, `two_d_mode: true`, `sensor_timeout: 0.2 s`.
- Odometry fuses **body velocities only** (vx, vy, vyaw) — not absolute pose — to avoid
  double-counting the Pico's own integration.
- If filtered pose drifts, verify odom and IMU timestamps are sane and that both streams
  arrive within the timeout.

### 2.4 Camera / detection

- YOLO input: 640×640 NV12; person confidence gate `keep_conf:=0.7` (raise to reduce
  false positives, lower to catch more candidates).
- Detection + depth pairing for localization requires the two within
  `max_pair_dt_sec:=0.3 s` (buffer `cloud_buffer_size:=8`). Widen `max_pair_dt_sec` if
  valid detections fail to localize; tighten it if stale depth causes bad coordinates.

### 2.5 SLAM (slam_gmapping)

- Full parameter set is in the README's SLAM block. Key tuning knobs: `maxUrange` (4.0)
  / `maxRange` (5.0) to the lidar's usable range; `particles` (30) up for robustness at
  higher CPU cost; `linearUpdate`/`angularUpdate` for scan-integration cadence.

---

## 3. Known issues

| # | Issue | Impact | Workaround / status |
|---|-------|--------|---------------------|
| 1 | **High CPU under VLM load** | X5 ~90 % CPU, ~70 °C with full stack + VLM (per OLED) | Teleop still smooth (motion is on the Pico). Ensure adequate airflow/heatsink; consider a fan for sustained runs. |
| 2 | **Hardware E-stop not yet fitted** | No physical power-cut for motors | Software 0.5 s `cmd_vel` watchdog brakes on signal loss; hardware E-stop is a `[PLACEHOLDER]` (see README). |
| 3 | **SLAM depends on off-board laptop** | Mapping stops if the laptop drops off the hotspot | Keep laptop on `police_patrol`; matching `ROS_DOMAIN_ID` required for discovery. |
| 4 | **VLM must be started separately** | `compare.launch.py` does not start `hobot_llamacpp` (needs model config staged into cwd) | `master.sh` launches it via [`llamacpp.sh`](../sh/llamacpp.sh); run manually if launching piecemeal. |
| 5 | **Vendor `src/*` dirs are placeholders** | `hobot_dnn`, `hobot_msgs`, `ros2_astra_camera`, `uros` are empty in the repo | Installed separately on the device; `src/ros2.repos` is empty. |
| 6 | **IMU boot bias** | Gyro bias corrupted if robot moves during Pico init | Keep the robot still at power-on / firmware reset. |
| 7 | **Fixed crop paths in `/tmp`** | Reference/candidate crops overwrite `/tmp/*_crop.jpg` each cycle | Expected for single-suspect workflow; not concurrency-safe for multiple operators. |
| 8 | **`xterm`-based launch** | Each node opens a held `xterm`; closing a window kills that node | Intended for demo visibility; use `pkill -f xterm` to stop all (see README). |
| 9 | **Pico enumeration flaky** | `micro_ros_agent` sometimes won't connect to the Pico on launch (no `/dev/ttyACM0` handshake) | Physically **unplug and replug** the Pico's USB occasionally so it re-enumerates, then relaunch [`bringup.sh`](../sh/bringup.sh). |
