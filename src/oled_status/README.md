# oled_status

Drives a 0.96" SSD1306 OLED as a status HUD for the VLM-Police-Patrol robot, showing the robot's IP address, live system metrics, and an animated state icon.

## Overview

`oled_status` is a single-node ROS 2 package that runs on the RDK X5 and renders a 128x64 monochrome status display over I2C. It subscribes to existing workspace topics (suspect match results, VLM prompt activity, capture state, and human detections) and derives a highest-priority state to animate, while continuously showing the current IP address and CPU/BPU telemetry. If `luma.oled`/Pillow or the panel are missing, it degrades gracefully to headless logging. It sits alongside the perception and matching nodes as the robot's on-device status indicator (see the workspace root [../../README.md](../../README.md)).

## Nodes / executables

| Executable | Source file | Role |
| --- | --- | --- |
| `oled_status` | [oled_status/oled_status_node.py](oled_status/oled_status_node.py) | ROS node `oled_status`; reads status topics + system sensors and renders the OLED HUD |

## ROS interfaces

### Parameters

| Name | Type | Default |
| --- | --- | --- |
| `i2c_bus` | int | `5` |
| `i2c_address` | int | `0x3C` (60) |
| `ip_interface` | str | `wlan0` |
| `fps` | float | `15.0` |
| `enable_display` | bool | `true` |
| `vlm_timeout_sec` | float | `900.0` |
| `show_stats` | bool | `true` (node-only; not exposed by the launch file) |

### Subscribed topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/suspect_feature_match` | `std_msgs/Bool` | True -> MATCH badge, False -> NOMATCH cross (held 6 s) |
| `/prompt_text` | `std_msgs/String` | Marks VLM analysis in flight -> ANALYZE spinner |
| `/oled/state` | `std_msgs/String` | `"capturing"` -> CAPTURE spinner; `idle`/`clear`/`normal` clears state |
| `/human_present` | `std_msgs/Bool` | True -> HUMAN icon (held 1.2 s) |
| `/yolo/detections` | `ai_msgs/PerceptionTargets` | Only subscribed if `ai_msgs` is importable; counts `person` targets -> HUMAN |

## Launch

Launch file: [launch/oled_status.launch.py](launch/oled_status.launch.py). Arguments and defaults: `i2c_bus` (`5`), `i2c_address` (`60` = 0x3C), `ip_interface` (`wlan0`), `fps` (`15.0`), `enable_display` (`true`), `vlm_timeout_sec` (`900.0`).

```bash
ros2 launch oled_status oled_status.launch.py ip_interface:=wlan0
```

The helper script [../../sh/oled.sh](../../sh/oled.sh) opens an xterm, sources ROS 2 Humble and the workspace, and runs exactly this launch command with `ip_interface:=wlan0`.

## Hardware / display

- SSD1306 0.96" OLED, 128x64, monochrome, over I2C (`i2c_bus=5` -> Linux bus 5 on the RDK X5, address `0x3C`).
- Header: IP address (top row) plus a horizontal divider; when stats are off, a blinking live dot is drawn top-right.
- Left zone: centered state label and an animated icon — CCTV lens (PATROL/default), person (HUMAN), spinner (ANALYZE/CAPTURE), police badge with flashing beacon (MATCH), blinking cross (NOMATCH).
- Right stats column (when `show_stats`): CPU usage % and CPU temperature (`temp3_input`), plus BPU temperature (`temp2_input`), read from `/sys/class/hwmon`. CPU % is derived from `/proc/stat` deltas. BPU temp reads valid only while the BPU is running.

## Dependencies

- ROS: `rclpy`, `std_msgs`, and optionally `ai_msgs` (import is guarded; falls back to `/human_present` if absent).
- Python (pip, not rosdep): `luma.oled` (pulls in `luma.core`) and `Pillow` (PIL) for panel access and drawing; `fcntl`/`socket`/`struct` (stdlib) for interface IP lookup.
- Build/test tooling: `ament_python`, `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

Install the panel driver with:

```bash
sudo pip3 install luma.oled
```

## Build & run

```bash
cd ~/ros2_ws
colcon build --packages-select oled_status
source install/setup.bash
ros2 launch oled_status oled_status.launch.py ip_interface:=wlan0
```

## Files

```
oled_status/
├── package.xml                  # manifest, deps (rclpy, std_msgs, ai_msgs)
├── setup.py                     # entry point: oled_status -> oled_status_node:main
├── setup.cfg                    # install script dirs
├── launch/
│   └── oled_status.launch.py    # launches the node with I2C + network args
├── oled_status/
│   ├── __init__.py
│   └── oled_status_node.py      # OledStatusNode: state machine + OLED rendering
└── resource/
    └── oled_status              # ament resource marker
```
