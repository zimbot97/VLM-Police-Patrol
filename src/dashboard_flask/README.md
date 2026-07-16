# dashboard_flask

A Flask + Socket.IO web dashboard that runs as a single ROS 2 node, exposing the robot's camera stream, teleop joystick, and suspect-matching workflow in the browser.

## Overview
`dashboard_flask` is the operator-facing web interface for the [VLM-Police-Patrol](../../README.md) robot running on the RDK X5. It runs one ROS 2 node (`dashboard_flask_node`) that spins `rclpy` in a background thread while serving a Flask/Socket.IO app in the main thread. The page shows a live MJPEG camera feed, provides a `cmd_vel` teleop joystick (with holonomic-mode toggle), and drives the `suspect_matcher` pipeline (upload reference, capture candidate crop, run VLM comparison). It also subscribes to map/pose/scan/suspect-pose topics for an optional map overlay.

## Nodes / executables

| Executable | Source file | Role |
|---|---|---|
| `flask_node` | [dashboard_flask/flask_node.py](dashboard_flask/flask_node.py) | Runs the `dashboard_flask_node` ROS 2 node plus the Flask/Socket.IO web server. |

Node name: `dashboard_flask_node`. Entry point: `flask_node = dashboard_flask.flask_node:main`.

## ROS interfaces

### Parameters

| Name | Type | Default |
|---|---|---|
| `image_topic` | string | `/camera/image_raw` |
| `compressed_image_topic` | string | `/camera/image_raw/compressed` |
| `use_compressed` | bool | `False` |
| `map_topic` | string | `/map` |
| `pose_topic` | string | `/amcl_pose` |
| `scan_topic` | string | `/scan` |
| `cmd_vel_topic` | string | `/cmd_vel` |
| `holonomic_mode_topic` | string | `/holonomic_mode` |
| `initialpose_topic` | string | `/initialpose` |
| `jpeg_quality` | int | `70` |
| `stream_max_width` | int | `0` (0 = no downscale) |
| `stream_fps` | int | `15` |
| `port` | int | `5000` |
| `capture_crop_service` | string | `/capture_crop` |
| `compare_service` | string | `/compare_images` |
| `detector_node_name` | string | `/yolo_detect_node` |
| `match_topic` | string | `/suspect_feature_match` |
| `match_detail_topic` | string | `/suspect_feature_match_detail` |
| `suspect_pose_topic` | string | `/suspect_pose` |
| `reference_image_path` | string | `/tmp/reference_crop.jpg` |
| `candidate_image_path` | string | `/tmp/candidate_crop.jpg` |
| `candidate_basename` | string | `candidate` |

### Subscribed topics

| Topic (param) | Type | Notes |
|---|---|---|
| `image_topic` | `sensor_msgs/Image` | Only when `use_compressed=False`; re-encoded to JPEG (BEST_EFFORT). |
| `compressed_image_topic` | `sensor_msgs/CompressedImage` | Only when `use_compressed=True`; relayed verbatim (BEST_EFFORT). |
| `map_topic` | `nav_msgs/OccupancyGrid` | RELIABLE / TRANSIENT_LOCAL. |
| `pose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL pose. |
| `scan_topic` | `sensor_msgs/LaserScan` | BEST_EFFORT, throttled to ~5 Hz. |
| `match_topic` | `std_msgs/Bool` | Suspect match result. |
| `match_detail_topic` | `std_msgs/String` | Match detail text. |
| `suspect_pose_topic` | `geometry_msgs/PoseStamped` | Latched (TRANSIENT_LOCAL) map-frame suspect location. |

### Published topics

| Topic (param) | Type | Notes |
|---|---|---|
| `cmd_vel_topic` | `geometry_msgs/Twist` | Teleop; uses `linear.x`, `linear.y`, `angular.z`. |
| `holonomic_mode_topic` | `std_msgs/Bool` | Drive-mode toggle. |
| `initialpose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL initial pose estimate (frame `map`). |

### Services (clients only)

| Name (param) | Type | Role |
|---|---|---|
| `capture_crop_service` (`/capture_crop`) | `std_srvs/Trigger` | Client — triggers the detector to save the candidate crop. |
| `compare_service` (`/compare_images`) | `std_srvs/Trigger` | Client — runs the VLM comparison (first call is a slow cold load). |
| `<detector_node_name>/set_parameters` | `rcl_interfaces/SetParameters` | Client — sets the detector's `save_basename` param before capturing. |

The node hosts no service servers.

## Web interface

### HTTP routes

| Route | Method | Purpose |
|---|---|---|
| `/` | GET | Dashboard page (`index.html`). |
| `/video_feed` | GET | MJPEG stream of the latest camera frame. |
| `/map.png` | GET | Latest occupancy-grid map as PNG (204 if none). |
| `/map_info` | GET | Map metadata (resolution, origin, width, height). |
| `/pose` | GET | Latest robot pose `{x, y, yaw_deg}`. |
| `/holonomic` | GET | Current drive mode `{enabled}`. |
| `/suspect/upload_reference` | POST | Multipart `image` -> saved as the reference crop. |
| `/suspect/capture_candidate` | POST | Sets detector basename + calls `/capture_crop`. |
| `/suspect/compare` | POST | Calls `/compare_images` and returns cached match result. |
| `/suspect/reference.jpg` | GET | Current reference crop (204 if none). |
| `/suspect/candidate.jpg` | GET | Current candidate crop (204 if none). |
| `/suspect/result` | GET | Latest cached `{match, detail}`. |
| `/suspect/pose` | GET | Last known map-frame suspect location (null if none). |

### Socket.IO events

| Event | Direction | Payload / effect |
|---|---|---|
| `cmd_vel` | client -> server | `{linear, strafe, angular}` -> published to `cmd_vel`. |
| `set_holonomic` | client -> server | `{enabled}` -> published to `holonomic_mode`; echoes `holonomic_state`. |
| `set_initial_pose` | client -> server | `{x, y, yaw}` -> published to `initialpose`. |
| `holonomic_state` | server -> client | `{enabled}` broadcast on mode change. |
| `map_update` | server -> client | Map metadata on new map. |
| `pose_update` | server -> client | Robot pose on new AMCL pose. |
| `scan_update` | server -> client | `{points}` world-frame laser points (~5 Hz). |
| `suspect_update` | server -> client | `{x, y}` suspect map location. |
| `match_result` | server -> client | `{match}` from `match_topic`. |
| `match_detail` | server -> client | `{detail}` from `match_detail_topic`. |

## Dependencies

From `package.xml`: `rclpy`, `ament_index_python`, `std_msgs`, `std_srvs`, `rcl_interfaces`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `cv_bridge`, `python3-opencv`.

Notable Python imports: `flask`, `flask_socketio` (declared in `setup.py` `install_requires`), `cv2`, `numpy`. The Socket.IO client is vendored at `dashboard_flask/static/socket.io.min.js` for offline use.

## Build & run

```bash
cd /home/brian/VLM-Police-Patrol
colcon build --packages-select dashboard_flask --merge-install
source install/setup.bash
```

Run directly:

```bash
ros2 run dashboard_flask flask_node --ros-args -p port:=5000
```

Convenience scripts at the package root:

- [`launch_dashboard.sh`](launch_dashboard.sh) — sources ROS 2 (+ optional TogetherROS on the RDK X5) and the workspace, then runs the node with defaults. Overridable via flags (`--image-topic`, `--map-topic`, `--pose-topic`, `--cmd-vel-topic`, `--holonomic-topic`, `--jpeg-quality`, `--port`) or matching env vars.
- [`flask.sh`](flask.sh) — launches in an `xterm` on the RDK X5 with compressed-image streaming enabled (`use_compressed:=true`, `compressed_image_topic:=/camera/color/image_raw/compressed`).

The dashboard is then served at `http://<host-ip>:<port>/` (default port 5000, bound to `0.0.0.0`).

## Files

```
dashboard_flask/
├── dashboard_flask/
│   ├── __init__.py
│   ├── flask_node.py            # ROS 2 node + Flask/Socket.IO app (all logic)
│   ├── templates/index.html     # Dashboard UI (video, joystick, suspect tools, map overlay)
│   └── static/socket.io.min.js  # Vendored Socket.IO client (offline)
├── resource/dashboard_flask     # ament resource marker
├── package.xml                  # Package manifest / dependencies
├── setup.py                     # ament_python setup + entry point
├── setup.cfg                    # console_scripts install dirs
├── launch_dashboard.sh          # Env-sourcing launch helper
├── flask.sh                     # RDK X5 xterm launch (compressed streaming)
└── README.md                    # This file
```

See the [root README](../../README.md) for how this package fits into the full VLM-Police-Patrol system.
