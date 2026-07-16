# police_patrol_bot

Bring-up and localization package for the VLM-Police-Patrol mecanum robot: bridges the RP2040 base to ROS 2, fuses odometry, localizes against a saved map, and launches the camera.

## Overview

This package is the runtime backbone for the patrol robot on the RDK X5. The RP2040 firmware (`pico_firware/mecanum_base`) runs all mecanum kinematics, wheel odometry and IMU sampling onboard; this package bridges it to ROS 2 via `micro_ros_agent`. It publishes the robot model and TF through `robot_state_publisher` (URDF from [`police_patrol_bot_description`](../police_patrol_bot_description)), fuses `/odom` + `/imu/data_raw` with a `robot_localization` EKF into the `odom -> base_footprint` transform, runs `nav2_amcl` localization against a saved map, and launches the Astra depth camera with a point-cloud-to-laserscan converter.

## Launch files

| Launch file | Nodes started | Key args (defaults) |
|---|---|---|
| [`launch/bringup.launch.py`](launch/bringup.launch.py) | `micro_ros_agent`, `robot_state_publisher`, `ekf_filter_node`, `wheel_joint_state_publisher` | `serial_port` (`/dev/ttyACM0`), `serial_baud` (`115200`), `use_sim_time` (`false`) |
| [`launch/amcl.launch.py`](launch/amcl.launch.py) | `map_server`, `amcl`, `lifecycle_manager_localization` | `map` (`maps/map.yaml`), `params_file` (`config/amcl.yaml`), `use_sim_time` (`false`) |
| [`launch/camera.launch.py`](launch/camera.launch.py) | `astra_pro` (included), `pointcloud_to_laserscan` | `min_height` (`0.90`), `max_height` (`1.10`), `cloud_in` (`/camera/depth/points`), `scan_frame` (`base_footprint`) |

### bringup.launch.py

Brings the mecanum base online. It:
1. Runs `micro_ros_agent` in `serial` mode over the Pico USB port (Pico publishes `/odom`, `/wheel_speeds`, `/imu/data_raw`; subscribes `/cmd_vel`).
2. Starts `robot_state_publisher` with the URDF processed from `police_patrol_bot_description/urdf/mecanum_robot.urdf.xacro` (via the `xacro` Python module), publishing the model and the fixed `base_footprint -> base_link` TF.
3. Starts the `robot_localization` `ekf_node` (named `ekf_filter_node`) using [`config/ekf.yaml`](config/ekf.yaml).
4. Runs `wheel_joint_state_publisher` to animate wheels in RViz.

### amcl.launch.py

Localization stack. `map_server` publishes the latched `/map`, `nav2_amcl` provides the `map -> odom` transform, and `nav2_lifecycle_manager` (`lifecycle_manager_localization`) auto-configures and activates both with `autostart: true`.

### camera.launch.py

Includes the `astra_camera` `astra_pro.launch.xml` (for the LeTMC-520) with `uvc_product_id: 1282`, `enable_point_cloud`, `enable_colored_point_cloud`, and `depth_registration` enabled. Then runs `pointcloud_to_laserscan` converting `cloud_in` to `/scan` in the `base_footprint` frame over a ±90° arc (`angle_increment` ~0.5°), `range_min` 0.30 m, `range_max` 8.0 m, `scan_time` 0.0333 s (30 Hz), `concurrency_level: 1` (the RDK X5 is CPU-constrained under BPU/VLM+YOLO load).

## Configuration

- [`config/ekf.yaml`](config/ekf.yaml) — `robot_localization` EKF at `frequency: 30.0` Hz, `two_d_mode: true` (zeroes z, roll, pitch), `sensor_timeout: 0.2`, publishing TF. Frames: `odom_frame: odom`, `base_link_frame: base_footprint`, `world_frame: odom` (`map_frame` declared but unused). Fuses body velocities from `odom0: /odom` (vx, vy, vyaw — not absolute pose, to avoid double-counting the Pico's integration) and from `imu0: /imu/data_raw` the yaw rate and x/y accelerations (MPU9250 raw, orientation not provided) with `imu0_remove_gravitational_acceleration: true`.
- [`config/amcl.yaml`](config/amcl.yaml) — `nav2_amcl` with `robot_model_type: nav2_amcl::OmniMotionModel`, `min_particles: 500` / `max_particles: 2000`, `laser_model_type: likelihood_field`, `laser_max_range: 12.0`, `max_beams: 60`, motion noise `alpha1..5: 0.05`, `update_min_d: 0.25`, `update_min_a: 0.2`, `transform_tolerance: 1.0`, `scan_topic: scan`, frames `base_link` / `odom` / `map`. `set_initial_pose: true` at pose (0, 0, yaw 0) so no manual "2D Pose Estimate" is needed at boot.
- [`maps/`](maps/) — saved map (`map.yaml` + `map.pgm`), `resolution: 0.05` m/px, `origin: [-5, -5, 0]`, `mode: trinary`, `occupied_thresh: 0.65`, `free_thresh: 0.25`.

## Scripts

[`scripts/wheel_joint_state_publisher.py`](scripts/wheel_joint_state_publisher.py) — subscribes to `/wheel_speeds` (`std_msgs/Float32MultiArray`, `[FL, RL, RR, FR]` in rad/s @ 20 Hz from firmware), integrates each wheel's angular velocity over wall-clock dt into a joint position, and republishes `/joint_states` (`sensor_msgs/JointState`) with URDF joint names `FL_/RL_/RR_/FR_wheel_joint` so `robot_state_publisher` animates the wheels in RViz.

## Dependencies

Build (`buildtool`): `ament_cmake` (plus `ament_cmake_python` in CMakeLists).

Exec: `police_patrol_bot_description`, `micro_ros_agent`, `robot_state_publisher`, `robot_localization`, `xacro`, `rclpy`, `sensor_msgs`, `std_msgs`, `astra_camera`, `pointcloud_to_laserscan`, `launch`, `launch_ros`, `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`.

Test: `ament_lint_auto`, `ament_lint_common`.

## Build & run

```bash
# from the workspace root (e.g. ~/ros2_ws)
colcon build --packages-select police_patrol_bot
source install/setup.bash

# base bring-up (micro-ROS bridge + state + EKF)
ros2 launch police_patrol_bot bringup.launch.py
# optional: override the Pico serial port
ros2 launch police_patrol_bot bringup.launch.py serial_port:=/dev/ttyACM0

# camera + laserscan
ros2 launch police_patrol_bot camera.launch.py

# map server + AMCL localization
ros2 launch police_patrol_bot amcl.launch.py
```

Convenience wrappers (open each in an `xterm`, sourcing ROS + the workspace):
[`sh/bringup.sh`](../../sh/bringup.sh), [`sh/camera.sh`](../../sh/camera.sh), [`sh/amcl.sh`](../../sh/amcl.sh). The full stack (these plus YOLO, VLM, suspect matcher, dashboard, OLED) is launched by `sh/master.sh`.

## Files

```
police_patrol_bot/
├── CMakeLists.txt                       # ament_cmake; installs launch/config/maps + the script
├── package.xml                          # deps (see above)
├── launch/
│   ├── bringup.launch.py                # micro-ROS agent + RSP + EKF + wheel joint states
│   ├── amcl.launch.py                   # map_server + amcl + lifecycle manager
│   └── camera.launch.py                 # Astra camera + pointcloud_to_laserscan
├── config/
│   ├── ekf.yaml                         # robot_localization EKF (odom + IMU fusion)
│   └── amcl.yaml                        # nav2_amcl localization params
├── maps/
│   ├── map.yaml                         # map metadata (0.05 m/px, trinary)
│   └── map.pgm                          # occupancy grid image
└── scripts/
    └── wheel_joint_state_publisher.py   # /wheel_speeds -> /joint_states
```

## See also

- Project root: [`../../README.md`](../../README.md)
- Calibration and system details: [`../../docs/technical.md`](../../docs/technical.md)
