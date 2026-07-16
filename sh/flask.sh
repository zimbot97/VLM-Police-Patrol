#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; ros2 run dashboard_flask flask_node --ros-args -p image_topic:=/yolo/image_annotated -p map_topic:=/map -p pose_topic:=/amcl_pose -p cmd_vel_topic:=/cmd_vel -p holonomic_mode_topic:=/holonomic_mode; exec bash"
