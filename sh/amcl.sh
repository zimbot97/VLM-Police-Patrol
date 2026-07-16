#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; 
ros2 launch police_patrol_bot amcl.launch.py
; exec bash"
