#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; 
ros2 launch oled_status oled_status.launch.py ip_interface:=wlan0
; exec bash"
