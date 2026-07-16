#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash;  
ros2 run suspect_matcher suspect_localizer --ros-args \
  -p cloud_buffer_size:=8 \
  -p max_pair_dt_sec:=0.3
 ; exec bash"
