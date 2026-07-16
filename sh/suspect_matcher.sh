#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash;  
ros2 launch suspect_matcher compare.launch.py \
  reference_image_path:=/tmp/reference_crop.jpg \
  candidate_image_path:=/tmp/candidate_crop.jpg; exec bash"
