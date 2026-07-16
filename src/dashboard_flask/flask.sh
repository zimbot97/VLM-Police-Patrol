#!/bin/bash
# Launch the dashboard on the RDK X5 with compressed-image streaming enabled
# (relays the camera's JPEG bytes verbatim — near-zero CPU vs re-encoding raw).
#
# Adjust compressed_image_topic to match your camera. Check with:
#   ros2 topic list | grep compressed
#
# If your camera has NO compressed topic, set use_compressed:=false and it
# falls back to encoding the raw image_topic itself.

xterm -hold -e "source /opt/ros/humble/setup.bash; \
source ~/ros2_ws/install/setup.bash; \
ros2 run dashboard_flask flask_node --ros-args \
  -p use_compressed:=true \
  -p compressed_image_topic:=/camera/color/image_raw/compressed \
  -p image_topic:=/camera/color/image_raw \
  -p cmd_vel_topic:=/cmd_vel \
  -p holonomic_mode_topic:=/holonomic_mode \
  -p stream_fps:=15 \
  -p stream_max_width:=0 \
  -p jpeg_quality:=70"
