#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash;  ros2 run suspect_matcher yolo_detect --ros-args \
  -p model_path:=/home/sunrise/rdk_model_zoo/samples/vision/ultralytics_yolo/model/yolo11n_detect_bayese_640x640_nv12.bin \
  -p camera_topic:=/camera/color/image_raw \
  -p live_view:=true \
  -p keep_conf:=0.7
  ; exec bash"
