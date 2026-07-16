#!/bin/bash
bash ~/ros2_ws/sh/bringup.sh &
sleep 2
bash ~/ros2_ws/sh/camera.sh &
sleep 2
bash ~/ros2_ws/sh/yolo.sh &
sleep 2
bash ~/ros2_ws/sh/llamacpp.sh &
sleep 2
bash ~/ros2_ws/sh/suspect_matcher.sh &
sleep 2
bash ~/ros2_ws/sh/amcl.sh &
sleep 2
bash ~/ros2_ws/sh/suspect_localize.sh &
sleep 2
bash ~/ros2_ws/sh/flask.sh &
sleep 2
bash ~/ros2_ws/sh/oled.sh &
sleep 2
