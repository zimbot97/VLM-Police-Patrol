#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash;  
ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
  -p feed_type:=1 -p model_type:=0 \
  -p model_file_name:=/home/sunrise/models/internvl2_5_1b/vit_model_int16_v2.bin \
  -p llm_model_name:=/home/sunrise/models/internvl2_5_1b/Qwen2.5-0.5B-Instruct-Q4_0.gguf \
  -p system_prompt:="config/system_prompt.txt" \
  --log-level warn
 ; exec bash"
