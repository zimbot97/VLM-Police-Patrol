"""
Launch the suspect_matcher compare node.

This launches ONLY the attribute-compare node, not hobot_llamacpp.
hobot_llamacpp must be started separately (it needs its config/ directory
with the model files staged into its working directory, per its README),
e.g.:

  cp -r install/lib/hobot_llamacpp/config/ .
  ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
    -p feed_type:=1 -p model_type:=0 \
    -p model_file_name:=vit_model_int16_v2.bin \
    -p llm_model_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf \
    -p system_prompt:="config/system_prompt.txt" \
    --log-level warn

Then launch this node with your two crop paths:

  ros2 launch suspect_matcher compare.launch.py \
    reference_image_path:=/tmp/reference_crop.jpg \
    candidate_image_path:=/tmp/candidate_crop.jpg
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    reference_image_path = LaunchConfiguration('reference_image_path')
    candidate_image_path = LaunchConfiguration('candidate_image_path')
    response_timeout_sec = LaunchConfiguration('response_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'reference_image_path', default_value='/tmp/reference_crop.jpg',
            description='Path to the reference (suspect) person-crop image.'),
        DeclareLaunchArgument(
            'candidate_image_path', default_value='/tmp/candidate_crop.jpg',
            description='Path to the candidate person-crop image.'),
        DeclareLaunchArgument(
            'response_timeout_sec', default_value='900.0',
            description='Per-query timeout; high default covers cold model load.'),

        Node(
            package='suspect_matcher',
            executable='attribute_compare',
            name='attribute_compare_from_files_node',
            output='screen',
            parameters=[{
                'reference_image_path': reference_image_path,
                'candidate_image_path': candidate_image_path,
                'response_timeout_sec': response_timeout_sec,
            }],
        ),
    ])
