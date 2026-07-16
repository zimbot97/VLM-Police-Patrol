"""
display.launch.py — visualise the mecanum robot URDF in RViz2.

Usage:
  ros2 launch police_patrol_bot_description display.launch.py
  ros2 launch police_patrol_bot_description display.launch.py gui:=false   # no joint sliders
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('police_patrol_bot_description')

    urdf_path  = os.path.join(pkg, 'urdf', 'mecanum_robot.urdf.xacro')
    rviz_path  = os.path.join(pkg, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch joint_state_publisher_gui for wheel sliders')

    gui = LaunchConfiguration('gui')

    # Process the xacro with the Python module (no dependency on the `xacro`
    # console script being on PATH).
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml,
                     'use_sim_time': False}],
        output='screen',
    )

    # GUI version — shows sliders for each continuous wheel joint
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )

    # Headless version (no sliders) — all joints at zero
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
    )

    # Strip any snap library paths that cause libpthread GLIBC conflicts
    clean_ld = ':'.join(
        p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':')
        if 'snap' not in p
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        additional_env={'LD_LIBRARY_PATH': clean_ld},
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
