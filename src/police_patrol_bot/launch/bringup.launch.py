"""
bringup.launch.py — bring the police patrol mecanum base online.

The RP2040 firmware (pico/mecanum_base) runs all kinematics, wheel odometry
and IMU onboard. This launch bridges it to ROS 2 and adds state/TF:

  1. micro_ros_agent      — bridges the Pico USB serial to DDS
                            (Pico pubs: /odom, /wheel_speeds, /imu/data_raw
                             Pico subs: /cmd_vel)
  2. robot_state_publisher — publishes the URDF + fixed TF (incl. base_footprint->base_link)
  3. ekf_filter_node       — fuses /odom + /imu/data_raw -> odom->base_footprint TF
  4. wheel_joint_state_publisher — /wheel_speeds -> /joint_states (animates wheels)

Usage:
  ros2 launch police_patrol_bot bringup.launch.py
  ros2 launch police_patrol_bot bringup.launch.py serial_port:=/dev/ttyACM0
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg      = get_package_share_directory('police_patrol_bot')
    pkg_desc = get_package_share_directory('police_patrol_bot_description')

    urdf_path = os.path.join(pkg_desc, 'urdf', 'mecanum_robot.urdf.xacro')
    ekf_path  = os.path.join(pkg, 'config', 'ekf.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='USB serial port for the RP2040 Pico')
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baud rate for the micro-ROS serial transport')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    serial_port  = LaunchConfiguration('serial_port')
    serial_baud  = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the xacro with the Python module (no dependency on the `xacro`
    # console script being on PATH).
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    # micro_ros_agent is a ROS 2 package executable, not a system binary —
    # run it as a Node (equivalent to `ros2 run micro_ros_agent micro_ros_agent`).
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', serial_port, '-b', serial_baud],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml,
                     'use_sim_time': use_sim_time}],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_path, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Integrate /wheel_speeds into /joint_states so RViz animates the wheels
    wheel_joint_states = Node(
        package='police_patrol_bot',
        executable='wheel_joint_state_publisher.py',
        name='wheel_joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        use_sim_time_arg,
        micro_ros_agent,
        robot_state_publisher,
        ekf,
        wheel_joint_states,
    ])
