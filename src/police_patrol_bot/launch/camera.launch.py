#!/usr/bin/env python3
"""
Launch astra_pro (LeTMC-520) with point cloud enabled, then convert the
depth point cloud to a LaserScan for 2D SLAM / Nav2.

Usage:
    ros2 launch <your_pkg> astra_pro_laserscan.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    astra_launch = os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')

    # ---- Tunables you may want to override at runtime ----
    scan_min_height = LaunchConfiguration('min_height')
    scan_max_height = LaunchConfiguration('max_height')
    cloud_in        = LaunchConfiguration('cloud_in')
    scan_frame      = LaunchConfiguration('scan_frame')

    declare_min_height = DeclareLaunchArgument(
        'min_height', default_value='0.90',
        description='Min height (m) of points included in the scan slice')
    declare_max_height = DeclareLaunchArgument(
        'max_height', default_value='1.10',
        description='Max height (m) of points included in the scan slice')
    declare_cloud_in = DeclareLaunchArgument(
        'cloud_in', default_value='/camera/depth/points',
        description='Input PointCloud2 topic (use depth_registered/points if you '
                    'want the registered cloud)')
    declare_scan_frame = DeclareLaunchArgument(
        'scan_frame', default_value='base_footprint',
        description='target_frame for the emitted LaserScan')

    # ---- Camera ----
    astra = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch),
        launch_arguments={
            'uvc_product_id': '1282',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true',
            'depth_registration': 'true',
        }.items(),
    )

    # ---- PointCloud2 -> LaserScan ----
    pc2ls = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', cloud_in),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': scan_frame,
            'transform_tolerance': 0.01,
            'min_height': scan_min_height,
            'max_height': scan_max_height,
            'angle_min': -1.5708,      # -90 deg
            'angle_max': 1.5708,       #  90 deg
            'angle_increment': 0.0087, # ~0.5 deg
            'scan_time': 0.0333,       # 30 Hz
            'range_min': 0.30,         # LeTMC-520 min usable depth
            'range_max': 8.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            # RDK X5 is CPU-constrained; -1 uses all cores. Set to 1-2 if the
            # BPU/VLM+YOLO load is already saturating things.
            'concurrency_level': 1,
        }],
    )

    return LaunchDescription([
        declare_min_height,
        declare_max_height,
        declare_cloud_in,
        declare_scan_frame,
        astra,
        pc2ls,
    ])
