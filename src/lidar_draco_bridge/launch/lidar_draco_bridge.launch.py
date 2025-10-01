#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for lidar_draco_bridge"""
    
    # Package directory
    pkg_share = FindPackageShare(package='lidar_draco_bridge').find('lidar_draco_bridge')
    config_dir = os.path.join(pkg_share, 'config')
    
    # Launch arguments
    draco_host_arg = DeclareLaunchArgument(
        'draco_host',
        default_value='localhost',
        description='Draco server host'
    )
    
    draco_port_arg = DeclareLaunchArgument(
        'draco_port',
        default_value='8080',
        description='Draco server port'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/velodyne2/velodyne_points2',
        description='Input LiDAR topic'
    )
    
    compression_type_arg = DeclareLaunchArgument(
        'compression_type',
        default_value='draco',
        description='Compression type (draco, zlib)'
    )
    
    compression_level_arg = DeclareLaunchArgument(
        'compression_level',
        default_value='6',
        description='Compression level (0-10)'
    )
    
    # LiDAR-Draco Bridge Node
    bridge_node = Node(
        package='lidar_draco_bridge',
        executable='lidar_draco_bridge_improved.py',
        name='lidar_draco_bridge',
        output='screen',
        parameters=[{
            'draco_server_host': LaunchConfiguration('draco_host'),
            'draco_server_port': LaunchConfiguration('draco_port'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'compression_type': LaunchConfiguration('compression_type'),
            'compression_level': LaunchConfiguration('compression_level'),
        }],
        remappings=[
            ('/velodyne2/velodyne_points2', LaunchConfiguration('lidar_topic')),
        ]
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            'Starting LiDAR-Draco Bridge with:',
            '  Host: ', LaunchConfiguration('draco_host'),
            '  Port: ', LaunchConfiguration('draco_port'),
            '  Topic: ', LaunchConfiguration('lidar_topic'),
            '  Compression: ', LaunchConfiguration('compression_type'),
        ]
    )
    
    return LaunchDescription([
        draco_host_arg,
        draco_port_arg,
        lidar_topic_arg,
        compression_type_arg,
        compression_level_arg,
        log_info,
        bridge_node,
    ])
