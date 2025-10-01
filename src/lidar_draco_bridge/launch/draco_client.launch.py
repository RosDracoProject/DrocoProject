#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for draco client"""
    
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
    
    # Draco Client Node
    client_node = Node(
        package='lidar_draco_bridge',
        executable='draco_client_improved.py',
        name='draco_client',
        output='screen',
        parameters=[{
            'draco_host': LaunchConfiguration('draco_host'),
            'draco_port': LaunchConfiguration('draco_port'),
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            'Starting Draco Client:',
            '  Host: ', LaunchConfiguration('draco_host'),
            '  Port: ', LaunchConfiguration('draco_port'),
        ]
    )
    
    return LaunchDescription([
        draco_host_arg,
        draco_port_arg,
        log_info,
        client_node,
    ])
