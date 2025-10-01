#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
Launch file for Ignition Gazebo 3D LiDAR simulation with TCP Point Cloud Transport
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_gazebo_lidar = get_package_share_directory('gazebo_lidar_simulation')
    
    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_tcp_transport = LaunchConfiguration('use_tcp_transport')
    tcp_server_port = LaunchConfiguration('tcp_server_port')
    
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_gazebo_lidar, 'worlds', 'ignition_lidar_world.sdf'),
        description='Full path to world file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_tcp_transport_arg = DeclareLaunchArgument(
        'use_tcp_transport',
        default_value='true',
        description='Use TCP transport for point cloud transmission'
    )
    
    tcp_server_port_arg = DeclareLaunchArgument(
        'tcp_server_port',
        default_value='8080',
        description='TCP server port for point cloud transmission'
    )
    
    # Ignition Gazebo launch
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_file],
        output='screen'
    )
    
    # ROS-Gazebo bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # ROS-Gazebo bridge for LiDAR data
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        output='screen'
    )
    
    # LiDAR point cloud publisher (Python)
    lidar_publisher = Node(
        package='gazebo_lidar_simulation',
        executable='lidar_publisher.py',
        name='lidar_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_tcp_transport': use_tcp_transport,
            'tcp_server_port': tcp_server_port
        }],
        output='screen'
    )
    
    # TCP LiDAR bridge (if using TCP transport)
    tcp_lidar_bridge = Node(
        package='gazebo_lidar_simulation',
        executable='tcp_lidar_bridge.py',
        name='tcp_lidar_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tcp_server_port': tcp_server_port
        }],
        condition=IfCondition(use_tcp_transport),
        output='screen'
    )
    
    # LiDAR subscriber (for testing)
    lidar_subscriber = Node(
        package='gazebo_lidar_simulation',
        executable='lidar_subscriber.py',
        name='lidar_subscriber',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        use_tcp_transport_arg,
        tcp_server_port_arg,
        gazebo_launch,
        clock_bridge,
        lidar_bridge,
        lidar_publisher,
        tcp_lidar_bridge,
        lidar_subscriber
    ])
