#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
Launch file for Gazebo Classic with Velodyne LiDAR simulation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


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
        default_value=os.path.join(pkg_gazebo_lidar, 'worlds', 'velodyne_world.world'),
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
    
    # Process URDF file
    robot_description_file = os.path.join(pkg_gazebo_lidar, 'models', 'velodyne_robot', 'velodyne_robot.urdf.xacro')
    robot_description = xacro.process_file(robot_description_file).toxml()
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'velodyne_robot'],
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
        robot_state_publisher,
        joint_state_publisher,
        gazebo_launch,
        spawn_robot,
        lidar_publisher,
        tcp_lidar_bridge,
        lidar_subscriber
    ])
