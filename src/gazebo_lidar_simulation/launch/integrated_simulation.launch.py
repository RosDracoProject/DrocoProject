#!/usr/bin/env python3
# Copyright (c) 2024, Gazebo LiDAR Simulation

"""
Integrated Launch File for mobile-3d-lidar-sim and point_cloud_transport
This launch file runs both simulation systems and bridges them together.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    gazebo_lidar_pkg = get_package_share_directory('gazebo_lidar_simulation')
    mobile_lidar_pkg = '/home/hkit/my_data/final_project/ros2_ws/src/mobile-3d-lidar-sim/my_bot'
    
    # Launch arguments
    use_tcp_transport_arg = DeclareLaunchArgument(
        'use_tcp_transport',
        default_value='true',
        description='Enable TCP transport for point cloud data'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='8080',
        description='TCP port for transport'
    )
    
    compression_level_arg = DeclareLaunchArgument(
        'compression_level',
        default_value='6',
        description='Compression level for zlib (1-9)'
    )
    
    enable_velodyne_to_transport_arg = DeclareLaunchArgument(
        'enable_velodyne_to_transport',
        default_value='true',
        description='Enable bridge from Velodyne to transport'
    )
    
    enable_transport_to_velodyne_arg = DeclareLaunchArgument(
        'enable_transport_to_velodyne',
        default_value='false',
        description='Enable bridge from transport to Velodyne'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(mobile_lidar_pkg, 'worlds', 'obstacles.world'),
        description='World file for Gazebo simulation'
    )
    
    # Get launch configuration
    use_tcp_transport = LaunchConfiguration('use_tcp_transport')
    tcp_port = LaunchConfiguration('tcp_port')
    compression_level = LaunchConfiguration('compression_level')
    enable_velodyne_to_transport = LaunchConfiguration('enable_velodyne_to_transport')
    enable_transport_to_velodyne = LaunchConfiguration('enable_transport_to_velodyne')
    world_file = LaunchConfiguration('world_file')
    
    # 1. Launch mobile-3d-lidar-sim (Velodyne robot)
    mobile_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(mobile_lidar_pkg, 'launch', 'launch_sim.launch.py')
        ]),
        launch_arguments={
            'world': world_file
        }.items()
    )
    
    # 2. Launch point_cloud_transport simulation (Ignition Gazebo)
    ignition_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_lidar_pkg, 'launch', 'ignition_lidar.launch.py')
        ])
    )
    
    # 3. Simulation Bridge Node
    simulation_bridge_node = Node(
        package='gazebo_lidar_simulation',
        executable='simulation_bridge.py',
        name='simulation_bridge',
        output='screen',
        parameters=[{
            'use_tcp_transport': use_tcp_transport,
            'tcp_server_port': tcp_port,
            'compression_level': compression_level,
            'enable_velodyne_to_transport': enable_velodyne_to_transport,
            'enable_transport_to_velodyne': enable_transport_to_velodyne
        }]
    )
    
    # 4. LiDAR Publisher (for point_cloud_transport)
    lidar_publisher_node = Node(
        package='gazebo_lidar_simulation',
        executable='lidar_publisher.py',
        name='lidar_publisher',
        output='screen',
        parameters=[{
            'use_tcp_transport': use_tcp_transport,
            'tcp_server_port': tcp_port
        }]
    )
    
    # 5. LiDAR Subscriber (for testing)
    lidar_subscriber_node = Node(
        package='gazebo_lidar_simulation',
        executable='lidar_subscriber.py',
        name='lidar_subscriber',
        output='screen'
    )
    
    # 6. RViz for visualization
    rviz_config_file = os.path.join(gazebo_lidar_pkg, 'config', 'integrated_simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # 7. Teleop for robot control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_tcp_transport_arg,
        tcp_port_arg,
        compression_level_arg,
        enable_velodyne_to_transport_arg,
        enable_transport_to_velodyne_arg,
        world_file_arg,
        
        # Launch files
        mobile_lidar_launch,
        ignition_lidar_launch,
        
        # Nodes
        simulation_bridge_node,
        lidar_publisher_node,
        lidar_subscriber_node,
        rviz_node,
        teleop_node,
    ])
