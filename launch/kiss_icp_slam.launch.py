#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        default_value='/home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57',
        description='Path to bag file'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/sensing/lidar/top/pointcloud_raw_ex',
        description='PointCloud2 topic name'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable RVIZ2 visualization'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='rslidar_top',
        description='Base frame for the robot'
    )
    
    publish_debug_clouds_arg = DeclareLaunchArgument(
        'publish_debug_clouds',
        default_value='true',
        description='Publish debug point clouds'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag file'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop bag file playback'
    )
    
    # Bag file playback
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bagfile'), '--loop', '--clock'],
        output='screen',
        shell=False
    )
    
    # KISS-ICP launch include
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kiss_icp'),
                'launch',
                'odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'topic': LaunchConfiguration('topic'),
            'visualize': LaunchConfiguration('visualize'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_debug_clouds': LaunchConfiguration('publish_debug_clouds'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    return LaunchDescription([
        bagfile_arg,
        topic_arg,
        visualize_arg,
        base_frame_arg,
        publish_debug_clouds_arg,
        use_sim_time_arg,
        loop_arg,
        bag_play,
        kiss_icp_launch,
    ])

