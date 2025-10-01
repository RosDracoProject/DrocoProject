#!/bin/bash

cd /home/hkit/my_data/final_project/ros2_ws

export ROS_DOMAIN_ID=15

echo "=== 3D LiDAR SLAM System ==="
echo ""
echo "Step 1: Bag 파일을 먼저 실행해주세요 (별도 터미널):"
echo "  ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --loop --clock"
echo ""
echo "Step 2: KISS-ICP SLAM + RVIZ2 실행 중..."
echo ""

# Source setup files
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch KISS-ICP with RVIZ2
ros2 launch kiss_icp odometry.launch.py \
  topic:=/sensing/lidar/top/pointcloud_raw_ex \
  visualize:=true \
  base_frame:=rslidar_top \
  publish_debug_clouds:=true \
  use_sim_time:=true

