#!/bin/bash

export ROS_DOMAIN_ID=15
export LD_LIBRARY_PATH=/home/hkit/my_data/final_project/ros2_ws/install/kiss_icp/lib:$LD_LIBRARY_PATH

cd /home/hkit/my_data/final_project/ros2_ws

echo "=== Starting KISS-ICP 3D SLAM System ==="
echo ""

# 1. Start bag file
echo "1. Starting bag file playback..."
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --loop &
BAG_PID=$!
sleep 2

# 2. Start KISS-ICP node
echo "2. Starting KISS-ICP node..."
/home/hkit/my_data/final_project/ros2_ws/install/kiss_icp/lib/kiss_icp/kiss_icp_node \
  --ros-args \
  -r pointcloud_topic:=/sensing/lidar/top/pointcloud_raw_ex \
  -p max_range:=100.0 \
  -p min_range:=0.5 \
  -p base_frame:=rslidar_top \
  -p publish_odom_tf:=true \
  -p publish_debug_clouds:=true &
KISS_PID=$!
sleep 2

# 3. Start static TF (map -> odom_lidar)
echo "3. Starting TF..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom_lidar &
TF_PID=$!
sleep 1

# 4. Start RVIZ2
echo "4. Starting RVIZ2..."
echo ""
echo "=== All systems running! ==="
echo "Press Ctrl+C to stop"
rviz2 -d config/kiss_slam_visualization.rviz

# Cleanup on exit
kill $BAG_PID $KISS_PID $TF_PID 2>/dev/null
echo "Stopped."

