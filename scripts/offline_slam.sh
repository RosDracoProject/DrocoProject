#!/bin/bash

# Offline SLAM using bag file
# This script processes the bag file to create a map

export ROS_DOMAIN_ID=15

echo "=== Offline SLAM for LiDAR-only bag file ==="
echo ""
echo "Starting all nodes..."

# 1. Play bag file (not looping, just once)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock &
BAG_PID=$!
sleep 2

# 2. Convert PointCloud to LaserScan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/sensing/lidar/top/pointcloud_raw_ex \
  -p target_frame:=base_link \
  -p min_height:=-0.5 \
  -p max_height:=3.0 \
  -p use_sim_time:=true &
P2L_PID=$!

# 3. Static TFs
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom --ros-args -p use_sim_time:=true &
TF1_PID=$!

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link --ros-args -p use_sim_time:=true &
TF2_PID=$!

sleep 2

# 4. SLAM Toolbox (offline mode)
ros2 launch slam_toolbox offline_launch.py use_sim_time:=true &
SLAM_PID=$!

echo ""
echo "All nodes started. Processing bag file..."
echo "Press Ctrl+C to stop"

# Wait for bag to finish
wait $BAG_PID

echo ""
echo "Bag file playback complete. SLAM processing done."
echo "Map saved to ~/.ros/slam_toolbox_map.data"

# Keep SLAM running for a bit to finalize
sleep 5

# Cleanup
kill $P2L_PID $TF1_PID $TF2_PID $SLAM_PID 2>/dev/null

echo "Done!"

