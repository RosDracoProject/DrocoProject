#!/bin/bash

# ROS2 Lidar-Draco Bridge 실행 스크립트

echo "=== ROS2 Lidar-Draco Bridge ==="
echo "ROS Domain ID: 10"
echo "Lidar Topic: /velodyne2/velodyne_points2"
echo "Draco Server: localhost:8080"
echo ""

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 도메인 ID 설정
export ROS_DOMAIN_ID=10

echo "Starting Lidar-Draco Bridge..."
echo "Press Ctrl+C to stop"
echo ""

# 브리지 실행
python3 src/lidar_draco_bridge.py
