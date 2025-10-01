#!/bin/bash

# 개선된 ROS2 Lidar-Draco Bridge 실행 스크립트 (point_cloud_transport_py 사용)

echo "=== Improved ROS2 Lidar-Draco Bridge ==="
echo "Using point_cloud_transport_py for efficient compression"
echo "ROS Domain ID: 10"
echo "Lidar Topic: /velodyne2/velodyne_points2"
echo "Output Topic: /lidar/compressed"
echo "Draco Server: localhost:8080"
echo ""

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 도메인 ID 설정
export ROS_DOMAIN_ID=10

echo "Starting Improved Lidar-Draco Bridge..."
echo "Features:"
echo "  - Draco compression support"
echo "  - Zlib compression support"
echo "  - Efficient transport layer"
echo "  - Real-time statistics"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# 개선된 브리지 실행
python3 src/lidar_draco_bridge_improved.py
