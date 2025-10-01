#!/bin/bash

# ROS2 패키지 기반 Lidar-Draco Bridge 실행 스크립트

echo "=== ROS2 Package-based Lidar-Draco Bridge ==="
echo "Using lidar_draco_bridge package"
echo "ROS Domain ID: 10"
echo ""

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 도메인 ID 설정
export ROS_DOMAIN_ID=10

echo "Starting LiDAR-Draco Bridge using ROS2 launch..."
echo "Features:"
echo "  - Package-based structure"
echo "  - Launch file support"
echo "  - Parameter configuration"
echo "  - Draco compression"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch 파일로 실행
ros2 launch lidar_draco_bridge lidar_draco_bridge.launch.py
