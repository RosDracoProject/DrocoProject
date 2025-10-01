#!/bin/bash

# 개선된 Draco 클라이언트 실행 스크립트 (point_cloud_transport_py 사용)

echo "=== Improved Draco Client ==="
echo "Using point_cloud_transport_py for efficient data reception"
echo "Connecting to: localhost:8080"
echo "Local topic: /lidar/compressed"
echo ""

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS 도메인 ID 설정
export ROS_DOMAIN_ID=10

echo "Starting Improved Draco Client..."
echo "Features:"
echo "  - Dual data reception (TCP + ROS2)"
echo "  - Compression ratio monitoring"
echo "  - Real-time statistics"
echo "  - Efficient transport layer"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# 개선된 클라이언트 실행
python3 src/draco_client_improved.py
