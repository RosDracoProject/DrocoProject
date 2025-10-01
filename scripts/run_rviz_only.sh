#!/bin/bash

# RVIZ2만 실행하는 스크립트 (ROS_DOMAIN_ID=10)

echo "=== RVIZ2 실행 (ROS_DOMAIN_ID=10) ==="
echo "드라코 브리지 데이터를 시각화합니다."
echo ""

# 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID=10

echo "RVIZ2 실행 중..."
echo "다음 토픽들을 확인할 수 있습니다:"
echo "  - /lidar/compressed (압축된 포인트 클라우드)"
echo "  - /sensing/lidar/top/pointcloud_raw_ex (원본 포인트 클라우드)"
echo ""

# RVIZ2 실행
rviz2 -d config/simple_draco_visualization.rviz

