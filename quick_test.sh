#!/bin/bash

# 빠른 드라코 테스트 스크립트

echo "🚀 빠른 드라코 테스트 시작!"

# 환경 설정
export ROS_DOMAIN_ID=10
export PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH

# 1. Bag 파일 재생
echo "📦 Bag 파일 재생 중..."
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57/ --loop &
BAG_PID=$!

# 2. 잠시 대기
sleep 2

# 3. 브리지 실행
echo "🌉 브리지 실행 중..."
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file &
BRIDGE_PID=$!

# 4. 잠시 대기
sleep 2

# 5. 클라이언트 실행 (20초간)
echo "📱 클라이언트 실행 중... (20초간 테스트)"
timeout 20s python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/draco_client_improved.py

# 6. 정리
echo "🧹 정리 중..."
kill $BAG_PID $BRIDGE_PID 2>/dev/null

echo "✅ 테스트 완료!"
