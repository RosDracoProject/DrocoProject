#!/bin/bash

# TCP/IP 드라코 브리지 테스트 스크립트

echo "=== TCP/IP 드라코 브리지 테스트 시작 ==="

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source /home/hkit/my_data/final_project/ros2_ws/install/setup.bash

# 도메인 ID 설정
export ROS_DOMAIN_ID=15

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 서버 실행 (백그라운드)
echo "서버 시작 중..."
ros2 run draco_bridge_cpp simple_draco_bridge &
SERVER_PID=$!

# 잠시 대기
sleep 3

# Bag 파일 재생 (백그라운드)
echo "Bag 파일 재생 시작..."
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --loop &
BAG_PID=$!

echo "=== 서버와 Bag 파일 실행 완료 ==="
echo "서버 PID: $SERVER_PID"
echo "Bag PID: $BAG_PID"
echo ""
echo "Ctrl+C로 종료하세요"

# 종료 시 모든 프로세스 정리
cleanup() {
    echo "프로세스 종료 중..."
    kill $SERVER_PID $BAG_PID 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM

# 대기
wait
