#!/bin/bash

# Bag 파일과 브리지를 함께 실행하는 스크립트

echo "=== 드라코 브리지 + Bag 파일 통합 실행 ==="
echo "Bag 파일을 재생하면서 브리지가 압축된 데이터를 처리합니다."
echo ""

# 환경 설정
export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
source install/setup.bash

# 시그널 핸들러 설정
cleanup() {
    echo ""
    echo "=== 종료 중 ==="
    if [ ! -z "$BAG_PID" ]; then
        echo "Bag 파일 종료 중..."
        kill $BAG_PID 2>/dev/null
    fi
    echo "정리 완료!"
    exit 0
}

# 시그널 트랩 설정
trap cleanup SIGINT SIGTERM

echo "1단계: Bag 파일 재생 시작"
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57/ --loop &
BAG_PID=$!
echo "Bag 파일 PID: $BAG_PID"

echo "2단계: 3초 대기 (데이터 준비)"
sleep 3

echo "3단계: 드라코 브리지 실행"
echo "브리지가 압축된 포인트 클라우드를 처리합니다."
echo "RVIZ2에서 /lidar/compressed 토픽을 확인하세요."
echo "Ctrl+C로 중단할 수 있습니다."
echo ""

# 브리지 실행 (포그라운드)
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file
