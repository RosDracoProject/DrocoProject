#!/bin/bash

# 무한 반복 자동 테스트 스크립트
# Ctrl+C로 중단할 수 있습니다

echo "=== 무한 반복 드라코 데이터 테스트 ==="
echo "이 스크립트는 bag 파일을 무한 반복하며 브리지/클라이언트를 계속 실행합니다."
echo "Ctrl+C를 눌러 중단할 수 있습니다."
echo ""

# ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID=10

# PYTHONPATH 설정
export PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH

# 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# 시그널 핸들러 설정
cleanup() {
    echo ""
    echo "=== 테스트 중단됨 ==="
    echo "프로세스 정리 중..."
    
    # 모든 프로세스 종료
    if [ ! -z "$BAG_PID" ]; then
        echo "Bag 파일 종료 중..."
        kill $BAG_PID 2>/dev/null
    fi
    
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "브리지 종료 중..."
        kill $BRIDGE_PID 2>/dev/null
    fi
    
    if [ ! -z "$CLIENT_PID" ]; then
        echo "클라이언트 종료 중..."
        kill $CLIENT_PID 2>/dev/null
    fi
    
    echo "정리 완료!"
    exit 0
}

# 시그널 트랩 설정
trap cleanup SIGINT SIGTERM

echo "1단계: Bag 파일 재생 시작 (무한 반복)"
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57/ --loop &
BAG_PID=$!
echo "Bag 파일 PID: $BAG_PID (무한 반복 모드)"

echo "2단계: 3초 대기 (데이터 준비)"
sleep 3

echo "3단계: 개선된 브리지 실행 (백그라운드)"
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file &
BRIDGE_PID=$!
echo "개선된 브리지 PID: $BRIDGE_PID"

echo "4단계: 3초 대기 (브리지 준비)"
sleep 3

# 무한 루프 시작
CYCLE_COUNT=0
while true; do
    CYCLE_COUNT=$((CYCLE_COUNT + 1))
    echo ""
    echo "=== 사이클 #$CYCLE_COUNT 시작 ==="
    echo "시간: $(date)"
    
    echo "5단계: 개선된 클라이언트 실행 (30초간 테스트)"
    timeout 30s python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/draco_client_improved.py &
    CLIENT_PID=$!
    
    echo "6단계: 테스트 진행 중... (30초)"
    sleep 30
    
    echo "7단계: 클라이언트 종료"
    kill $CLIENT_PID 2>/dev/null
    wait $CLIENT_PID 2>/dev/null
    
    echo "8단계: 5초 대기 (다음 사이클 준비)"
    sleep 5
    
    echo "=== 사이클 #$CYCLE_COUNT 완료 ==="
    echo "총 처리된 사이클: $CYCLE_COUNT"
    echo "다음 사이클을 위해 대기 중... (Ctrl+C로 중단)"
done
