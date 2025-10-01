#!/bin/bash

# 완전한 드라코 브리지 테스트 (Bag + Bridge + Client + RVIZ2)

echo "=== 완전한 드라코 브리지 테스트 ==="
echo "Bag 파일 + 브리지 + 클라이언트 + RVIZ2를 모두 실행합니다."
echo ""

# 환경 설정
export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
source install/setup.bash

# 시그널 핸들러 설정
cleanup() {
    echo ""
    echo "=== 모든 프로세스 종료 중 ==="
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
    if [ ! -z "$RVIZ_PID" ]; then
        echo "RVIZ2 종료 중..."
        kill $RVIZ_PID 2>/dev/null
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
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file &
BRIDGE_PID=$!
echo "브리지 PID: $BRIDGE_PID"

echo "4단계: 3초 대기 (브리지 준비)"
sleep 3

echo "5단계: 드라코 클라이언트 실행"
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/draco_client_improved.py &
CLIENT_PID=$!
echo "클라이언트 PID: $CLIENT_PID"

echo "6단계: 3초 대기 (클라이언트 준비)"
sleep 3

echo "7단계: RVIZ2 실행 (시각화)"
rviz2 -d config/simple_draco_visualization.rviz &
RVIZ_PID=$!
echo "RVIZ2 PID: $RVIZ_PID"

echo ""
echo "=== 모든 시스템 실행 완료 ==="
echo "RVIZ2에서 다음 토픽들을 확인할 수 있습니다:"
echo "  - /lidar/decompressed (압축 해제된 포인트 클라우드)"
echo "  - /sensing/lidar/top/pointcloud_raw_ex (원본 포인트 클라우드)"
echo ""
echo "Ctrl+C로 모든 프로세스를 종료할 수 있습니다."

# 사용자가 Ctrl+C를 누를 때까지 대기
while true; do
    sleep 1
done
