#!/bin/bash

# 통합 드라코 브리지 테스트 (브리지 + 클라이언트 통합)

echo "=== 통합 드라코 브리지 테스트 ==="
echo "브리지가 압축과 압축 해제를 모두 처리합니다."
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
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "통합 브리지 종료 중..."
        kill $BRIDGE_PID 2>/dev/null
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

echo "3단계: 통합 드라코 브리지 실행"
echo "브리지가 압축과 압축 해제를 모두 처리합니다."
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file &
BRIDGE_PID=$!
echo "통합 브리지 PID: $BRIDGE_PID"

echo "4단계: 3초 대기 (브리지 준비)"
sleep 3

echo "5단계: RVIZ2 실행 (시각화)"
rviz2 -d config/simple_draco_visualization.rviz &
RVIZ_PID=$!
echo "RVIZ2 PID: $RVIZ_PID"

echo ""
echo "=== 통합 시스템 실행 완료 ==="
echo "RVIZ2에서 다음 토픽들을 확인할 수 있습니다:"
echo "  - /lidar/decompressed (압축 해제된 포인트 클라우드)"
echo "  - /sensing/lidar/top/pointcloud_raw_ex (원본 포인트 클라우드)"
echo "  - /lidar/compressed (압축된 포인트 클라우드)"
echo ""
echo "Ctrl+C로 모든 프로세스를 종료할 수 있습니다."

# 사용자가 Ctrl+C를 누를 때까지 대기
while true; do
    sleep 1
done
