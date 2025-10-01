#!/bin/bash

# RVIZ2와 함께 드라코 브리지 테스트 스크립트

echo "=== RVIZ2와 함께 드라코 브리지 테스트 ==="
echo "이 스크립트는 bag 파일을 재생하고 브리지/클라이언트를 실행하며 RVIZ2에서 시각화합니다."
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
    
    if [ ! -z "$RVIZ_PID" ]; then
        echo "RVIZ2 종료 중..."
        kill $RVIZ_PID 2>/dev/null
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

echo "5단계: RVIZ2 실행 (시각화) - ROS_DOMAIN_ID=10"
ROS_DOMAIN_ID=10 rviz2 -d config/simple_draco_visualization.rviz &
RVIZ_PID=$!
echo "RVIZ2 PID: $RVIZ_PID (DOMAIN_ID=10)"

echo "6단계: 2초 대기 (RVIZ2 준비)"
sleep 2

echo "7단계: 개선된 클라이언트 실행 (60초간 테스트)"
timeout 60s python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/draco_client_improved.py &
CLIENT_PID=$!

echo "8단계: 테스트 진행 중... (60초)"
echo "RVIZ2에서 다음 토픽들을 확인할 수 있습니다:"
echo "  - /lidar/compressed (압축된 포인트 클라우드)"
echo "  - /sensing/lidar/top/pointcloud_raw_ex (원본 포인트 클라우드)"
echo "  - /velodyne2/velodyne_points2 (시뮬레이션 데이터)"
echo ""
sleep 60

echo "9단계: 클라이언트 종료"
kill $CLIENT_PID 2>/dev/null
wait $CLIENT_PID 2>/dev/null

echo ""
echo "=== 테스트 완료 ==="
echo "RVIZ2에서 압축된 포인트 클라우드를 확인했습니다!"
echo "프로세스를 정리하려면 Ctrl+C를 누르세요."

# 사용자가 Ctrl+C를 누를 때까지 대기
while true; do
    sleep 1
done
