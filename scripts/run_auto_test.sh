#!/bin/bash

# 자동 데이터 비교 테스트 스크립트

echo "=== 자동 드라코 데이터 비교 테스트 ==="
echo "이 스크립트는 자동으로 bag 파일을 재생하고 브리지/클라이언트를 실행합니다."
echo ""

# ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID=10

# PYTHONPATH 설정
export PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH

echo "1단계: Bag 파일과 브리지를 함께 실행"
echo "Bag 파일을 백그라운드에서 시작..."
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57/ --loop &
BAG_PID=$!
echo "Bag 파일 PID: $BAG_PID"

echo "2단계: 3초 대기 (데이터 준비)"
sleep 3

echo "3단계: 개선된 브리지 실행 (포그라운드)"
echo "브리지가 실행되면 압축된 데이터를 확인할 수 있습니다."
echo "Ctrl+C로 중단할 수 있습니다."
echo ""

# 브리지를 포그라운드에서 실행 (사용자가 직접 제어)
python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file

echo "7단계: 프로세스 정리"
echo "Bag 파일 종료 중..."
kill $BAG_PID 2>/dev/null
echo "브리지 종료 중..."
kill $BRIDGE_PID 2>/dev/null
echo "클라이언트 종료 중..."
kill $CLIENT_PID 2>/dev/null

echo ""
echo "=== 테스트 완료 ==="
echo "드라코 압축 데이터 전송 테스트가 완료되었습니다!"
echo "로그를 확인하여 압축률과 데이터 품질을 확인하세요."
