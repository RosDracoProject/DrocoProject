#!/bin/bash

# 헤더 정보 확인 스크립트

export ROS_DOMAIN_ID=15
BAG_FILE="/home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57"
TOPIC="/sensing/lidar/top/pointcloud_raw_ex"

echo "================================"
echo "Bag 파일 헤더 정보 확인"
echo "================================"
echo ""

# Bag 파일 재생 (백그라운드)
echo "Bag 파일 재생 중..."
ros2 bag play "$BAG_FILE" --clock &>/dev/null &
BAG_PID=$!

# 잠시 대기
sleep 3

# 헤더 정보 확인
echo ""
echo "========== 헤더 정보 =========="
ros2 topic echo "$TOPIC" --once --field header 2>/dev/null

# Bag 프로세스 종료
kill $BAG_PID 2>/dev/null
wait $BAG_PID 2>/dev/null

echo ""
echo "================================"
echo "확인 완료"
echo "================================"

