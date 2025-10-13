#!/bin/bash

# 범용 KISS-ICP SLAM 실행 스크립트
# 자동으로 bag 파일의 frame_id를 감지하고 설정합니다

if [ -z "$1" ]; then
    echo "사용법: $0 <bag_파일_경로> [frame_id]"
    echo "예시: $0 /home/hkit/my_data/lidar_data/full_20251001_163123_0/"
    echo "      $0 /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 rslidar_top"
    exit 1
fi

BAG_PATH="$1"
MANUAL_FRAME_ID="$2"

if [ ! -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH" ]; then
    echo "❌ 오류: bag 파일을 찾을 수 없습니다: $BAG_PATH"
    exit 1
fi

export ROS_DOMAIN_ID=15
cd /home/hkit/my_data/final_project/ros2_ws

echo "=========================================="
echo "  🗺️  범용 KISS-ICP 3D SLAM"
echo "=========================================="
echo ""
echo "📁 Bag 파일: $BAG_PATH"
echo ""

# frame_id 결정
if [ -n "$MANUAL_FRAME_ID" ]; then
    echo "[1/3] 수동 지정된 frame_id 사용..."
    FRAME_ID="$MANUAL_FRAME_ID"
    echo "✅ Frame ID: $FRAME_ID (수동 설정)"
else
    # frame_id 감지를 위해 bag 파일을 짧게 재생
    echo "[1/3] frame_id 자동 감지 중..."
    echo "    bag 파일 재생 시작..."
    
    # ROS_DOMAIN_ID 설정하고 bag 재생
    export ROS_DOMAIN_ID=15
    ros2 bag play "$BAG_PATH" --clock > /dev/null 2>&1 &
    DETECT_PID=$!
    
    echo "    데이터 대기 중 (최대 15초)..."
    
    # 여러 번 시도
    FRAME_ID=""
    for i in {1..5}; do
        sleep 3
        echo "    시도 $i/5..."
        FRAME_ID=$(timeout 5 ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex header.frame_id --once 2>/dev/null | grep -v "frame_id:" | grep -v "^$" | tr -d ' \n\r')
        
        if [ -n "$FRAME_ID" ]; then
            echo "✅ 감지 성공: $FRAME_ID"
            break
        fi
    done

    # 감지 프로세스 정리
    kill $DETECT_PID 2>/dev/null
    wait $DETECT_PID 2>/dev/null
    sleep 1

    if [ -z "$FRAME_ID" ]; then
        echo ""
        echo "❌ frame_id 자동 감지 실패!"
        echo ""
        echo "수동으로 frame_id를 지정해서 다시 실행해주세요:"
        echo "  - rslidar_top: $0 \"$BAG_PATH\" rslidar_top"
        echo "  - lidar_top:   $0 \"$BAG_PATH\" lidar_top"
        echo ""
        exit 1
    fi
fi

# 정식 Bag 파일 재생 시작
echo "[2/3] Bag 파일 재생 중..."
ros2 bag play "$BAG_PATH" --clock &
BAG_PID=$!
sleep 2

# KISS-ICP 실행
echo "[3/3] KISS-ICP 시작..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

echo ""
echo "=========================================="
echo "  ✅ 시스템 실행 중!"
echo "=========================================="
echo ""
echo "📊 설정:"
echo "  - 입력 토픽: /sensing/lidar/top/pointcloud_raw_ex"
echo "  - Base Frame: $FRAME_ID"
echo "  - Fixed Frame: $FRAME_ID (RViz2에서 설정)"
echo ""
echo "🎨 RViz2에서 확인할 것:"
echo "  1. Fixed Frame을 '$FRAME_ID'로 설정"
echo "  2. /kiss/local_map: 3D 맵 (무지개색)"
echo "  3. /kiss/frame: 현재 스캔 (하늘색)"
echo ""
echo "🛑 종료: Ctrl+C"
echo "=========================================="
echo ""

ros2 launch kiss_icp odometry.launch.py \
  topic:=/sensing/lidar/top/pointcloud_raw_ex \
  visualize:=true \
  base_frame:=$FRAME_ID \
  publish_debug_clouds:=true \
  use_sim_time:=true

# 정리
echo ""
echo "종료 중..."
kill $BAG_PID 2>/dev/null
wait $BAG_PID 2>/dev/null
echo "✅ 완료!"

