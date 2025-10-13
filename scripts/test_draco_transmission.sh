#!/bin/bash

# Draco 압축/전송 테스트 스크립트
# Bag 파일 → 압축 → TCP 전송 → 복호화 확인

if [ -z "$1" ]; then
    echo "사용법: $0 <bag_파일_경로>"
    echo "예시: $0 /home/hkit/my_data/lidar_data/full_20251001_163123_0/"
    exit 1
fi

BAG_PATH="$1"

if [ ! -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH" ]; then
    echo "❌ 오류: bag 파일을 찾을 수 없습니다: $BAG_PATH"
    exit 1
fi

export ROS_DOMAIN_ID=15
cd /home/hkit/my_data/final_project/ros2_ws

echo "=========================================="
echo "  📡 Draco 압축/전송 테스트"
echo "=========================================="
echo ""
echo "📁 Bag 파일: $BAG_PATH"
echo ""
echo "데이터 흐름:"
echo "  Bag 파일 → Draco Bridge (압축) → TCP:8888 → Draco Client (복호화)"
echo ""
echo "=========================================="
echo ""

# 환경 설정
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# 1. Bag 파일 재생
echo "[1/4] Bag 파일 재생 시작..."
ros2 bag play "$BAG_PATH" --clock --loop &
BAG_PID=$!
sleep 3

# 2. Draco Bridge (압축 서버) 시작
echo "[2/4] Draco Bridge (압축 서버) 시작..."
echo "      입력: /sensing/lidar/top/pointcloud_raw_ex"
echo "      출력: TCP:8888 (압축 데이터)"
ros2 run draco_bridge_cpp simple_draco_bridge &
BRIDGE_PID=$!
sleep 3

# 3. Draco Client (복호화) 시작
echo "[3/4] Draco Client (복호화) 시작..."
echo "      입력: TCP:8888"
echo "      출력: /lidar/decompressed"
ros2 run draco_bridge_cpp simple_draco_client &
CLIENT_PID=$!
sleep 3

echo "[4/4] 전송 테스트 시작..."
echo ""
echo "=========================================="
echo "  ✅ 시스템 실행 중!"
echo "=========================================="
echo ""
echo "📊 확인 가능한 토픽:"
echo "  - /sensing/lidar/top/pointcloud_raw_ex (원본)"
echo "  - /lidar/compressed (압축됨)"
echo "  - /lidar/decompressed (복호화됨)"
echo ""
echo "📈 성능 확인 명령어 (다른 터미널에서):"
echo "  ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex"
echo "  ros2 topic hz /lidar/decompressed"
echo "  ros2 topic bw /lidar/compressed"
echo ""
echo "🎨 RViz2로 시각화 (다른 터미널에서):"
echo "  rviz2 -d config/simple_draco_visualization.rviz"
echo ""
echo "🛑 종료: Ctrl+C"
echo "=========================================="
echo ""

# 5초 후 상태 확인
sleep 5
echo ""
echo "📊 현재 토픽 상태 확인..."
timeout 3 ros2 topic hz /lidar/decompressed --window 5 2>&1 | head -5 &
RATE_CHECK_PID=$!

# 사용자 입력 대기
wait

# 정리
echo ""
echo "종료 중..."
kill $BAG_PID $BRIDGE_PID $CLIENT_PID $RATE_CHECK_PID 2>/dev/null
wait $BAG_PID $BRIDGE_PID $CLIENT_PID $RATE_CHECK_PID 2>/dev/null
echo "✅ 완료!"

