# TCP/IP 드라코 브리지 테스트 가이드

## 개요
이 프로젝트는 ROS2 환경에서 라이다 포인트 클라우드 데이터를 TCP/IP를 통해 압축/전송하는 시스템입니다.

## 시스템 구조
```
Bag 파일 → 서버(압축) → TCP/IP 전송 → 클라이언트(복호화) → RVIZ2
```

## 실행 방법

### 1. 서버 및 데이터 소스 실행
```bash
cd /home/hkit/my_data/final_project/ros2_ws && ./run_tcp_test.sh
```
이 명령어는 다음을 실행합니다:
- 서버 (`simple_draco_bridge`) 실행
- Bag 파일 재생 (`rosbag2_2024_09_24-14_28_57`)

### 2. 클라이언트 실행
```bash
cd /home/hkit/my_data/final_project/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=15 && ros2 run draco_bridge_cpp simple_draco_client
```

### 3. RVIZ2 시각화 (선택사항)
```bash
cd /home/hkit/my_data/final_project/ros2_ws && export ROS_DOMAIN_ID=15 && rviz2 -d config/simple_draco_visualization.rviz
```

## 토픽 정보

### 입력 토픽
- `/sensing/lidar/top/pointcloud_raw_ex`: 원본 라이다 데이터

### 출력 토픽
- `/lidar/compressed`: 압축된 데이터 (서버에서 발행)
- `/lidar/decompressed`: 복호화된 데이터 (클라이언트에서 발행)

## 네트워크 설정
- **포트**: 8888
- **프로토콜**: TCP/IP
- **서버 주소**: 127.0.0.1 (로컬호스트)

## 압축 설정
- **압축 알고리즘**: zlib
- **압축 레벨**: Z_DEFAULT_COMPRESSION
- **청크 크기**: 64KB (TCP 전송용)

## 성능 모니터링

### 데이터 전송률 확인
```bash
# 원본 데이터
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex

# 압축된 데이터
ros2 topic hz /lidar/compressed

# 복호화된 데이터
ros2 topic hz /lidar/decompressed
```

### 압축 통계
서버에서 10개 메시지마다 다음 통계를 출력합니다:
- 처리된 메시지 수
- 원본 크기 (MB)
- 압축 크기 (MB)
- 압축률 (비율)
- 압축 비율 (%)

## 문제 해결

### 1. 클라이언트 연결 실패
```
[ERROR] [simple_draco_client]: Failed to connect to server
```
**해결방법**: 서버가 실행 중인지 확인
```bash
ps aux | grep simple_draco_bridge
```

### 2. 데이터가 전송되지 않음
**확인사항**:
- Bag 파일이 재생되고 있는지 확인
- 서버가 원본 토픽을 구독하고 있는지 확인
- 클라이언트가 서버에 연결되었는지 확인

### 3. RVIZ2에서 데이터가 보이지 않음
**확인사항**:
- ROS_DOMAIN_ID가 일치하는지 확인 (15)
- Fixed Frame이 올바른지 확인 (`rslidar_top`)
- QoS 설정이 호환되는지 확인

## 환경 설정
- **ROS2**: Humble
- **도메인 ID**: 15
- **C++ 표준**: C++17
- **의존성**: zlib, rclcpp, sensor_msgs

## 파일 구조
```
src/draco_bridge_cpp/
├── src/
│   ├── simple_draco_bridge.cpp    # 서버 (압축 및 전송)
│   └── simple_draco_client.cpp    # 클라이언트 (수신 및 복호화)
├── CMakeLists.txt
└── package.xml

config/
└── simple_draco_visualization.rviz  # RVIZ2 설정

run_tcp_test.sh                      # 테스트 스크립트
```

## 종료 방법
- `Ctrl+C`로 각 프로세스 종료
- 또는 `pkill -f simple_draco_bridge`로 서버 강제 종료
