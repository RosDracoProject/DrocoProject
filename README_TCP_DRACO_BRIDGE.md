# TCP/IP 드라코 브리지 테스트 가이드

## 개요
이 프로젝트는 ROS2 환경에서 라이다 포인트 클라우드 데이터를 TCP/IP를 통해 압축/전송하는 시스템입니다.

## 시스템 구조
```
Bag 파일 → 서버(압축) → TCP/IP 전송 → 클라이언트(복호화) → RVIZ2
```

## 실행 방법 (같은 컴퓨터에서 테스트)

### 터미널 1: Bag 파일 재생
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock
```

### 터미널 2: 서버 실행 (압축 + 전송)
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args -p compression_level:=6
```

**압축 레벨 옵션:**
- `-p compression_level:=1` : 빠른 속도 (낮은 압축률)
- `-p compression_level:=6` : 균형 (기본값)
- `-p compression_level:=9` : 높은 압축률 (느린 속도)

### 터미널 3: 클라이언트 실행 (수신 + 복호화)
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client
```

### 터미널 4: 성능 확인 (선택사항)
```bash
export ROS_DOMAIN_ID=15
ros2 topic hz /lidar/decompressed
```

### 터미널 5: RVIZ2 시각화 (선택사항)
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15
rviz2 -d config/simple_draco_visualization.rviz
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
- **서버 주소**: 
  - 같은 컴퓨터: `127.0.0.1` (로컬호스트)
  - 다른 컴퓨터: 서버 컴퓨터의 IP 주소 (예: `192.168.3.251`)

## 압축 설정
- **압축 알고리즘**: zlib
- **압축 레벨**: 0-9 (기본값: 6)
  - 레벨 1: 빠른 속도, 낮은 압축률
  - 레벨 6: 균형 (기본값)
  - 레벨 9: 느린 속도, 높은 압축률
- **청크 크기**: 64KB (TCP 전송용)

### 압축 레벨 변경 방법
```bash
# 서버 실행 시 압축 레벨 지정
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args -p compression_level:=1
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args -p compression_level:=6
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args -p compression_level:=9
```

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

## 다른 컴퓨터에서 클라이언트 실행 (컴퓨터 간 통신)

### 1️⃣ 서버 컴퓨터 설정 (이 컴퓨터)

#### IP 주소 확인
```bash
hostname -I
# 예: 192.168.3.251
```

#### 방화벽 포트 열기
```bash
sudo ufw allow 8888/tcp
```

#### 서버 실행
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15

# Bag 파일 재생 (터미널 1)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock

# 서버 실행 (터미널 2)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args -p compression_level:=6
```

### 2️⃣ 클라이언트 컴퓨터 설정 (다른 컴퓨터)

#### 필수 패키지 설치
```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 필수 패키지 설치
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  python3-colcon-common-extensions \
  zlib1g-dev
```

#### 소스 코드 복사 및 빌드
```bash
# 서버 컴퓨터에서 소스 압축
cd /home/hkit/my_data/final_project/ros2_ws
tar -czf draco_bridge_src.tar.gz src/draco_bridge_cpp/

# 클라이언트 컴퓨터로 전송 (USB 또는 scp)
# scp draco_bridge_src.tar.gz user@클라이언트IP:~/

# 클라이언트 컴퓨터에서 압축 해제 및 빌드
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
tar -xzf ~/draco_bridge_src.tar.gz -C .

source /opt/ros/humble/setup.bash
colcon build --packages-select draco_bridge_cpp --symlink-install
```

#### 클라이언트 실행
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client --ros-args -p server_host:=192.168.3.251
```

### 3️⃣ 연결 테스트

#### 네트워크 연결 확인 (클라이언트에서)
```bash
# Ping 테스트
ping 192.168.3.251

# 포트 연결 테스트
telnet 192.168.3.251 8888
# 또는
nc -v 192.168.3.251 8888
```

#### 데이터 수신 확인 (클라이언트에서)
```bash
export ROS_DOMAIN_ID=15
ros2 topic hz /lidar/decompressed
ros2 topic echo /lidar/decompressed --no-arr
```

## 문제 해결

### 1. 클라이언트 연결 실패
```
[ERROR] [simple_draco_client]: Failed to connect to server
```
**원인**:
- 서버가 실행되지 않음
- 방화벽이 8888 포트를 차단
- 네트워크 연결 문제
- 잘못된 서버 IP 주소

**해결방법**:
```bash
# 서버 실행 확인
ps aux | grep simple_draco_bridge

# 포트 리스닝 확인
ss -tln | grep 8888

# 방화벽 설정 확인
sudo ufw status
sudo ufw allow 8888/tcp

# 네트워크 연결 테스트
ping 서버IP
telnet 서버IP 8888
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
