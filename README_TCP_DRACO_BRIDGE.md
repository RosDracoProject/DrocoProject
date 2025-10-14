# TCP/IP 드라코 브리지 테스트 가이드

## 개요
이 프로젝트는 ROS2 환경에서 라이다 포인트 클라우드 데이터를 **Google Draco 압축**을 사용하여 TCP/IP를 통해 전송하는 시스템입니다.

**특징:**
- ✅ **Google Draco 압축**: 3D 포인트 클라우드 특화 압축 (10:1 ~ 30:1 압축률)
- ✅ **헤더 정보 전송**: frame_id, timestamp 등 메타데이터 보존
- ✅ **TCP/IP 네트워크 전송**: 서버-클라이언트 구조
- ✅ **실시간 처리**: 10Hz 라이다 데이터 실시간 압축/전송

## 시스템 구조
```
Bag 파일 → 서버(Draco 압축) → TCP/IP 전송 → 클라이언트(Draco 복호화) → RVIZ2/KISS-ICP
```

## 실행 방법 (같은 컴퓨터에서 테스트)

### 터미널 1: Bag 파일 재생
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15

# 일반 재생 (한 번만 재생)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock

# 무한 반복 재생 (계속 반복)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock --loop --start-paused
```

### 터미널 2: 서버 실행 (Draco 압축 + 전송)
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15

# 기본 설정으로 실행
ros2 run draco_bridge_cpp simple_draco_bridge

# 또는 파라미터 지정
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=11 \
  -p compression_speed:=5
```

**Draco 압축 옵션:**
- `-p quantization_bits:=11` : 양자화 비트 (기본값: 11, 범위: 1-31)
  - 높을수록: 높은 정밀도, 낮은 압축률
  - 낮을수록: 낮은 정밀도, 높은 압축률
- `-p compression_speed:=5` : 압축 속도 (기본값: 5, 범위: 0-10)
  - 0: 가장 느림, 최고 압축률
  - 10: 가장 빠름, 낮은 압축률

### 터미널 3: 클라이언트 실행 (Draco 복호화)
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15

# 로컬 테스트 (같은 컴퓨터)
ros2 run draco_bridge_cpp simple_draco_client

# 원격 서버 연결 (다른 컴퓨터)
ros2 run draco_bridge_cpp simple_draco_client --ros-args \
  -p server_host:=192.168.3.251 \
  -p server_port:=8888
```

### 터미널 4: KISS-ICP SLAM (선택사항)
```bash
export ROS_DOMAIN_ID=15
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  use_sim_time:=true
```

**중요**: frame_id가 자동으로 전송되므로 별도 설정 불필요!

### 터미널 5: 성능 확인 (선택사항)
```bash
export ROS_DOMAIN_ID=15

# 데이터 전송률 확인
ros2 topic hz /lidar/decompressed

# frame_id 확인
ros2 topic echo /lidar/decompressed --field header.frame_id --once

# 데이터 내용 확인
ros2 topic echo /lidar/decompressed --no-arr
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

### Draco 압축 알고리즘
- **압축 방식**: Google Draco (3D 포인트 클라우드 특화)
- **압축률**: 10:1 ~ 30:1 (일반 zlib 대비 3~5배 향상)
- **손실 여부**: 양자화로 인한 미세한 손실 (설정 가능)

### 주요 파라미터

#### 1. quantization_bits (양자화 비트)
- **기본값**: 11
- **범위**: 1-31
- **설명**: 좌표 정밀도 제어
  - **높은 값 (14-20)**: 높은 정밀도, 낮은 압축률
  - **중간 값 (10-13)**: 균형 (권장)
  - **낮은 값 (1-9)**: 낮은 정밀도, 높은 압축률

#### 2. compression_speed (압축 속도)
- **기본값**: 5
- **범위**: 0-10
- **설명**: 압축 속도 vs 압축률 트레이드오프
  - **0-3**: 느린 압축, 최고 압축률
  - **4-6**: 균형 (권장)
  - **7-10**: 빠른 압축, 낮은 압축률

### 파라미터 조정 예제

```bash
# 최고 압축률 (느림, SLAM 오프라인 처리용)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=14 \
  -p compression_speed:=0

# 균형 설정 (실시간 처리용, 기본값)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=11 \
  -p compression_speed:=5

# 고속 압축 (빠름, 실시간 최우선)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=9 \
  -p compression_speed:=10
```

### 전송 프로토콜

서버가 클라이언트에게 전송하는 정보:
1. **메인 헤더**: data_size, frame_id, timestamp, 메타데이터
2. **필드 정보**: PointCloud2 필드 구조
3. **Draco 압축 데이터**: 압축된 포인트 클라우드

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
# 일반 재생:
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock
# 무한 반복 재생:
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock --loop

# 서버 실행 (터미널 2)
ros2 run draco_bridge_cpp simple_draco_bridge
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
  libdraco-dev

# Draco 라이브러리 설치 확인
ldconfig -p | grep draco
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
- Fixed Frame이 bag 파일의 frame_id와 일치하는지 확인
  - frame_id는 자동으로 전송되므로 `ros2 topic echo /lidar/decompressed --field header.frame_id --once`로 확인
- QoS 설정이 호환되는지 확인

### 4. TF 오류 (KISS-ICP 사용 시)
```
[WARN] [tf2_buffer]: Detected jump back in time
```
**원인**: bag 파일이 반복 재생될 때 정상적인 경고입니다.

**해결방법**:
```bash
# --start-paused 옵션으로 시작
ros2 bag play <bag_파일> --clock --start-paused
# 모든 노드가 준비된 후 스페이스바로 재생 시작
```

### 5. Draco 라이브러리를 찾을 수 없음
```
CMake Error: Draco library not found
```
**해결방법**:
```bash
sudo apt install libdraco-dev
ldconfig -p | grep draco  # 설치 확인
```

## 환경 설정
- **ROS2**: Humble
- **도메인 ID**: 15
- **C++ 표준**: C++17
- **압축 라이브러리**: Google Draco
- **의존성**: libdraco-dev, rclcpp, sensor_msgs

## 파일 구조
```
src/draco_bridge_cpp/
├── src/
│   ├── simple_draco_bridge.cpp    # 서버 (Draco 압축 및 전송)
│   ├── simple_draco_client.cpp    # 클라이언트 (Draco 복호화)
│   ├── draco_bridge_server.cpp    # 구버전 서버 (참고용)
│   └── draco_bridge_client.cpp    # 구버전 클라이언트 (참고용)
├── CMakeLists.txt                  # Draco 라이브러리 링크 설정
└── package.xml

config/
└── simple_draco_visualization.rviz  # RVIZ2 설정

README_TCP_DRACO_BRIDGE.md           # 이 파일
```

## 성능 비교

### zlib vs Draco 압축률 비교

| 항목 | zlib (기존) | Draco (현재) | 개선율 |
|------|------------|--------------|--------|
| 압축률 | 2:1 ~ 4:1 | 10:1 ~ 30:1 | 3~5배 |
| 속도 | 빠름 | 중간 (설정 가능) | - |
| 특화 | 범용 | 3D 포인트 클라우드 | - |
| 손실 | 무손실 | 양자화 손실 (미세) | - |

### 실제 테스트 결과 (예상)
- **원본 크기**: ~2.5 MB/frame
- **Draco 압축 후**: ~100-250 KB/frame
- **압축률**: 10:1 ~ 25:1
- **처리 속도**: 10Hz (실시간)

## 추가 기능

### 1. KISS-ICP SLAM과 연동
클라이언트에서 복호화된 데이터를 KISS-ICP로 직접 전송:
```bash
# 클라이언트 실행 후
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  use_sim_time:=true
```

### 2. frame_id 자동 전송
- bag 파일의 원본 frame_id가 자동으로 보존됩니다
- 별도의 frame_id 설정 불필요
- TF 트리가 자동으로 올바르게 구성됩니다

### 3. 타임스탬프 보존
- 원본 타임스탬프가 그대로 전송됩니다
- `use_sim_time:=true`와 완벽하게 호환됩니다

## 종료 방법
```bash
# 각 터미널에서
Ctrl+C

# 또는 강제 종료
pkill -f simple_draco_bridge
pkill -f simple_draco_client
pkill -f kiss_icp
```

## 참고 자료
- **Google Draco**: https://github.com/google/draco
- **KISS-ICP**: https://github.com/PRBonn/kiss-icp
- **ROS2 Humble**: https://docs.ros.org/en/humble/
