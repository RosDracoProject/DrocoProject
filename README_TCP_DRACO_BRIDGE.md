# TCP/IP Draco Bridge - LiDAR 포인트 클라우드 전송 시스템

## 개요

ROS2 LiDAR 데이터를 **Google Draco 압축**으로 네트워크를 통해 전송하는 시스템입니다.

### 핵심 기능
| 기능 | 설명 |
|------|------|
| 🗜️ **Draco 압축** | 3D 포인트 클라우드 특화 압축 알고리즘 |
| 📡 **TCP/IP 전송** | 안정적인 네트워크 통신 |
| 🔖 **메타데이터 보존** | frame_id, timestamp 자동 전송 |
| ⚡ **실시간 처리** | 10Hz LiDAR 데이터 지원 |
| 🤝 **SLAM 호환** | KISS-ICP 바로 연동 가능 |

### 실측 성능
```
원본:  2.3 MB/frame (144,000 points)
  ↓
압축:  123 KB/frame
  ↓
압축률: 18.68:1 (대역폭 94.6% 절감)
```

### 시스템 구조
```
Bag 파일 → 서버(Draco 압축) → TCP/IP → 클라이언트(Draco 복호화) → KISS-ICP/RVIZ2
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
  -p server_host:=192.168.3.22 \
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

### 터미널 5: 웹 모니터 (실시간 성능 시각화) 🌐
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_web_monitor web_monitor_node
```

**웹 브라우저에서 접속:**
```
http://localhost:5000
```

**실시간 표시:**
- 📊 압축률 그래프 (18.68:1)
- ⏱️ 복호화 시간 (ms)
- 📡 네트워크 대역폭 (KB/s)
- 📦 복호화 후 데이터 크기 (bytes)
- 💻 CPU/메모리 사용률

### 터미널 6: 성능 확인 (선택사항)
```bash
export ROS_DOMAIN_ID=15

# 데이터 전송률 확인
ros2 topic hz /lidar/decompressed

# 통계 토픽 확인
ros2 topic echo /draco/compression_ratio
ros2 topic echo /draco/decompression_time
ros2 topic echo /draco/network_throughput
ros2 topic echo /draco/decompressed_size

# frame_id 확인
ros2 topic echo /lidar/decompressed --field header.frame_id --once
```

## 시스템 설정

### ROS2 토픽

#### 데이터 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/sensing/lidar/top/pointcloud_raw_ex` | PointCloud2 | 입력 (원본) |
| `/lidar/compressed` | PointCloud2 | 서버 출력 (압축됨) |
| `/lidar/decompressed` | PointCloud2 | 클라이언트 출력 (복호화됨) |

#### 웹 모니터링 토픽 (클라이언트가 발행)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/draco/compression_ratio` | Float64 | 압축률 (예: 18.68) |
| `/draco/decompression_time` | Float64 | 복호화 시간 (ms) |
| `/draco/network_throughput` | Float64 | 네트워크 대역폭 (KB/s) |
| `/draco/decompressed_size` | Float64 | 복호화 후 데이터 크기 (bytes) |

### 네트워크
| 항목 | 값 |
|------|-----|
| **프로토콜** | TCP/IP |
| **포트** | 8888 |
| **로컬 주소** | 127.0.0.1 |
| **원격 주소** | 서버 IP (예: 192.168.3.251) |

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
  - **높은 값 (14-20)**: 높은 정밀도, **낮은 압축률** ← 빠른 처리
  - **중간 값 (10-13)**: 균형 (권장)
  - **낮은 값 (6-9)**: 낮은 정밀도, **높은 압축률** ← 고압축

**트레이드오프**: 정밀도 vs 압축률 (값이 낮을수록 압축률 ↑, 정밀도 ↓)

#### 2. compression_speed (압축 속도)
- **기본값**: 5
- **범위**: 0-9 ⚠️ (10은 오류 발생)
- **설명**: 압축 속도 vs 압축률 트레이드오프
  - **0-3**: 느린 압축, 최고 압축률
  - **4-6**: 균형 (권장)
  - **7-9**: 빠른 압축, 낮은 압축률

### 파라미터 조정 예제

```bash
# 고속 (낮은 압축률, 빠른 처리)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=14 \
  -p compression_speed:=9

# 균형 설정 (실시간 처리용, 기본값)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=11 \
  -p compression_speed:=5

# 고압축 (높은 압축률, 느린 처리)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=8 \
  -p compression_speed:=0
```

**중요**: 
- `quantization_bits` ↓ = 압축률 ↑ (정밀도 ↓)
- `compression_speed` 최대값은 **9** (10은 오류!)

### 커스텀 전송 프로토콜

우리가 설계한 TCP 프로토콜 구조:
```
┌─────────────────────────────────┐
│ 1. 메인 헤더                    │  ← 우리가 설계
│    - data_size, frame_id        │
│    - timestamp (sec, nanosec)   │
│    - height, width, step 등     │
├─────────────────────────────────┤
│ 2. 필드 정보 (PointCloud2)     │  ← 우리가 설계
│    - 필드 개수, 이름, 타입      │
├─────────────────────────────────┤
│ 3. Draco 압축 바이너리          │  ← Draco가 생성
│    - 압축된 x, y, z, intensity  │
└─────────────────────────────────┘
```

**중요**: Draco는 순수 압축만 담당하며, ROS 메타데이터 전송은 우리가 구현했습니다!

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

## 기술 스택

| 항목 | 버전/설정 |
|------|-----------|
| **ROS2** | Humble |
| **도메인 ID** | 15 |
| **C++ 표준** | C++17 |
| **압축 라이브러리** | Google Draco |
| **주요 의존성** | libdraco-dev, rclcpp, sensor_msgs |

## 파일 구조
```
src/
├── draco_bridge_cpp/
│   ├── src/
│   │   ├── simple_draco_bridge.cpp    # 서버 (Draco 압축 + TCP 전송)
│   │   ├── simple_draco_client.cpp    # 클라이언트 (TCP 수신 + Draco 복호화 + 통계 발행)
│   │   ├── draco_bridge_server.cpp    # 참고용
│   │   └── draco_bridge_client.cpp    # 참고용
│   ├── CMakeLists.txt
│   └── package.xml
│
└── draco_web_monitor/               # 🌐 웹 모니터링
    ├── draco_web_monitor/
    │   ├── web_monitor_node.py      # ROS2 노드
    │   ├── web_server.py            # Flask 서버
    │   └── templates/
    │       └── index.html            # 대시보드 UI
    ├── setup.py
    └── package.xml

config/
└── simple_draco_visualization.rviz

README_TCP_DRACO_BRIDGE.md           # 이 파일
```

## 성능 벤치마크

### 실측 데이터 (144,000 포인트/프레임)

| 항목 | 값 |
|------|-----|
| **원본 크기** | 2.30 MB |
| **압축 후** | 123 KB |
| **압축률** | **18.68:1** |
| **처리 속도** | 10 Hz (실시간) |
| **대역폭 절감** | 94.6% |

### 압축 알고리즘 비교

| 알고리즘 | 압축률 | 속도 | 특화 분야 |
|----------|--------|------|-----------|
| **zlib** | 2:1 ~ 4:1 | 빠름 | 범용 |
| **Draco** | 10:1 ~ 30:1 | 중간 | 3D 포인트 클라우드 |

**결론**: Draco는 zlib 대비 **3~5배 높은 압축률**

## 웹 모니터링 시스템 🌐

### 실시간 성능 대시보드

웹 브라우저에서 시스템 성능을 실시간으로 모니터링할 수 있습니다!

#### 실행 방법
```bash
# 클라이언트 실행 후 (터미널 3)
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_web_monitor web_monitor_node
```

#### 웹 접속
```
http://localhost:5000
```

#### 표시 항목
| 카테고리 | 지표 | 설명 |
|----------|------|------|
| **압축 성능** | 압축률 그래프 | 실시간 18.68:1 |
| **처리 시간** | 복호화 시간 | 밀리초 단위 |
| **네트워크** | 대역폭 사용량 | KB/s |
| **데이터 크기** | 복호화 후 크기 | bytes |
| **시스템** | CPU/메모리 | 사용률 % |

#### 스크린샷 예시
```
┌────────────────────────────────────────┐
│ 압축률: 18.68:1  │ 복호화: 5.2ms     │
│ CPU: 15.3%       │ 메모리: 23.1%     │
├────────────────────────────────────────┤
│  [압축률 그래프]     [시간 그래프]    │
│                                        │
│  [시스템 그래프]     [크기 그래프]    │
└────────────────────────────────────────┘
```

#### 데이터 흐름
```
클라이언트 (simple_draco_client)
    ↓ 계산 및 발행
ROS2 토픽:
    - /draco/compression_ratio (18.68)
    - /draco/decompression_time (5.2)
    - /draco/network_throughput (1234.5)
    - /draco/decompressed_size (2304000)
    ↓ 구독
웹 모니터 (web_monitor_node)
    ↓ Flask API
웹 브라우저
    ↓ Chart.js
실시간 그래프 표시
```

---

## 주요 기능

### ✅ 메타데이터 자동 보존
```
원본 bag 파일 → 서버 → 클라이언트
    ├─ frame_id ────────────────→ 자동 전송 ────→ TF 자동 구성
    ├─ timestamp ───────────────→ 자동 전송 ────→ use_sim_time 호환  
    └─ fields ──────────────────→ 자동 전송 ────→ 구조 보존
```

**장점**: 
- ✅ frame_id 수동 설정 불필요
- ✅ TF 트리 자동 구성
- ✅ SLAM 바로 연동 가능

### 🔗 KISS-ICP 직접 연동
```bash
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  use_sim_time:=true
```
별도 설정 없이 바로 SLAM 실행!

## 종료 방법
```bash
# 각 터미널에서
Ctrl+C

# 또는 강제 종료
pkill -f simple_draco_bridge
pkill -f simple_draco_client
pkill -f web_monitor_node
pkill -f kiss_icp
```

## 전체 시스템 실행 순서 요약

```bash
# 1단계: Bag 재생 (터미널 1)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock --loop

# 2단계: Draco 서버 (터미널 2)
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_bridge

# 3단계: Draco 클라이언트 (터미널 3)
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client

# 4단계: KISS-ICP (터미널 4)
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 launch kiss_icp odometry.launch.py topic:=/lidar/decompressed visualize:=true use_sim_time:=true

# 5단계: 웹 모니터 (터미널 5) 🌐
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 run draco_web_monitor web_monitor_node

# 6단계: 웹 브라우저에서 접속
# http://localhost:5000
```

## 참고 자료
- **Google Draco**: https://github.com/google/draco
- **KISS-ICP**: https://github.com/PRBonn/kiss-icp
- **ROS2 Humble**: https://docs.ros.org/en/humble/
