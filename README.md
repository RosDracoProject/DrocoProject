# Draco Point Cloud Transport for ROS2

ROS2 환경에서 LiDAR 포인트 클라우드 데이터를 효율적으로 압축하고 TCP/IP 네트워크를 통해 전송하는 통합 시스템입니다.

## 🚀 주요 기능

- **Draco 압축**: Google의 Draco 라이브러리를 사용한 고효율 포인트 클라우드 압축
- **TCP/IP 네트워크 전송**: 압축된 데이터를 TCP/IP를 통해 안정적으로 전송
- **Point Cloud Transport 플러그인**: ROS2 `image_transport`와 유사한 플러그인 아키텍처
- **LiDAR 시뮬레이션**: Gazebo/Ignition 기반 Velodyne LiDAR 시뮬레이션 환경
- **실시간 시각화**: RViz2를 통한 압축/복원된 데이터 실시간 모니터링

## 📦 패키지 구조

```
ros2_ws/
├── src/                               # 활성 패키지 (C++ 기반, 고성능)
│   ├── draco_bridge_cpp/              # Draco 압축 브리지 (메인)
│   ├── point_cloud_transport/         # Point Cloud Transport 코어 라이브러리
│   ├── tcp_point_cloud_transport_cpp/ # TCP 전송 플러그인
│   ├── tf_mover/                      # TF 변환 유틸리티
│   ├── mobile-3d-lidar-sim/           # 모바일 3D LiDAR 시뮬레이션 (서브모듈)
│   └── velodyne_simulator/            # Velodyne 시뮬레이터 (서브모듈)
├── archive/                           # 미사용 패키지 (참고용)
│   ├── lidar_draco_bridge/            # Python Draco 브리지 (느림)
│   ├── tcp_point_cloud_transport_py/  # Python TCP 플러그인
│   ├── point_cloud_transport_py/      # Python 바인딩
│   └── gazebo_lidar_simulation/       # Gazebo 시뮬레이션
├── config/                            # RViz 설정 파일
├── scripts/                           # 실행 스크립트 모음
└── README.md                          # 이 파일
```

> **Note**: 성능 최적화를 위해 C++ 구현만 사용합니다. Python 패키지와 시뮬레이션 도구는 `archive/`로 이동되었습니다.

## 🔧 시스템 요구사항

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS2**: Humble Hawksbill
- **CMake**: 3.8 이상
- **C++ 컴파일러**: C++17 지원

### 의존성 패키지

```bash
sudo apt update
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-pcl-ros \
  libdraco-dev \
  zlib1g-dev
```

## 🏗️ 빌드 방법

```bash
# 워크스페이스로 이동
cd /home/hkit/my_data/final_project/ros2_ws

# 서브모듈 초기화 (처음 한 번만)
git submodule update --init --recursive

# 빌드
colcon build --symlink-install

# 환경 설정
source install/setup.bash
export ROS_DOMAIN_ID=15
```

## 🎯 빠른 시작

### 1. TCP Draco Bridge 테스트

**터미널 1: 서버 실행**
```bash
./scripts/run_tcp_test.sh
```

**터미널 2: 클라이언트 실행**
```bash
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client
```

**터미널 3: RViz2 시각화 (선택)**
```bash
export ROS_DOMAIN_ID=15
rviz2 -d config/simple_draco_visualization.rviz
```

### 2. Gazebo LiDAR 시뮬레이션

```bash
./scripts/run_lidar_test.sh
```

### 3. 통합 시뮬레이션 (Gazebo + Draco Bridge)

```bash
./scripts/run_integrated_bridge.sh
```

## 📊 성능 모니터링

### 토픽 Hz 확인
```bash
# 원본 데이터
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex

# 압축된 데이터
ros2 topic hz /lidar/compressed

# 복원된 데이터
ros2 topic hz /lidar/decompressed
```

### 압축률 통계
서버 콘솔에서 실시간으로 압축 통계를 확인할 수 있습니다:
- 원본 크기 vs 압축 크기
- 압축률 (%)
- 처리 메시지 수

## 🔌 주요 토픽

| 토픽 이름 | 타입 | 설명 |
|----------|------|------|
| `/sensing/lidar/top/pointcloud_raw_ex` | `sensor_msgs/PointCloud2` | 원본 LiDAR 데이터 |
| `/lidar/compressed` | `std_msgs/ByteMultiArray` | 압축된 데이터 |
| `/lidar/decompressed` | `sensor_msgs/PointCloud2` | 복원된 데이터 |

## 🛠️ 주요 스크립트

모든 스크립트는 `scripts/` 디렉토리에 위치합니다:

- `run_tcp_test.sh`: TCP Draco Bridge 기본 테스트
- `run_lidar_test.sh`: LiDAR 시뮬레이션 실행
- `run_integrated_bridge.sh`: 통합 시뮬레이션
- `run_with_rviz.sh`: RViz2와 함께 실행
- `quick_test.sh`: 빠른 동작 테스트

## 🐛 문제 해결

### 클라이언트 연결 실패
```bash
# 서버가 실행 중인지 확인
ps aux | grep simple_draco_bridge
```

### ROS_DOMAIN_ID 불일치
모든 터미널에서 동일한 `ROS_DOMAIN_ID`를 사용해야 합니다:
```bash
export ROS_DOMAIN_ID=15
```

### 포인트 클라우드가 RViz에 표시되지 않음
- Fixed Frame이 올바른지 확인 (`rslidar_top` 또는 `velodyne`)
- 토픽 이름이 정확한지 확인
- QoS 설정 확인 (Best Effort vs Reliable)

## 📚 추가 문서

- [TCP/IP Draco Bridge 상세 가이드](README_TCP_DRACO_BRIDGE.md)
- [Gazebo LiDAR 시뮬레이션](src/gazebo_lidar_simulation/README.md)
- [Point Cloud Transport](src/point_cloud_transport/doc/index.rst)

## 🤝 기여

이슈나 개선사항이 있으시면 GitHub Issues에 등록해주세요.

## 📄 라이선스

각 패키지는 개별 라이선스를 따릅니다. 자세한 내용은 각 패키지의 `LICENSE` 파일을 참조하세요.

## 🔗 참고 자료

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Google Draco](https://github.com/google/draco)
- [Point Cloud Transport](https://github.com/ros-perception/point_cloud_transport)

## 👥 개발자

RosDracoProject Team

