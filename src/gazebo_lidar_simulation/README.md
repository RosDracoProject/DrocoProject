# Gazebo 3D LiDAR Simulation

이 패키지는 Gazebo를 사용한 3D LiDAR 시뮬레이션과 TCP Point Cloud Transport 통합을 제공합니다.

## 🚀 기능

- **3D LiDAR 시뮬레이션**: Gazebo에서 64-beam 3D LiDAR 센서 시뮬레이션
- **TCP Point Cloud Transport**: 압축된 포인트 클라우드 데이터의 TCP/IP 전송
- **실시간 처리**: ROS2 기반 실시간 포인트 클라우드 처리
- **압축 지원**: Zlib 및 Draco 압축을 통한 효율적인 데이터 전송

## 📦 의존성

### 필수 패키지
- `gazebo_ros`
- `gazebo_ros_pkgs`
- `gazebo_plugins`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`

### Point Cloud Transport 패키지
- `point_cloud_transport_py`
- `tcp_point_cloud_transport_cpp`

## 🛠️ 설치 및 빌드

```bash
# 워크스페이스로 이동
cd /home/hkit/my_data/final_project/point_cloud_transport

# 패키지 빌드
colcon build --packages-select gazebo_lidar_simulation

# 환경 설정
source install/setup.bash
```

## 🎮 사용법

### 1. 기본 시뮬레이션 실행

```bash
# Gazebo LiDAR 시뮬레이션 실행
ros2 launch gazebo_lidar_simulation gazebo_lidar.launch.py
```

### 2. TCP Transport 활성화

```bash
# TCP Transport와 함께 실행
ros2 launch gazebo_lidar_simulation gazebo_lidar.launch.py use_tcp_transport:=true tcp_server_port:=8080
```

### 3. 개별 컴포넌트 실행

```bash
# LiDAR Publisher만 실행
ros2 run gazebo_lidar_simulation lidar_publisher.py

# LiDAR Subscriber만 실행
ros2 run gazebo_lidar_simulation lidar_subscriber.py

# TCP Bridge만 실행
ros2 run gazebo_lidar_simulation tcp_lidar_bridge.py
```

## 📁 패키지 구조

```
gazebo_lidar_simulation/
├── worlds/                    # Gazebo 월드 파일
│   └── lidar_test_world.world
├── models/                    # 로봇 모델
│   └── lidar_robot/
│       └── lidar_robot.sdf
├── launch/                    # 런치 파일
│   └── gazebo_lidar.launch.py
├── scripts/                   # Python 스크립트
│   ├── lidar_publisher.py
│   ├── lidar_subscriber.py
│   └── tcp_lidar_bridge.py
├── config/                    # 설정 파일
│   └── lidar_config.yaml
├── package.xml
├── CMakeLists.txt
└── README.md
```

## ⚙️ 설정

### LiDAR 센서 설정
- **수평 샘플**: 720개 (0.5도 해상도)
- **수직 샘플**: 64개
- **측정 범위**: 0.1m ~ 30.0m
- **업데이트 주기**: 10Hz

### TCP Transport 설정
- **서버 포트**: 8080 (기본값)
- **최대 클라이언트**: 10개
- **압축**: Zlib (레벨 6)
- **전송 주기**: 10Hz

## 🔧 파라미터

### 런치 파일 파라미터
- `world_file`: Gazebo 월드 파일 경로
- `use_sim_time`: 시뮬레이션 시간 사용 여부
- `use_tcp_transport`: TCP Transport 사용 여부
- `tcp_server_port`: TCP 서버 포트

### 노드 파라미터
- `compression_enabled`: 압축 활성화
- `compression_type`: 압축 타입 (zlib, draco)
- `compression_level`: 압축 레벨 (1-9)

## 📊 모니터링

### 토픽
- `/scan`: 원시 LiDAR 데이터 (sensor_msgs/PointCloud2)
- `/lidar/points`: 처리된 포인트 클라우드 데이터

### 통계 정보
- 메시지 전송률
- 데이터 압축률
- TCP 연결 상태
- 처리 지연 시간

## 🧪 테스트

### 1. 기본 기능 테스트
```bash
# 포인트 클라우드 데이터 확인
ros2 topic echo /scan

# TCP 연결 테스트
telnet localhost 8080
```

### 2. 성능 테스트
```bash
# 데이터 전송률 측정
ros2 topic hz /scan

# 메모리 사용량 확인
ros2 run rqt_top rqt_top
```

## 🐛 문제 해결

### 일반적인 문제
1. **Gazebo가 시작되지 않음**: `gazebo_ros` 패키지가 설치되어 있는지 확인
2. **LiDAR 데이터가 없음**: 센서 플러그인이 올바르게 로드되었는지 확인
3. **TCP 연결 실패**: 방화벽 설정 및 포트 사용 여부 확인

### 로그 확인
```bash
# 상세 로그 출력
ros2 launch gazebo_lidar_simulation gazebo_lidar.launch.py --ros-args --log-level debug
```

## 📈 성능 최적화

### 압축 설정
- **Zlib 레벨 6**: 균형잡힌 압축률과 속도
- **Draco**: 더 높은 압축률 (설치 필요)

### 네트워크 최적화
- TCP 버퍼 크기 조정
- QoS 설정 최적화
- 멀티스레딩 활용

## 🤝 기여

버그 리포트나 기능 요청은 GitHub Issues를 통해 제출해주세요.

## 📄 라이선스

Apache-2.0 License

