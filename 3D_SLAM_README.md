# 3D LiDAR SLAM 시스템 사용 가이드

## 개요

KISS-ICP를 사용하여 3D LiDAR 데이터만으로 SLAM (Simultaneous Localization and Mapping)을 수행하는 시스템입니다.

**특징:**
- ✅ 3D PointCloud 데이터만 필요 (TF, Odometry, IMU 불필요)
- ✅ 자동으로 로봇의 움직임 추정 및 맵 생성
- ✅ 실시간 RVIZ2 시각화
- ✅ Z축 높이에 따른 컬러 맵핑

---

## 시스템 구성

### 필수 패키지
- ROS2 Humble
- KISS-ICP (이미 빌드됨)
- CMake 4.1.2+

### 데이터
- Bag 파일: `/home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57`
- 토픽: `/sensing/lidar/top/pointcloud_raw_ex` (sensor_msgs/PointCloud2)

---

## 실행 방법

### 📌 기본 실행

**터미널 1: Bag 파일 재생**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock
```

**터미널 2: KISS-ICP SLAM + RVIZ2**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=15

ros2 launch kiss_icp odometry.launch.py \
  topic:=/sensing/lidar/top/pointcloud_raw_ex \
  visualize:=true \
  base_frame:=rslidar_top \
  publish_debug_clouds:=true \
  use_sim_time:=true
```

### 🔁 무한 반복 재생 (테스트용)

**주의**: bag 파일의 시작점과 끝점이 다르면 맵이 왜곡될 수 있습니다.

```bash
# 터미널 1
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --loop --clock
```

---

## 토픽 변경 방법

### 다른 토픽으로 SLAM 수행

```bash
# 예시 1: 다른 PointCloud 토픽
ros2 launch kiss_icp odometry.launch.py \
  topic:=/sensing/lidar/top/pointcloud \
  visualize:=true \
  base_frame:=rslidar_top \
  publish_debug_clouds:=true \
  use_sim_time:=true

# 예시 2: 압축 해제된 데이터
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  base_frame:=rslidar_top \
  publish_debug_clouds:=true \
  use_sim_time:=true
```

### 빠른 토픽 변경 테스트

1. **Ctrl+C**로 SLAM 노드 종료
2. **토픽 이름만 변경**하여 재실행
3. 결과 비교

---

## 출력 토픽

KISS-ICP가 발행하는 토픽들:

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/kiss/local_map` | PointCloud2 | 3D 포인트 클라우드 맵 (Z축 컬러) |
| `/kiss/frame` | PointCloud2 | 현재 스캔 프레임 |
| `/kiss/keypoints` | PointCloud2 | 특징점 |
| `/kiss/odometry` | Odometry | 로봇 위치 추정 |

### TF 구조
```
map → odom_lidar → rslidar_top
      (KISS-ICP)
```

---

## RVIZ2 시각화

### 자동으로 표시되는 것들:
- **회색 포인트 (무지개 컬러)**: `/kiss/local_map` - 3D SLAM 맵
  - 빨강: 낮은 높이
  - 노랑: 중간 높이
  - 초록: 높은 높이
  - 파랑: 매우 높은 높이
- **하늘색 포인트**: `/kiss/frame` - 현재 프레임
- **빨간색 축**: 로봇 위치 및 방향

### Fixed Frame
- **odom_lidar**: KISS-ICP의 odometry 프레임
- **map**: 글로벌 맵 프레임 (정적 TF 필요)

---

## 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | - | 구독할 PointCloud2 토픽 이름 |
| `visualize` | true | RVIZ2 시각화 활성화 |
| `base_frame` | rslidar_top | 로봇의 base frame |
| `publish_debug_clouds` | true | 디버그 포인트 클라우드 발행 |
| `use_sim_time` | true | bag 파일의 시간 사용 |
| `max_range` | 100.0 | 최대 LiDAR 범위 (m) |
| `min_range` | 0.5 | 최소 LiDAR 범위 (m) |

---

## 트러블슈팅

### 1. "토픽이 안 보여요"
```bash
# ROS2 daemon 재시작
export ROS_DOMAIN_ID=15
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### 2. "맵이 안 만들어져요"
- bag 파일이 제대로 재생되고 있는지 확인
- `/sensing/lidar/top/pointcloud_raw_ex` 토픽에 데이터가 있는지 확인:
```bash
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex
```

### 3. "맵이 반복할 때마다 이상해져요"
- bag 파일의 시작점과 끝점이 다르기 때문입니다
- `--loop` 옵션을 제거하고 한 번만 재생하세요:
```bash
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock
```

### 4. "bash 환경 변수 오류"
```bash
# 오류 무시하고 계속 진행
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
```

---

## 작동 원리

### KISS-ICP (Keep It Simple, Stupid - ICP)

1. **입력**: 3D PointCloud (`sensor_msgs/PointCloud2`)
2. **처리**:
   - 연속된 스캔을 ICP (Iterative Closest Point)로 매칭
   - 로봇의 움직임(odometry) 자동 추정
   - Voxel 기반 3D 맵 생성
3. **출력**:
   - Odometry (위치 추정)
   - 3D 포인트 클라우드 맵
   - TF 변환

### 왜 TF 없이도 작동하나?

SLAM 알고리즘이 **연속된 포인트 클라우드를 비교**하여:
- 이전 프레임과 현재 프레임의 차이 계산
- 로봇이 얼마나 움직였는지 자동 추정
- 움직임을 누적하여 맵 생성

**LiDAR 데이터만 있으면 됩니다!**

---

## 참고 자료

- KISS-ICP GitHub: https://github.com/PRBonn/kiss-icp
- KISS-SLAM GitHub: https://github.com/PRBonn/kiss-slam
- Paper: "KISS-ICP: In Defense of Point-to-Point ICP" (IEEE RA-L 2023)

---

## Draco 압축/복호화 연동

### 시나리오: 압축 전송 후 SLAM

Draco 압축을 통해 데이터를 TCP로 전송하고, 복호화된 데이터로 SLAM을 수행합니다.

#### 데이터 흐름
```
Bag 파일 → Draco Bridge (압축) → TCP/IP → Draco Client (복호화) → KISS-ICP (SLAM)
   ↓              ↓                             ↓                        ↓
원본 토픽    /lidar/compressed            /lidar/decompressed      3D 맵 생성
```

#### 실행 방법

**터미널 1: Bag 파일 재생**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --loop --clock
```

**터미널 2: Draco Bridge (압축 서버)**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_bridge
```
- 입력: `/sensing/lidar/top/pointcloud_raw_ex`
- 출력: `/lidar/compressed` + TCP 전송 (port 8888)

**터미널 3: Draco Client (복호화 클라이언트)**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client
```
- 입력: TCP 수신 (localhost:8888)
- 출력: `/lidar/decompressed`

**터미널 4: KISS-ICP SLAM**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=15

ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  base_frame:=rslidar_top \
  publish_debug_clouds:=true \
  use_sim_time:=true
```

#### 비교 테스트

**원본 데이터로 SLAM**:
```bash
ros2 launch kiss_icp odometry.launch.py \
  topic:=/sensing/lidar/top/pointcloud_raw_ex \
  visualize:=true
```

**압축 해제된 데이터로 SLAM**:
```bash
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true
```

두 결과를 비교하여 **압축/복호화가 SLAM 성능에 미치는 영향**을 평가할 수 있습니다!

#### 성능 확인

**데이터 레이트 확인**:
```bash
# 원본
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex

# 압축
ros2 topic hz /lidar/compressed

# 복호화
ros2 topic hz /lidar/decompressed
```

**압축률 확인**:
Draco Bridge 로그에서 압축률 통계 확인

---

## 다음 단계

### 맵 저장
생성된 맵을 PCD 파일로 저장하려면:
```bash
ros2 topic echo /kiss/local_map --once > map.txt
# 또는 rosbag으로 기록
ros2 bag record /kiss/local_map /kiss/odometry
```

### 다른 bag 파일 사용
```bash
# bag 파일 경로만 변경
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_30_22 --clock
```

---

**작성일**: 2025-10-01  
**작성자**: KISS-ICP 3D SLAM 시스템

