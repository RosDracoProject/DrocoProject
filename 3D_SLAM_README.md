# KISS-ICP 3D SLAM 시스템

## 개요

**LiDAR 데이터만**으로 3D SLAM을 수행하는 시스템입니다.

### 핵심 특징
| 기능 | 설명 |
|------|------|
| 🎯 **간단함** | TF, Odometry, IMU 불필요 |
| 🗺️ **자동 맵핑** | 3D 포인트 클라우드 맵 자동 생성 |
| 📍 **위치 추정** | 로봇 움직임 자동 계산 |
| 🎨 **시각화** | Z축 높이별 컬러 맵핑 |

### 시스템 요구사항
| 항목 | 요구사항 |
|------|----------|
| **ROS2** | Humble |
| **SLAM** | KISS-ICP (빌드 완료) |
| **데이터** | PointCloud2 토픽 |

---

## 실행 방법

### 🚀 빠른 실행 (권장 - 범용)

**모든 bag 파일에서 자동으로 작동합니다!**

```bash
cd /home/hkit/my_data/final_project/ros2_ws
./scripts/auto_kiss_icp.sh /home/hkit/my_data/lidar_data/full_20251001_163123_0/
```

스크립트가 자동으로:
- ✅ bag 파일의 frame_id 감지
- ✅ 적절한 설정으로 KISS-ICP 시작
- ✅ RViz2 자동 실행

**다른 bag 파일 사용 시:**
```bash
./scripts/auto_kiss_icp.sh /path/to/your/bag/file/
```

---

### 📌 수동 실행 (고급 사용자용)

**터미널 1: Bag 파일 재생**
```bash
cd /home/hkit/my_data/final_project/ros2_ws
export ROS_DOMAIN_ID=15

# 일반 재생 (한 번만)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock

# 무한 반복 재생 (테스트용)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock --loop
```

> **⚠️ 주의**: `--loop` 사용 시 bag 파일의 시작점과 끝점이 다르면 맵이 왜곡될 수 있습니다.

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

> **⚠️ 중요**: bag 파일마다 frame_id가 다를 수 있습니다 (`rslidar_top` vs `lidar_top`).  
> RViz2에서 **Fixed Frame**을 해당 frame_id로 변경해야 `/kiss/frame`이 보입니다!

---

## KISS-ICP 출력

### 발행 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/kiss/local_map` | PointCloud2 | 3D 맵 (Z축 컬러) |
| `/kiss/frame` | PointCloud2 | 현재 스캔 |
| `/kiss/keypoints` | PointCloud2 | 특징점 |
| `/kiss/odometry` | Odometry | 위치 추정 |

### TF 트리
```
map → odom_lidar → rslidar_top
```

### RVIZ2 시각화
| 표시 | 색상 | 설명 |
|------|------|------|
| 포인트 클라우드 | 🌈 무지개 | 높이별 컬러 맵 |
| 현재 프레임 | 🔵 하늘색 | 실시간 스캔 |
| 로봇 위치 | 🔴 빨강 축 | Odometry |

### 주요 파라미터
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | - | 입력 토픽 |
| `base_frame` | rslidar_top | Base 프레임 |
| `use_sim_time` | true | Bag 시간 사용 |
| `max_range` | 100.0 | 최대 범위 (m) |

---

## 문제 해결

### 🔥 `/kiss/frame`이 안 보임 (가장 흔함)

**원인**: frame_id 불일치

**해결**:
```bash
# 1. 자동 스크립트 (권장)
./scripts/auto_kiss_icp.sh /path/to/bag/

# 2. 수동 - frame_id 확인
ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex --field header.frame_id --once

# 3. RVIZ2 Fixed Frame을 위에서 확인한 값으로 변경
```

### 토픽이 안 보임
```bash
export ROS_DOMAIN_ID=15
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

### 맵이 안 만들어짐
1. Bag 재생 확인: `ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex`
2. 5-10초 대기 (초기화 시간)

### `--loop` 사용 시 맵 왜곡
시작점 ≠ 끝점이면 맵 왜곡 → `--loop` 제거

---

## 작동 원리

### KISS-ICP 동작
```
입력: PointCloud2
  ↓
ICP 매칭 (연속 스캔)
  ↓
위치 추정 + 맵 생성
  ↓
출력: Odometry + 3D 맵
```

**핵심**: LiDAR 데이터만으로 연속 스캔을 비교해 움직임 추정!

---

## Draco 압축 + SLAM 통합

### 데이터 흐름
```
Bag → 서버(Draco 압축) → TCP → 클라이언트(Draco 복호화) → KISS-ICP → 3D 맵
```

### 전체 시스템 실행

```bash
# 터미널 1: Bag 재생
export ROS_DOMAIN_ID=15
# 일반 재생
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock
# 또는 무한 반복 (테스트용)
ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57 --clock --loop

# 터미널 2: Draco 서버 (압축)
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_bridge

# 터미널 3: Draco 클라이언트 (복호화)
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 run draco_bridge_cpp simple_draco_client

# 터미널 4: KISS-ICP SLAM
source install/setup.bash && export ROS_DOMAIN_ID=15
ros2 launch kiss_icp odometry.launch.py \
  topic:=/lidar/decompressed \
  visualize:=true \
  use_sim_time:=true
```

**자동 설정**: frame_id, timestamp 자동 전송! base_frame 설정 불필요!

### 성능 비교 테스트

| 토픽 | 설명 | 명령 |
|------|------|------|
| 원본 | 압축 없음 | `topic:=/sensing/lidar/top/pointcloud_raw_ex` |
| 복호화 | 압축 후 | `topic:=/lidar/decompressed` |

### Draco 압축 최적화

```bash
# 고속 (낮은 압축률, 빠른 처리)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=14 \
  -p compression_speed:=10

# 균형 (기본값, 권장)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=11 \
  -p compression_speed:=5

# 고압축 (높은 압축률, 느린 처리)
ros2 run draco_bridge_cpp simple_draco_bridge --ros-args \
  -p quantization_bits:=8 \
  -p compression_speed:=0
```

| 설정 | quantization_bits | speed | 압축률 | 속도 | 용도 |
|------|-------------------|-------|--------|------|------|
| **고속** | 14 (높음) | 10 (빠름) | ~15:1 | 빠름 | 로컬 네트워크 |
| **균형** | 11 (중간) | 5 (중간) | ~19:1 | 중간 | 일반 (권장) |
| **고압축** | 8 (낮음) | 0 (느림) | ~25:1+ | 느림 | 원격 네트워크 |

**핵심**: quantization_bits ↓ = 압축률 ↑, compression_speed ↓ = 압축률 ↑

### 성능 모니터링
```bash
# 데이터 레이트
ros2 topic hz /lidar/decompressed

# 압축률 (서버 로그 확인)
# 예: 압축률: 18.68:1 (2.3MB → 123KB)
```

---

## 참고 자료
- **KISS-ICP**: https://github.com/PRBonn/kiss-icp
- **Google Draco**: https://github.com/google/draco
- **Paper**: "KISS-ICP: In Defense of Point-to-Point ICP" (IEEE RA-L 2023)

---

