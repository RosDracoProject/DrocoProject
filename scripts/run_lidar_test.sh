#!/bin/bash

# 라이다 데이터 소스 선택 테스트 스크립트

echo "=== 라이다 데이터 소스 선택 ==="
echo "1. 시뮬레이션 데이터 (Gazebo)"
echo "2. Bag 파일 데이터 (실제 라이다 데이터)"
echo "3. 사용자 정의 토픽"
echo "4. 브리지와 클라이언트 동시 실행 (데이터 비교 테스트)"
echo ""

read -p "데이터 소스를 선택하세요 (1-4): " choice

case $choice in
    1)
        echo "시뮬레이션 데이터를 선택했습니다."
        echo "Gazebo 시뮬레이션을 먼저 실행하세요:"
        echo "  ros2 launch my_bot launch_sim.launch.py"
        echo ""
        echo "브리지 실행 중..."
        export ROS_DOMAIN_ID=10
        PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=simulation
        ;;
    2)
        echo "Bag 파일 데이터를 선택했습니다."
        echo "Bag 파일을 재생하세요:"
        echo "  ros2 bag play /home/hkit/my_data/lidar_data/rosbag2_2024_09_24-14_28_57/ --loop"
        echo ""
        echo "브리지 실행 중..."
        export ROS_DOMAIN_ID=10
        PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file
        ;;
    3)
        read -p "사용자 정의 토픽을 입력하세요: " custom_topic
        echo "사용자 정의 토픽을 선택했습니다: $custom_topic"
        echo "브리지 실행 중..."
        export ROS_DOMAIN_ID=10
        PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=real_sensor -p lidar_topic:=$custom_topic
        ;;
    4)
        echo "데이터 비교 테스트를 시작합니다."
        echo "이 옵션은 브리지와 클라이언트를 동시에 실행하여 데이터 품질을 비교합니다."
        echo ""
        echo "1단계: 브리지 실행 (백그라운드)"
        export ROS_DOMAIN_ID=10
        PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/lidar_draco_bridge_improved.py --ros-args -p data_source:=bag_file &
        BRIDGE_PID=$!
        echo "브리지 PID: $BRIDGE_PID"
        
        echo "2단계: 3초 대기 후 클라이언트 실행"
        sleep 3
        
        echo "3단계: 클라이언트 실행"
        PYTHONPATH=/home/hkit/my_data/final_project/ros2_ws/install/point_cloud_transport_py/lib/python3.10/site-packages:$PYTHONPATH python3 install/lidar_draco_bridge/lib/lidar_draco_bridge/draco_client_improved.py
        
        echo "4단계: 브리지 종료"
        kill $BRIDGE_PID 2>/dev/null
        echo "테스트 완료!"
        ;;
    *)
        echo "잘못된 선택입니다."
        exit 1
        ;;
esac
