cmake_minimum_required(VERSION 3.10.2)
project(gazebo_lidar_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(point_cloud_transport_py REQUIRED)
find_package(tcp_point_cloud_transport_cpp REQUIRED)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install robot models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(PROGRAMS
  scripts/lidar_publisher.py
  scripts/lidar_subscriber.py
  scripts/tcp_lidar_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_ros
  gazebo_ros_pkgs
  gazebo_plugins
  point_cloud_transport_py
  tcp_point_cloud_transport_cpp
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
