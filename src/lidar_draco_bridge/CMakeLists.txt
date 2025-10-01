cmake_minimum_required(VERSION 3.8)
project(lidar_draco_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(sensor_msgs_py REQUIRED)  # Not available in this environment
find_package(point_cloud_transport_py REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/lidar_draco_bridge.py
  scripts/draco_client.py
  scripts/lidar_draco_bridge_improved.py
  scripts/draco_client_improved.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
install(DIRECTORY lidar_draco_bridge
  DESTINATION lib/python3.10/site-packages/
)

# Test dependencies (disabled for now)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_lint_common REQUIRED)
#   find_package(ament_cmake_pytest REQUIRED)
#   
#   ament_lint_auto_find_test_dependencies()
#   
#   # Add pytest tests
#   ament_add_pytest_test(test_lidar_draco_bridge
#     src/lidar_draco_bridge/test/test_lidar_draco_bridge.py
#     TIMEOUT 60
#   )
# endif()

ament_package()
