cmake_minimum_required(VERSION 3.10.2)
project(point_cloud_transport_py)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Python dependencies
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
install(DIRECTORY python/ DESTINATION lib/python3.10/site-packages)

# Install examples
install(PROGRAMS
  examples/point_cloud_publisher.py
  examples/point_cloud_subscriber.py
  examples/point_cloud_codec_example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclpy
  sensor_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Python tests
  ament_add_pytest_test(test_point_cloud_transport_py
    test/test_point_cloud_transport.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()