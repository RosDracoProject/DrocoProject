cmake_minimum_required(VERSION 3.10.2)

project(tcp_point_cloud_transport_py)

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
find_package(ament_cmake_python REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tcp_point_cloud_transport_cpp REQUIRED)

# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Python extension module (if needed for C++ bindings)
# pybind11_add_module(${PROJECT_NAME}_py
#   python/${PROJECT_NAME}/tcp_transport.cpp
# )

# target_link_libraries(${PROJECT_NAME}_py PRIVATE
#   tcp_point_cloud_transport_cpp::tcp_point_cloud_transport_cpp
#   point_cloud_transport::point_cloud_transport
#   rclpy::rclpy
#   sensor_msgs::sensor_msgs)

# Install Python module
# install(TARGETS ${PROJECT_NAME}_py
#   DESTINATION lib/python3.10/site-packages/${PROJECT_NAME})

# Install Python package
install(DIRECTORY python/ DESTINATION lib/python3.10/site-packages)

# Install examples
install(PROGRAMS
  examples/tcp_publisher_example.py
  examples/tcp_subscriber_example.py
  examples/tcp_client_example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  point_cloud_transport
  rclpy
  sensor_msgs
  tcp_point_cloud_transport_cpp)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Python tests
  ament_add_pytest_test(test_tcp_transport_py
    test/test_tcp_transport.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    
  ament_add_pytest_test(test_draco_compression_py
    test/test_draco_compression.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
