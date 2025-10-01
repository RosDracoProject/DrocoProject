cmake_minimum_required(VERSION 3.8)
project(draco_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executables
add_executable(simple_draco_bridge src/simple_draco_bridge.cpp)
add_executable(simple_draco_client src/simple_draco_client.cpp)

# Link libraries
ament_target_dependencies(simple_draco_bridge rclcpp sensor_msgs std_msgs)
ament_target_dependencies(simple_draco_client rclcpp sensor_msgs std_msgs)

target_link_libraries(simple_draco_bridge z)
target_link_libraries(simple_draco_client z)

# Install
install(TARGETS
  simple_draco_bridge
  simple_draco_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
