cmake_minimum_required(VERSION 3.10.2)

project(tcp_point_cloud_transport_cpp)

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
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)

# Find Draco library (optional)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DRACO draco)
if(DRACO_FOUND)
  add_compile_definitions(HAVE_DRACO)
  message(STATUS "Draco library found - compression support enabled")
else()
  message(WARNING "Draco library not found - compression will be disabled")
endif()

# Include directories
include_directories(include)

# C++ Library
add_library(${PROJECT_NAME}
  src/tcp_point_cloud_message.cpp
  src/tcp_publisher_plugin.cpp
  src/tcp_subscriber_plugin.cpp
  src/draco_compression.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs
  pluginlib::pluginlib
  rcpputils::rcpputils)

if(DRACO_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${DRACO_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PUBLIC ${DRACO_INCLUDE_DIRS})
endif()

# Install C++ library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install plugin description
install(FILES tcp_plugins.xml DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  point_cloud_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  pluginlib
  rcpputils)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # C++ tests
  ament_add_gtest(${PROJECT_NAME}_test_tcp_message
    test/test_tcp_message.cpp)
  if(TARGET ${PROJECT_NAME}_test_tcp_message)
    target_link_libraries(${PROJECT_NAME}_test_tcp_message ${PROJECT_NAME})
  endif()
  
  ament_add_gtest(${PROJECT_NAME}_test_tcp_publisher
    test/test_tcp_publisher.cpp)
  if(TARGET ${PROJECT_NAME}_test_tcp_publisher)
    target_link_libraries(${PROJECT_NAME}_test_tcp_publisher ${PROJECT_NAME} rclcpp::rclcpp)
  endif()
endif()

ament_package()
