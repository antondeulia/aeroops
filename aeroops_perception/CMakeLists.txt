cmake_minimum_required(VERSION 3.8)
project(aeroops_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(camera_monitor_component SHARED
  src/camera_monitor_node.cpp
)

target_include_directories(camera_monitor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(camera_monitor_component
  ${OpenCV_LIBS}
)

ament_target_dependencies(camera_monitor_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  cv_bridge
  std_msgs
  std_srvs
)

rclcpp_components_register_nodes(
  camera_monitor_component
  "aeroops::perception::CameraMonitorNode"
)

install(TARGETS
  camera_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
