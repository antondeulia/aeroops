cmake_minimum_required(VERSION 3.8)
project(aeroops_px4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(aeroops_interfaces REQUIRED)

add_library(offboard_control_component SHARED
  src/offboard_control_node.cpp
)

target_include_directories(offboard_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_control_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  px4_msgs
  std_srvs
  aeroops_interfaces
)

rclcpp_components_register_nodes(
  offboard_control_component
  "aeroops_px4::OffboardControlNode"
)

install(TARGETS
  offboard_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()