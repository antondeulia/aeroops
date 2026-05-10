cmake_minimum_required(VERSION 3.8)
project(aeroops_px4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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

add_library(px4_tf_bridge_component SHARED
  src/px4_tf_bridge_node.cpp
)

target_include_directories(px4_tf_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_tf_bridge_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  px4_msgs
  tf2
  tf2_ros
)

rclcpp_components_register_node(px4_tf_bridge_component
  PLUGIN "aeroops_px4::Px4TfBridgeNode"
  EXECUTABLE px4_tf_bridge
)

add_library(camera_frame_relay_component SHARED
  src/camera_frame_relay_node.cpp
)

target_include_directories(camera_frame_relay_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_frame_relay_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

rclcpp_components_register_node(camera_frame_relay_component
  PLUGIN "aeroops_px4::CameraFrameRelayNode"
  EXECUTABLE camera_frame_relay
)

install(TARGETS
  offboard_control_component
  px4_tf_bridge_component
  camera_frame_relay_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
