cmake_minimum_required(VERSION 3.8)
project(aeroops_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aeroops_interfaces REQUIRED)

# 1: Add Library
add_library(mission_manager_component SHARED
  src/mission_manager.cpp
)

target_include_directories(mission_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_manager_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  std_msgs
  aeroops_interfaces
)

rclcpp_components_register_node(mission_manager_component
  PLUGIN "aeroops_core::MissionManager"
  EXECUTABLE mission_manager
)

install(
  DIRECTORY include/
  DESTINATION include
)

foreach(dir launch config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(
  TARGETS mission_manager_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  std_msgs
  aeroops_interfaces
)

ament_package()
