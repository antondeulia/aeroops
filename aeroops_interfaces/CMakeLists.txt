cmake_minimum_required(VERSION 3.8)
project(aeroops_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/MissionStatus.msg"
  "srv/ArmDisarm.srv"
  "srv/UploadMission.srv"
  "srv/StartMission.srv"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
