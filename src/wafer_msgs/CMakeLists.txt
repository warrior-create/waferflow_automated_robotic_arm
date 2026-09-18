cmake_minimum_required(VERSION 3.8)
project(wafer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WaferPose.msg"
  "msg/PlacementResult.msg"
  "msg/CycleMetrics.msg"
  "srv/AlignWafer.srv"
  "action/GenerateTrajectory.action"
  DEPENDENCIES builtin_interfaces trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
