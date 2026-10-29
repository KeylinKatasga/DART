cmake_minimum_required(VERSION 3.8)
project(dart_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GreenLight.msg"
  "msg/CoordinateObservation.msg"
  "msg/TargetState.msg"
  "msg/GuidanceCommand.msg"
  "msg/FinCommand.msg"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
