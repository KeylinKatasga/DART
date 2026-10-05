cmake_minimum_required(VERSION 3.8)
project(dart_estimate_node)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mcu_node REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetEstimate.msg"
  DEPENDENCIES builtin_interfaces std_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(dart_estimate_executable src/dart_estimate_node.cpp)
set_target_properties(dart_estimate_executable PROPERTIES OUTPUT_NAME dart_estimate_node)
target_compile_features(dart_estimate_executable PRIVATE cxx_std_20)
target_compile_options(dart_estimate_executable PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(dart_estimate_executable "${typesupport}")
ament_target_dependencies(
  dart_estimate_executable dart_interfaces diagnostic_msgs mcu_node rclcpp std_msgs)

install(TARGETS dart_estimate_executable RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
