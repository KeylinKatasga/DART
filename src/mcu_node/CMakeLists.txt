cmake_minimum_required(VERSION 3.8)
project(mcu_node)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Rx.msg" "msg/Tx.msg" "msg/RawFrame.msg"
  DEPENDENCIES builtin_interfaces)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(gimbal_protocol src/protocol.cpp src/serial_port.cpp)
target_compile_features(gimbal_protocol PUBLIC cxx_std_20)
target_compile_options(gimbal_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(gimbal_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Interface generation owns the CMake target mcu_node.
add_executable(mcu_executable src/main.cpp src/gimbal.cpp)
set_target_properties(mcu_executable PROPERTIES OUTPUT_NAME mcu_node)
target_compile_features(mcu_executable PRIVATE cxx_std_20)
target_compile_options(mcu_executable PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mcu_executable gimbal_protocol "${typesupport}")
ament_target_dependencies(mcu_executable dart_interfaces diagnostic_msgs rclcpp)
install(TARGETS gimbal_protocol mcu_executable
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(protocol_test test/protocol_test.cpp)
  target_link_libraries(protocol_test gimbal_protocol)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
