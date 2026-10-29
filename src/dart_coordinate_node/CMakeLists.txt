cmake_minimum_required(VERSION 3.8)
project(dart_coordinate_node)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mcu_node REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dart_coordinate_node src/dart_coordinate_node.cpp)
target_compile_features(dart_coordinate_node PRIVATE cxx_std_20)
target_compile_options(dart_coordinate_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(dart_coordinate_node PRIVATE include)
ament_target_dependencies(dart_coordinate_node dart_interfaces diagnostic_msgs mcu_node rclcpp)

install(TARGETS dart_coordinate_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(coordinate_math_test test/coordinate_math_test.cpp)
  target_compile_features(coordinate_math_test PRIVATE cxx_std_20)
  target_include_directories(coordinate_math_test PRIVATE include)
endif()

ament_package()
