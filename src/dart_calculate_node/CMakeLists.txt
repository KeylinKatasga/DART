cmake_minimum_required(VERSION 3.8)
project(dart_calculate_node)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dart_calculate_node src/dart_calculate_node.cpp)
target_compile_features(dart_calculate_node PRIVATE cxx_std_20)
target_compile_options(dart_calculate_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(dart_calculate_node PRIVATE include)
ament_target_dependencies(dart_calculate_node dart_interfaces diagnostic_msgs rclcpp)

install(TARGETS dart_calculate_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fin_allocator_test test/fin_allocator_test.cpp)
  target_compile_features(fin_allocator_test PRIVATE cxx_std_20)
  target_include_directories(fin_allocator_test PRIVATE include)
  ament_add_gtest(attitude_adjuster_test test/attitude_adjuster_test.cpp)
  target_compile_features(attitude_adjuster_test PRIVATE cxx_std_20)
  target_include_directories(attitude_adjuster_test PRIVATE include)
endif()

ament_package()
