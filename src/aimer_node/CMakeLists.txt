cmake_minimum_required(VERSION 3.8)
project(aimer_node)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(aimer_node src/aimer_node.cpp)
target_compile_features(aimer_node PRIVATE cxx_std_20)
target_compile_options(aimer_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aimer_node PRIVATE include)
ament_target_dependencies(aimer_node dart_interfaces diagnostic_msgs rclcpp)

install(TARGETS aimer_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fin_allocator_test test/fin_allocator_test.cpp)
  target_include_directories(fin_allocator_test PRIVATE include)
endif()

ament_package()
