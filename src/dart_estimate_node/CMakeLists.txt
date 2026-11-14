cmake_minimum_required(VERSION 3.8)
project(dart_estimate_node)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetEstimate.msg"
  DEPENDENCIES builtin_interfaces std_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(singer_filter src/singer_filter.cpp)
target_compile_features(singer_filter PUBLIC cxx_std_20)
target_compile_options(singer_filter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(singer_filter PUBLIC include)
target_link_libraries(singer_filter Eigen3::Eigen)

add_executable(dart_estimate_executable src/dart_estimate_node.cpp)
set_target_properties(dart_estimate_executable PROPERTIES OUTPUT_NAME dart_estimate_node)
target_compile_features(dart_estimate_executable PRIVATE cxx_std_20)
target_compile_options(dart_estimate_executable PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(dart_estimate_executable singer_filter "${typesupport}")
ament_target_dependencies(
  dart_estimate_executable dart_interfaces diagnostic_msgs rclcpp std_msgs)

install(TARGETS singer_filter dart_estimate_executable
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(singer_filter_test test/singer_filter_test.cpp)
  target_compile_features(singer_filter_test PRIVATE cxx_std_20)
  target_link_libraries(singer_filter_test singer_filter)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
