cmake_minimum_required(VERSION 3.8)
project(dart_self_estimate_node)

find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mcu_node REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(self_ekf src/self_ekf.cpp)
target_compile_features(self_ekf PUBLIC cxx_std_20)
target_compile_options(self_ekf PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(self_ekf PUBLIC include)
target_link_libraries(self_ekf Ceres::ceres Eigen3::Eigen)

add_executable(dart_self_estimate_node src/dart_self_estimate_node.cpp)
target_compile_features(dart_self_estimate_node PRIVATE cxx_std_20)
target_compile_options(dart_self_estimate_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(dart_self_estimate_node self_ekf)
ament_target_dependencies(
  dart_self_estimate_node dart_interfaces diagnostic_msgs mcu_node rclcpp sensor_msgs)

install(TARGETS self_ekf dart_self_estimate_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(self_ekf_test test/self_ekf_test.cpp)
  target_compile_features(self_ekf_test PRIVATE cxx_std_20)
  target_link_libraries(self_ekf_test self_ekf)
endif()

ament_package()
