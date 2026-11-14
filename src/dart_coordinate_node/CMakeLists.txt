cmake_minimum_required(VERSION 3.8)
project(dart_coordinate_node)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mcu_node REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(coordinate_core src/circle_pose.cpp)
target_compile_features(coordinate_core PUBLIC cxx_std_20)
target_compile_options(coordinate_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(coordinate_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(coordinate_core ${OpenCV_LIBS})

add_executable(dart_coordinate_node src/dart_coordinate_node.cpp)
target_compile_features(dart_coordinate_node PRIVATE cxx_std_20)
target_compile_options(dart_coordinate_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(dart_coordinate_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dart_coordinate_node coordinate_core)
ament_target_dependencies(dart_coordinate_node dart_interfaces diagnostic_msgs mcu_node rclcpp sensor_msgs)

install(TARGETS coordinate_core dart_coordinate_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(coordinate_math_test test/coordinate_math_test.cpp)
  target_compile_features(coordinate_math_test PRIVATE cxx_std_20)
  target_include_directories(coordinate_math_test PRIVATE include)
  ament_add_gtest(circle_pose_test test/circle_pose_test.cpp)
  target_compile_features(circle_pose_test PRIVATE cxx_std_20)
  target_link_libraries(circle_pose_test coordinate_core)
endif()

ament_package()
