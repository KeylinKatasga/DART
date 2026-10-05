cmake_minimum_required(VERSION 3.8)
project(detect_node)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(detect_core src/detector.cpp)
target_compile_features(detect_core PUBLIC cxx_std_20)
target_compile_options(detect_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(detect_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detect_core ${OpenCV_LIBS})

add_executable(detect_node src/detect_node.cpp)
target_compile_features(detect_node PRIVATE cxx_std_20)
target_compile_options(detect_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(detect_node detect_core ${OpenCV_LIBS})
ament_target_dependencies(detect_node
  dart_interfaces diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS detect_core detect_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(detector_test test/detector_test.cpp)
  target_link_libraries(detector_test detect_core ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_package()

