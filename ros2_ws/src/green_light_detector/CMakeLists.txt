cmake_minimum_required(VERSION 3.8)
project(green_light_detector)

find_package(ament_cmake REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(green_light_core src/detector.cpp)
target_compile_features(green_light_core PUBLIC cxx_std_20)
target_compile_options(green_light_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(green_light_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(green_light_core ${OpenCV_LIBS})

add_executable(green_light_node src/green_light_node.cpp)
target_compile_features(green_light_node PRIVATE cxx_std_20)
target_compile_options(green_light_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(green_light_node green_light_core ${OpenCV_LIBS})
ament_target_dependencies(green_light_node
  dart_interfaces diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS green_light_core green_light_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(detector_test test/detector_test.cpp)
  target_link_libraries(detector_test green_light_core ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_package()
