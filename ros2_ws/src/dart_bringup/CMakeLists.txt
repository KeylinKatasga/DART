cmake_minimum_required(VERSION 3.8)
project(dart_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/"
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
