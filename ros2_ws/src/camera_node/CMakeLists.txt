cmake_minimum_required(VERSION 3.8)
project(camera_node)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(GALAXY_SDK_ROOT "$ENV{GALAXY_SDK_ROOT}" CACHE PATH
  "Galaxy SDK root containing include/ and lib/")
find_path(GALAXY_SDK_INCLUDE_DIR GxIAPI.h
  # The verified ros_sp image keeps the complete, mutually compatible header
  # set here. Its third_party SDK export lacks GxPixelFormat.h.
  HINTS /workspace/io/daheng ${GALAXY_SDK_ROOT}
  PATH_SUFFIXES inc include)
find_library(GALAXY_SDK_LIBRARY NAMES gxiapi
  HINTS /lib ${GALAXY_SDK_ROOT}
  PATH_SUFFIXES lib lib/amd64 lib/x86_64)
if(NOT GALAXY_SDK_INCLUDE_DIR OR NOT GALAXY_SDK_LIBRARY)
  message(FATAL_ERROR
    "Galaxy SDK not found. Set GALAXY_SDK_ROOT or provide GxIAPI.h, "
    "DxImageProc.h and libgxiapi.so in standard paths.")
endif()
if(NOT EXISTS "${GALAXY_SDK_INCLUDE_DIR}/DxImageProc.h")
  message(FATAL_ERROR "DxImageProc.h is missing beside GxIAPI.h")
endif()

add_library(camera_driver src/daheng_camera.cpp)
target_compile_features(camera_driver PUBLIC cxx_std_20)
target_compile_options(camera_driver PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${GALAXY_SDK_INCLUDE_DIR})
target_link_libraries(camera_driver ${GALAXY_SDK_LIBRARY} ${OpenCV_LIBS})

add_library(camera_ros src/camera_node.cpp)
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE CAMERA_NODE_GIT_REVISION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT CAMERA_NODE_GIT_REVISION)
  set(CAMERA_NODE_GIT_REVISION unknown)
endif()
target_compile_definitions(camera_ros PRIVATE
  CAMERA_NODE_GIT_REVISION="${CAMERA_NODE_GIT_REVISION}")
target_compile_features(camera_ros PUBLIC cxx_std_20)
target_compile_options(camera_ros PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(camera_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_ros camera_driver)
ament_target_dependencies(camera_ros
  camera_info_manager diagnostic_msgs rclcpp sensor_msgs)

add_executable(camera_node src/main.cpp)
target_compile_features(camera_node PRIVATE cxx_std_20)
target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_node camera_ros)
ament_target_dependencies(camera_node rclcpp)

install(TARGETS camera_driver camera_ros camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(camera_node_test test/camera_node_test.cpp)
  target_link_libraries(camera_node_test camera_ros)
  ament_target_dependencies(camera_node_test
    camera_info_manager diagnostic_msgs rclcpp sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(camera_info_manager diagnostic_msgs rclcpp sensor_msgs)
ament_package()
