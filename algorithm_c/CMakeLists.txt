cmake_minimum_required(VERSION 3.16)
project(dart_algorithm_c VERSION 0.1.0 LANGUAGES C)

add_library(dart_algorithm STATIC
  src/dart_algorithm.c
  src/dart_control.c
  src/dart_math.c
  src/dart_self_ekf.c
  src/dart_singer.c)
target_compile_features(dart_algorithm PUBLIC c_std_11)
target_compile_options(dart_algorithm PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(dart_algorithm PUBLIC include PRIVATE src)
target_link_libraries(dart_algorithm PUBLIC m)

include(CTest)
if(BUILD_TESTING)
  add_executable(dart_algorithm_test test/test_algorithm.c)
  target_compile_features(dart_algorithm_test PRIVATE c_std_11)
  target_compile_options(dart_algorithm_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(dart_algorithm_test PRIVATE dart_algorithm)
  add_test(NAME dart_algorithm_test COMMAND dart_algorithm_test)
endif()

install(TARGETS dart_algorithm ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
