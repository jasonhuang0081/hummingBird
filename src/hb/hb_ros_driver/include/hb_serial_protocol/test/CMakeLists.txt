cmake_minimum_required(VERSION 3.0.2)
project(hb_serial_test)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTest REQUIRED)

include_directories(
  ..
  ${GTEST_INCLUDE_DIRS}
)

add_executable(hb_serial_test test.cpp)
target_link_libraries(hb_serial_test ${GTEST_LIBRARIES} pthread)
