cmake_minimum_required(VERSION 3.22)
cmake_policy(SET CMP0135 NEW)
project(traffic_assignment)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find Eigen 3.4
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# set include directories
include_directories(
    "."
)

add_library(traffic_assignment)
target_sources(traffic_assignment
    PRIVATE
    "road_network/format.h"
    "road_network/graph.cc"
    "road_network/graph.h"
    "road_network/link_performance.h"
    "road_network/matrix.h"
)

# TEST
# this project uses GoogleTest
enable_testing()
find_package(GTest REQUIRED)

function(ta_test test_file)
  get_filename_component(test_target_name "${test_file}" NAME_WE)

  add_executable("${test_target_name}" "")
  target_sources("${test_target_name}"
      PRIVATE
      "${test_file}")

  target_link_libraries("${test_target_name}" traffic_assignment GTest::gtest_main)

  add_test(NAME "${test_target_name}" COMMAND "${test_target_name}")
endfunction()

ta_test("main_test.cc")
target_link_libraries(main_test Eigen3::Eigen)
ta_test("road_network/graph_test.cc")
