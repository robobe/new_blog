cmake_minimum_required(VERSION 3.22)

# Project setup
project(DemoApp VERSION 0.1.0 LANGUAGES CXX)

# Require GoogleTest
find_package(GTest REQUIRED)
include(GoogleTest)

enable_testing()

# defining executable target and its source file
add_executable(
  test_demo
  test/test_demo.cpp
)
# to link the executable with GoogleTest main library
target_link_libraries(
  test_demo
  GTest::gtest_main
)

# to discover and register all tests in the executable
gtest_discover_tests(test_demo)