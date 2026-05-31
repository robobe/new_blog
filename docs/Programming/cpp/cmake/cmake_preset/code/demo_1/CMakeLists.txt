# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(option_demo LANGUAGES CXX)

option(USE_FAST_MODE "Build with fast mode enabled" OFF)

add_executable(app hello.cpp)

# if(USE_FAST_MODE)
#   target_compile_definitions(app PRIVATE USE_FAST_MODE)
# endif()