cmake_minimum_required(VERSION 3.20)
project(remote_debug_demo LANGUAGES CXX)

add_executable(remote_debug_demo src/main.cpp)
target_compile_features(remote_debug_demo PRIVATE cxx_std_17)
target_compile_options(remote_debug_demo PRIVATE -Wall -Wextra -Wpedantic)
