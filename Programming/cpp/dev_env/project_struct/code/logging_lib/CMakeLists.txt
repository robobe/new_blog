cmake_minimum_required(VERSION 3.16)
project(logging_lib LANGUAGES CXX)

add_library(logging_lib src/log.cpp)
add_library(logging_lib::logging_lib ALIAS logging_lib)

target_include_directories(logging_lib PUBLIC include)
target_compile_features(logging_lib PUBLIC cxx_std_17)
