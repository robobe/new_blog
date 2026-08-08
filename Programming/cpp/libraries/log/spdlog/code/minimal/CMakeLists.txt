cmake_minimum_required(VERSION 3.16)
project(spdlog_minimal LANGUAGES CXX)

find_package(spdlog CONFIG REQUIRED)

add_executable(spdlog_minimal main.cpp)
target_compile_features(spdlog_minimal PRIVATE cxx_std_17)
target_link_libraries(spdlog_minimal PRIVATE spdlog::spdlog)
