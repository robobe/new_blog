cmake_minimum_required(VERSION 3.16)
project(spdlog_configured LANGUAGES CXX)

find_package(spdlog CONFIG REQUIRED)

add_executable(spdlog_configured main.cpp)
target_compile_features(spdlog_configured PRIVATE cxx_std_17)
target_link_libraries(spdlog_configured PRIVATE spdlog::spdlog)
