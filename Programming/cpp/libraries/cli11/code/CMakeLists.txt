cmake_minimum_required(VERSION 3.16)

project(cli11_hello LANGUAGES CXX)

find_package(CLI11 CONFIG REQUIRED)

add_executable(cli11_hello main.cpp)
target_link_libraries(cli11_hello PRIVATE CLI11::CLI11)
target_compile_features(cli11_hello PRIVATE cxx_std_17)
target_compile_options(cli11_hello PRIVATE -Wall -Wextra -pedantic)
