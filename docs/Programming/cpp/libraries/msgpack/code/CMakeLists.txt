cmake_minimum_required(VERSION 3.10)
project(msgpack_define_map_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Msgpack REQUIRED)

add_executable(msgpack_example demo.cpp)
target_include_directories(msgpack_example PRIVATE  ${MSGPACK_INCLUDE_DIRS})
