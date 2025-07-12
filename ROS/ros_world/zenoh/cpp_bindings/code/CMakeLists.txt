cmake_minimum_required(VERSION 3.10)
project(zenoh_ws)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(zenohpico) 
find_package(zenohcxx REQUIRED)

add_executable(pub_sub pub_sub.cpp)

target_link_libraries(pub_sub
    zenohcxx::zenohpico)
