cmake_minimum_required(VERSION 3.16)

project(rknn_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(rknn_demo
    main.cpp
)

target_link_libraries(rknn_demo
    PRIVATE
        ${OpenCV_LIBS}
        rknnrt
)

