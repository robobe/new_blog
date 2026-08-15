 cmake_minimum_required(VERSION 3.16)

project(hello_cv LANGUAGES CXX)

find_package(OpenCV REQUIRED)

add_executable(hello_cv main.cpp)
# Include OpenCV headers
target_include_directories(hello_cv PRIVATE ${OpenCV_INCLUDE_DIRS})
# Link against OpenCV libraries
target_link_libraries(hello_cv ${OpenCV_LIBS})