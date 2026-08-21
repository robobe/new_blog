cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)

set(LOGGING_LIB_SOURCE "" CACHE PATH "Path to the logging_lib source directory")
if(NOT EXISTS "${LOGGING_LIB_SOURCE}/CMakeLists.txt")
    message(FATAL_ERROR "Set LOGGING_LIB_SOURCE to the logging_lib source directory")
endif()

add_subdirectory("${LOGGING_LIB_SOURCE}" "${CMAKE_BINARY_DIR}/logging_lib")

add_executable(my_app src/main.cpp)
target_link_libraries(my_app PRIVATE logging_lib::logging_lib)
