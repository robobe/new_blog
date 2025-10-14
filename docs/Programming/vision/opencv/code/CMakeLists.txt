cmake_minimum_required(VERSION 3.10)
project(hello_opencv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)



# Build the executable
add_executable(hello_opencv hello_cv.cpp)
# Include OpenCV headers
target_include_directories(hello_opencv PRIVATE ${OpenCV_INCLUDE_DIRS})
# Link against OpenCV libraries
target_link_libraries(hello_opencv ${OpenCV_LIBS})
