cmake_minimum_required(VERSION 3.8)
project(cpp_demo)

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)

add_executable(pub_demo src/pub_demo.cpp)
target_link_libraries(pub_demo fmt::fmt)
ament_target_dependencies(pub_demo rclcpp std_msgs)

install(TARGETS pub_demo
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
