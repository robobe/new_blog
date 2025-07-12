
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(zenohpico) 
find_package(zenohcxx REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(pub_sub_cdr pub_sub.cpp)

target_link_libraries(pub_sub_cdr
    zenohcxx::zenohpico
)

ament_target_dependencies(pub_sub_cdr rclcpp std_msgs)


