set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(zenohpico) 
find_package(zenohcxx REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(sub sub.cpp)

target_link_libraries(sub
    zenohcxx::zenohpico
)

ament_target_dependencies(sub rclcpp std_msgs)


