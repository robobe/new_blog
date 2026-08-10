cmake_minimum_required(VERSION 3.16)

project(yaml_read_demo LANGUAGES CXX)

find_package(yaml-cpp REQUIRED)

add_executable(read_yaml main.cpp)
add_executable(map_yaml_to_struct map_yaml_to_struct.cpp)
add_executable(update_struct_to_yaml update_struct_to_yaml.cpp)
target_link_libraries(read_yaml PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(map_yaml_to_struct PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(update_struct_to_yaml PRIVATE yaml-cpp::yaml-cpp)
target_compile_features(read_yaml PRIVATE cxx_std_17)
target_compile_features(map_yaml_to_struct PRIVATE cxx_std_17)
target_compile_features(update_struct_to_yaml PRIVATE cxx_std_17)
target_compile_options(read_yaml PRIVATE -Wall -Wextra -pedantic)
target_compile_options(map_yaml_to_struct PRIVATE -Wall -Wextra -pedantic)
target_compile_options(update_struct_to_yaml PRIVATE -Wall -Wextra -pedantic)

# Copy the example configuration into the build directory.
configure_file(config.yaml config.yaml COPYONLY)
