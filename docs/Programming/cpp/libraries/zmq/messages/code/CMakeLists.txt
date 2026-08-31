cmake_minimum_required(VERSION 3.20)
project(zmq_messages LANGUAGES CXX)

find_package(cppzmq REQUIRED)

add_executable(message_server server.cpp)
target_link_libraries(message_server PRIVATE cppzmq)

add_executable(message_client client.cpp)
target_link_libraries(message_client PRIVATE cppzmq)

foreach(target IN ITEMS message_server message_client)
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()
