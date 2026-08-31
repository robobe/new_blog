cmake_minimum_required(VERSION 3.20)
project(zmq_image_serialization LANGUAGES CXX)

include(CTest)

find_package(cppzmq REQUIRED)
find_package(msgpack-cxx REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(image_sender image_sender.cpp)
target_link_libraries(image_sender PRIVATE cppzmq msgpack-cxx ${OpenCV_LIBS})

add_executable(image_receiver image_receiver.cpp)
target_link_libraries(image_receiver PRIVATE cppzmq msgpack-cxx ${OpenCV_LIBS})

add_executable(serialization_roundtrip serialization_roundtrip.cpp)
target_link_libraries(serialization_roundtrip PRIVATE msgpack-cxx)

foreach(target IN ITEMS image_sender image_receiver serialization_roundtrip)
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()

if(BUILD_TESTING)
    add_test(NAME serialization_roundtrip COMMAND serialization_roundtrip)
endif()
