cmake_minimum_required(VERSION 3.16)

project(toolchain_and_debugging LANGUAGES CXX)

include(CTest)

add_library(greeter STATIC src/greeter.cpp)
target_include_directories(greeter PUBLIC include)
target_compile_features(greeter PUBLIC cxx_std_20)

add_executable(toolchain_demo src/main.cpp)
target_link_libraries(toolchain_demo PRIVATE greeter)

add_executable(preprocessor_demo preprocessor_demo.cpp)
target_compile_features(preprocessor_demo PRIVATE cxx_std_20)

add_executable(exercise_starter exercise_starter.cpp)
target_compile_features(exercise_starter PRIVATE cxx_std_20)

foreach(target greeter toolchain_demo preprocessor_demo exercise_starter)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()

add_test(NAME greeting_default COMMAND toolchain_demo)
set_tests_properties(greeting_default PROPERTIES
    PASS_REGULAR_EXPRESSION "Hello, world!"
)

add_test(NAME greeting_name COMMAND toolchain_demo Ada)
set_tests_properties(greeting_name PROPERTIES
    PASS_REGULAR_EXPRESSION "Hello, Ada!"
)

add_test(NAME preprocessor_application_name COMMAND preprocessor_demo)
set_tests_properties(preprocessor_application_name PROPERTIES
    PASS_REGULAR_EXPRESSION "Application: toolchain-demo"
)

add_test(NAME exercise_starter_runs COMMAND exercise_starter)
