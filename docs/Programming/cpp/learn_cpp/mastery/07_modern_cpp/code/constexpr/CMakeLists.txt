cmake_minimum_required(VERSION 3.16)
project(constexpr_lesson LANGUAGES CXX)

include(CTest)

foreach(example IN ITEMS constexpr_demo exercise_starter)
    add_executable(${example} ${example}.cpp)
    target_compile_features(${example} PRIVATE cxx_std_20)
    target_compile_options(${example} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()

if(BUILD_TESTING)
    add_test(NAME constexpr_demo_output COMMAND constexpr_demo)
    set_tests_properties(constexpr_demo_output PROPERTIES
        PASS_REGULAR_EXPRESSION "Compile-time square: 36")

    add_test(NAME constexpr_exercise_starter_runs COMMAND exercise_starter)
endif()
