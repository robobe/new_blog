cmake_minimum_required(VERSION 3.16)
project(text_and_data_modeling LANGUAGES CXX)

include(CTest)

set(EXAMPLES string_ownership string_view_demo domain_model exercise_starter)

foreach(example IN LISTS EXAMPLES)
    add_executable(${example} ${example}.cpp)
    target_compile_features(${example} PRIVATE cxx_std_20)
    target_compile_options(${example} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()

if(BUILD_TESTING)
    add_test(NAME string_ownership_output COMMAND string_ownership)
    set_tests_properties(string_ownership_output PROPERTIES
        PASS_REGULAR_EXPRESSION "Original: rover")

    add_test(NAME string_view_output COMMAND string_view_demo)
    set_tests_properties(string_view_output PROPERTIES
        PASS_REGULAR_EXPRESSION "Prefix: camera")

    add_test(NAME domain_model_output COMMAND domain_model)
    set_tests_properties(domain_model_output PROPERTIES
        PASS_REGULAR_EXPRESSION "Mode: altitude hold")

    add_test(NAME exercise_starter_runs COMMAND exercise_starter)
endif()
