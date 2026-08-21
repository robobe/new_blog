cmake_minimum_required(VERSION 3.20)

project(calculator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

# -------------------------
# Application/library code
# -------------------------

add_library(calculator
    src/calculator.cpp
)

target_include_directories(calculator
    PUBLIC
        include
)

# -------------------------
# Tests
# -------------------------

if(BUILD_TESTING)
    find_package(Catch2 3 REQUIRED)

    add_executable(calculator_tests
        tests/test_calculator.cpp
    )

    target_link_libraries(calculator_tests
        PRIVATE
            calculator
            Catch2::Catch2WithMain
    )

    include(Catch)
    catch_discover_tests(calculator_tests)
endif()
