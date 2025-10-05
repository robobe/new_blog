cmake_minimum_required(VERSION 3.22)

# Project setup
project(DemoApp VERSION 0.1.0 LANGUAGES CXX)

# Options
option(DEMO_BUILD_TESTS "Build tests" ON)

# Library
add_library(demolib src/lib.cpp)
add_library(demo::demolib ALIAS demolib)

target_include_directories(demolib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set_target_properties(demolib PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Executable
add_executable(demo_app src/main.cpp)

target_link_libraries(demo_app PRIVATE demo::demolib)
set_target_properties(demo_app PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Testing
include(CTest)
if (DEMO_BUILD_TESTS AND BUILD_TESTING)
    include(FetchContent)
    # Avoid overriding parent project's compiler/linker settings in gtest
    set(BUILD_GMOCK OFF CACHE BOOL "Disable gmock" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "Disable gtest install" FORCE)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    add_executable(test_add test/test_add.cpp)
    target_link_libraries(test_add PRIVATE demo::demolib GTest::gtest_main)
    add_test(NAME test_add COMMAND test_add)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS demolib demo_app
    EXPORT DemoAppTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT DemoAppTargets
    FILE DemoAppTargets.cmake
    NAMESPACE demo::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DemoApp
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/DemoAppConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/DemoAppConfig.cmake.in
               ${CMAKE_CURRENT_BINARY_DIR}/DemoAppConfig.cmake
               @ONLY)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/DemoAppConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/DemoAppConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DemoApp
)

# Packaging (CPack) - define variables BEFORE including CPack
set(CPACK_PACKAGE_NAME "DemoApp")
set(CPACK_PACKAGE_VENDOR "Demo")
set(CPACK_PACKAGE_CONTACT "example@example.com")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${PROJECT_VERSION}_${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_GENERATOR "DEB")

# Debian specific metadata
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Demo Maintainer <example@example.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://example.com/demoapp")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "DemoApp: A simple C++ demo with add() and greet() functions.")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON) # Auto-generate shared library dependencies
include(CPack)
