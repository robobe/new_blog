---
tags:
    - ros
    - build
    - cpack
---

# Using CPack to build ROS2 debian package
- Read metadata from package.xml
- Support multiple architecture

!!! note "TBD"
    Why not `bloom`
     
## Prerequisite
- `xmllint` is required to read package.xml file


## Issue
- handle python packages automatically
  - edit postinst file to install python package
  - use rosdep to reslove python package reference automatically


---

## Usage
- Include `Packing.cmake` in your `CMakeLists.txt` file

```cmake
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(Packing)
```

```cmake title="Packing.cmake"
option(OUTPUT_FOLDER "OUTOUT_FOLDER" "")

execute_process (
    COMMAND bash -c "echo 'cat //package/name/text()' | xmllint --shell ${CMAKE_CURRENT_SOURCE_DIR}/package.xml | sed '/^\\/ >/d' | sed -z 's/\\n//g'"
    OUTPUT_VARIABLE ROS_PROJECT_NAME
)

execute_process (
    COMMAND bash -c "echo 'cat //package/description/text()' | xmllint --shell ${CMAKE_CURRENT_SOURCE_DIR}/package.xml | sed '/^\\/ >/d' | sed -z 's/\\n//g'"
    OUTPUT_VARIABLE ROS_PROJECT_DESCRIPTION
)

execute_process (
    COMMAND bash -c "echo 'cat //package/maintainer/text()' | xmllint --shell ${CMAKE_CURRENT_SOURCE_DIR}/package.xml | sed '/^\\/ >/d' | sed -z 's/\\n//g'"
    OUTPUT_VARIABLE ROS_PROJECT_MAINTAINER
)
execute_process (
    COMMAND bash -c "echo 'cat //package/version/text()' | xmllint --shell ${CMAKE_CURRENT_SOURCE_DIR}/package.xml | sed '/^\\/ >/d' | sed 's/<[^>]*>//g' | sed -z 's/\\n//g'"
    OUTPUT_VARIABLE ROS_PACKAGE_VERSION
)

execute_process (
    COMMAND bash -c "xmllint --xpath '//depend/text()' ${CMAKE_CURRENT_SOURCE_DIR}/package.xml | awk '{printf \"%s%s\", sep, $0; sep = \",\"}'"
    OUTPUT_VARIABLE DEPENDS
)

execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

set(CPACK_PACKAGE_NAME ${ROS_PROJECT_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${ROS_PROJECT_DESCRIPTION})
set(CPACK_PACKAGE_VENDOR "ROBOBE Company")
set(CPACK_PACKAGE_CONTACT "${ROS_PROJECT_MAINTAINER}@dev.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${ROS_PROJECT_MAINTAINER}")
if (NOT DEFINED DEPENDS)
    message(ERROR "no depend tag in package.xml")
else()
    set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEPENDS})
endif()

set(CPACK_VERBATIM_VARIABLES YES)

set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
if(${OUTPUT_FOLDER} STREQUAL "")
    SET(CPACK_OUTPUT_FILE_PREFIX "_packages")
else()
    SET(CPACK_OUTPUT_FILE_PREFIX "${OUTPUT_FOLDER}")
endif()


string(REPLACE "." ";" VERSION ${ROS_PACKAGE_VERSION})
list (GET VERSION 0 PROJECT_VERSION_MAJOR)
list (GET VERSION 1 PROJECT_VERSION_MINOR)
list (GET VERSION 2 PROJECT_VERSION_PATCH)

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

set (CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/humble")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CMAKE_SYSTEM_NAME Linux)


set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/deb")
set(ROS_VER $ENV{ROS_DISTRO})
set(CPACK_PACKAGE_FILE_NAME "ros-${ROS_VER}-${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION}-${ARCHITECTURE}")
# set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_GENERATOR "DEB")

include(CPack)
```