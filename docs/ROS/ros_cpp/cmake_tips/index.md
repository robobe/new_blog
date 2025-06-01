---
tags:
    - cmake
    - ros
    - cmakelist
    - tips
---

# ROS cmake usage tips

## Install directory set permission and filter

```cmake
install(DIRECTORY bin/
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.py"
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE WORLD_EXECUTE)

```

## Install file set permission 

```cmake
install(FILES config.yaml
        DESTINATION share/${PROJECT_NAME}/config
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

```

---

## Architecture Detection

```cmake
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(STATUS "Building for x86_64 architecture")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "Building for ARM64 (aarch64)")
else()
  message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

```