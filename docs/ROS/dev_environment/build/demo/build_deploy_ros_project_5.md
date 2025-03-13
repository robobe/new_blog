---
tags:
    - ros
    - deploy
    - tutorial
    - cross-compile
    - mixing
---

# ROS2 project from development to deployment, Part 4: Cross Compiler

# In progress

Build package for nvidia jetson using cross compiler  
using **colcon mixin** to set cross compiler flags

## Download and install cross compiler
Download toolchain from [jetpack 6.2](https://developer.nvidia.com/embedded/jetson-linux-r3643)

!!! note "toolchain"
    Every jetpack has it own toolchain, so make sure to download the correct one.

    The toolchain url can be build from

    ```bash title="get jetson linux version"
    cat /etc/nv_tegra_release
    #
    # R36 (release), REVISION: 4.3,
    ```
    ```bash
    https://developer.nvidia.com/embedded/jetson-linux-r3643
    ```


## Build rootfs
[Build rootfs using Debootstrap](https://robobe.github.io/new_blog/DevOps/tools/devops_debootstrap/?h=cross#set-cross-compiler-paths)

---

## config mixin

```yaml title="mixin file"
mixins:
  cross-arm64:
    build-args:
      cmake:
        - -DCMAKE_TOOLCHAIN_FILE=/path/to/aarch64_toolchain.cmake
        - -DCMAKE_CXX_FLAGS="--sysroot=/path/to/arm64/sysroot"
        - -DCMAKE_C_FLAGS="--sysroot=/path/to/arm64/sysroot"
        - -DCMAKE_EXE_LINKER_FLAGS="--sysroot=/path/to/arm64/sysroot"
  

```

```bash title="register mixing file"
colcon mixin add cross_compile ./cross_compile.mixin
colcon mixin update
```

```bash title="usage"
colcon build --mixin cross-arm64
```

---

### Demo

```bash title="aarch64.cmake"
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)  # Change to match your target architecture
set(CROSS_COMPILER_PATH /home/user/cross_compilers/aarch64--glibc--stable-2022.08-1)
# Set cross-compiler paths
set(CMAKE_C_COMPILER ${CROSS_COMPILER_PATH}/bin/${CMAKE_SYSTEM_PROCESSOR}-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_PATH}/bin/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu-g++)
set(CMAKE_SYSROOT /home/user/rootfs/ubuntuRootFS)  # Set to your root filesystem

# Set where CMake should look for libraries and headers
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT} ${CMAKE_SYSROOT}/usr ${CMAKE_SYSROOT}/usr/local)

# Only search inside the target rootfs
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ROS 2 paths
set(AMENT_PREFIX_PATH ${CMAKE_SYSROOT}/opt/ros/humble)
set(CMAKE_PREFIX_PATH ${AMENT_PREFIX_PATH})
```
`
```bash title="Humble ARM64 cross compiler"
mixins:
  cross-arm64:
    build-args:
      cmake:
        - -DCMAKE_TOOLCHAIN_FILE=/opt/cross_toolchain/aarch64.cmake

```