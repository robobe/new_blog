---
tags:
    - ros
    - deploy
    - tutorial
    - cross-compile
    - mixing
---

# ROS2 project from development to deployment, Part 4: Cross Compiler

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

```bash title="Humble ARM64 cross compiler"
mixins:
  cross-arm64:
    build-args:
      cmake:
        - -DCMAKE_TOOLCHAIN_FILE=/opt/cross_toolchain/aarch64.cmake
        - -DCMAKE_CXX_FLAGS="--sysroot=/opt/sysroot/arm64"
        - -DCMAKE_C_FLAGS="--sysroot=/opt/sysroot/arm64"
        - -DCMAKE_EXE_LINKER_FLAGS="--sysroot=/opt/sysroot/arm64"
        - -DBUILD_TESTING=OFF
        - -DCMAKE_FIND_ROOT_PATH="/opt/ros/humble;/opt/sysroot/arm64"
        - -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER
        - -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY
        - -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu
```