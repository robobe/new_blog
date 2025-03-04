---
tags:
    - devops
    - tools
---

# Debootstrap

- First Stage: Downloads the basic packages and sets up the environment.
- Second Stage: Completes the system setup, installs additional packages, and configures the environment to be fully usable.


## First stage

```
sudo debootstrap --arch=arm64 --foreign jammy /home/user/rootfs/ubuntuRootFS http://us.ports.ubuntu.com/ubuntu-ports
```

## Second stage

```bash
# Copy qemu binary to emulate ARM architecture on x86 machine
sudo cp /usr/bin/qemu-aarch64-static /path/to/rootfs/usr/bin/

# Chroot into the new rootfs
sudo chroot /path/to/rootfs

# Run the second-stage installation
/debootstrap/debootstrap --second-stage

```

```bash
deb http://us.ports.ubuntu.com/ubuntu-ports jammy main restricted
deb http://us.ports.ubuntu.com/ubuntu-ports jammy-updates main restricted
# deb-src http://tw.ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse
# deb-src http://tw.ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse
deb http://us.ports.ubuntu.com/ubuntu-ports jammy universe
deb http://us.ports.ubuntu.com/ubuntu-ports jammy-updates universe
deb http://us.ports.ubuntu.com/ubuntu-ports jammy multiverse
deb http://us.ports.ubuntu.com/ubuntu-ports jammy-updates multiverse
```

```bash
sudo mount -o bind /dev /home/user/rootfs/ubuntuRootFS/dev
sudo mount -t devpts devpts /home/user/rootfs/ubuntuRootFS/dev/pts
sudo mount -o bind /proc /home/user/rootfs/ubuntuRootFS/proc
sudo mount -o bind /sys /home/user/rootfs/ubuntuRootFS/sys
sudo mount -o bind /run /home/user/rootfs/ubuntuRootFS/run

```

```bash
sudo chroot /path/to/rootfs /bin/bash

```

## install ros on the rootfs

```bash
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

```

```
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
```

```
$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```

```
apt update
```

```
sudo apt install ros-humble-ros-base
```


```cmake title=""toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)  # Change to match your target architecture

# Set cross-compiler paths
set(CMAKE_C_COMPILER /path/to/cross-compiler/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /path/to/cross-compiler/bin/aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /path/to/rootfs)  # Set to your root filesystem

# Set where CMake should look for libraries and headers
set(CMAKE_FIND_ROOT_PATH /path/to/rootfs /path/to/rootfs/usr /path/to/rootfs/usr/local)

# Only search inside the target rootfs
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ROS 2 paths
set(AMENT_PREFIX_PATH /path/to/rootfs/opt/ros/humble)
set(CMAKE_PREFIX_PATH ${AMENT_PREFIX_PATH})


```

```
colcon build --cmake-args -DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain.cmake

```