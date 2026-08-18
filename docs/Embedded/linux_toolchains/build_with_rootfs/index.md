---
title: Cross-compile OpenCV with a target root filesystem
tags:
    - rsync
    - cross compiler
    - opencv
    - rootfs
---

This example builds an OpenCV program on an x86_64 host for an AArch64 Linux
target. The target's headers and libraries are copied into a local *sysroot*.
CMake then searches that sysroot while the cross-compiler produces an AArch64
executable. Finally, the executable is copied to the target and tested over
SSH without X11 or a desktop session.

The process is:

1. Copy `/lib` and `/usr` from the target to a sysroot on the host.
2. Configure CMake with an AArch64 toolchain file and the sysroot.
3. Build and link against the target's OpenCV libraries.
4. Copy the executable to the target with `scp`.
5. Run it through SSH and verify the generated PNG.

The sysroot and compiler must target the same architecture and ABI. This
example uses the GNU/glibc AArch64 toolchain because the target root filesystem
contains glibc libraries.

!!! tip "ABI"
    
    **ABI** means *Application Binary Interface*. While an API defines how source
    code calls functions, an ABI defines how compiled machine code works together.
    It includes calling conventions, register use, data type sizes and alignment,
    symbol naming, object-file format, and the expected runtime libraries and
    dynamic loader. Two systems can both use AArch64 but still be incompatible if,
    for example, one executable expects glibc and the other system provides musl.

!!! tip "Match the compiler to the target ABI"
    Copying headers and libraries from the target is not enough if the compiler
    produces binaries for a different ABI. Check the target with `uname -m`,
    inspect the executable with `file build-arm/hello_cv`, and use `ldd` on the
    target executable. The architecture, libc, dynamic loader, and linked
    libraries must agree.

---

## Install prerequisites

Install `rsync` on both the host and target. On the host, also install the
AArch64 cross-compiler and CMake:

```bash
sudo apt install rsync cmake g++-aarch64-linux-gnu
```

!!! tip "ABI compatibility"
    
    ```bash 
    sudo apt install \
        gcc-12-aarch64-linux-gnu \
        g++-12-aarch64-linux-gnu
    ```

## Configure SSH

Add a short name for the target to the host's SSH configuration:

```title=".ssh/config"
Host radxa
    HostName 192.168.1.60
    User rock
    IdentityFile ~/.ssh/id_ed25519
```

Test the connection before continuing:

```bash
ssh radxa uname -m
```

The expected architecture is `aarch64`.

## Create the sysroot

Create a local directory on the host:

```bash
mkdir -p ~/sysroots/radxa
```

```bash
rsync -avL \
    radxa:/lib \
    radxa:/usr \
    ~/sysroots/radxa/
```

`-L` follows symbolic links so the sysroot receives the actual library files.
Run the same command again whenever packages such as OpenCV are updated on the
target.

## Create the CMake toolchain file

The toolchain file tells CMake that this is a cross-build, selects the AArch64
compilers, and restricts library, header, and package searches to the copied
sysroot. Host programs needed during the build are still found on the host.

```cmake title="cmake/toolchain-aarch64.cmake"
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc-12)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++-12)

set(CMAKE_SYSROOT "$ENV{HOME}/sysroots/radxa")
set(CMAKE_FIND_ROOT_PATH "$ENV{HOME}/sysroots/radxa")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

!!! tip "ABI libc version"
    target hardware: radxa zero 3w
    target os: debian kernel 6.1
    libcxx: GLIBCXX_3.4.30
    
    Use gnu compiler version 12
    

|   Setting        |  Search behavior            |  Reason
|------------------|-----------------------------|-----------------------------
|   PROGRAM NEVER  |  Search host paths          |  Build tools must run on the host.
|   LIBRARY ONLY   |  Search the target sysroot  |  Link target-compatible libraries.
|   INCLUDE ONLY   |  Search the target sysroot  |  Use headers installed on the target.
|   PACKAGE ONLY   |  Search the target sysroot  |  Find target CMake packages.


## Define the application

`find_package(OpenCV)` now finds the target's OpenCV package inside the
sysroot. The resulting include paths and libraries are used to build
`hello_cv`.

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.16)

project(hello_cv LANGUAGES CXX)

find_package(OpenCV REQUIRED)

add_executable(hello_cv main.cpp)
# Include OpenCV headers
target_include_directories(hello_cv PRIVATE ${OpenCV_INCLUDE_DIRS})
# Link against OpenCV libraries
target_link_libraries(hello_cv ${OpenCV_LIBS})
```

The application creates an image in memory, draws a rectangle and text, and
writes the result with `cv::imwrite()`. It does not call `cv::imshow()` or
`cv::waitKey()`, so it does not need X11 and can run in a headless SSH session.

## Code example

The program performs a complete OpenCV test without opening a window:

1. It reads an optional output filename from the first command-line argument.
   If none is provided, it uses `opencv-headless-check.png`.
2. It prints `CV_VERSION` to confirm which OpenCV version is available at
   runtime.
3. It creates a 400 x 300, three-channel BGR image with a blue background.
4. It draws a white rectangle and green text, testing basic OpenCV image and
   drawing operations.
5. It writes the image to disk with `cv::imwrite()`. A write failure produces
   an error message and exit status `1`.
6. It prints the image dimensions and mean BGR values. These terminal values
   provide a quick check over SSH even before the PNG is copied to the host.

No HighGUI functions are used, so the program does not require `DISPLAY`, X11,
Wayland, or an attached monitor.

```cpp
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    const std::string output_path =
        argc > 1 ? argv[1] : "opencv-headless-check.png";

    std::cout << "Hello, OpenCV!" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Create an image in memory; no X11 display is required.
    cv::Mat image(300, 400, CV_8UC3, cv::Scalar(255, 0, 0));

    cv::rectangle(image, cv::Point(50, 50), cv::Point(350, 250), cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "Hello OpenCV", cv::Point(60, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    if (!cv::imwrite(output_path, image)) {
        std::cerr << "Failed to write image: " << output_path << std::endl;
        return 1;
    }

    const cv::Scalar mean_bgr = cv::mean(image);
    std::cout << "Image size: " << image.cols << "x" << image.rows << std::endl;
    std::cout << "Mean BGR: " << mean_bgr[0] << ", " << mean_bgr[1]
              << ", " << mean_bgr[2] << std::endl;
    std::cout << "Wrote: " << output_path << std::endl;

    return 0;
}

```


## Configure and build

Run these commands from the directory containing `CMakeLists.txt`:

```bash
cmake -S . -B build-arm \
    -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake
```

```bash
cmake --build build-arm
```

Confirm that the result is an AArch64 executable before copying it:

```bash
file build-arm/hello_cv
```

## Copy and run on the target

Copy the executable, run it through SSH, and request an output image in `/tmp`:

```bash
scp build-arm/hello_cv radxa:/tmp/
ssh radxa '/tmp/hello_cv /tmp/opencv-headless-check.png'
```

Successful output reports the OpenCV version, image size, mean BGR values, and
the path of the generated file. Verify the image on the target:

```bash
ssh radxa 'file /tmp/opencv-headless-check.png && ls -lh /tmp/opencv-headless-check.png'
```

To inspect it visually, copy it back to the host and open it locally:

```bash
scp radxa:/tmp/opencv-headless-check.png .
```

If the executable reports a missing shared library, use `ldd` on the target to
identify which runtime dependency is unavailable:

```bash
ssh radxa 'ldd /tmp/hello_cv'
```
