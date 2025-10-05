---
title: fmt library
tags:
    - fmt
    - cpp
    - format
---

{{ page_folder_links() }}

{fmt} is an open-source formatting library providing a fast and safe alternative to C stdio and C++ iostreams.[more](https://github.com/fmtlib/fmt)

```bash title="install"
sudo apt install libfmt-dev
```


##  Simple Demo

```cpp
#include <fmt/core.h>
#include <iostream>


int main() {
    fmt::print("Hello, world!\n");
    fmt::print("Hello, {}!\n", "world");
    fmt::print("The answer is {}.\n", 42);
    fmt::print("Hex: {:#x}, Oct: {:#o}\n", 255, 255);
    fmt::print("Pi is approximately {:.2f}\n", 3.14159);
    fmt::print("Binary: {:#b}\n", 42);
    fmt::print("{1} {0}\n", "world", "Hello");

    std::string data = fmt::format("Formatted number: {:.3f}", 3.14159);
    std::cout << data << std::endl;
    return 0;
}
```

```cmake
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(CppTutorial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


find_package(fmt REQUIRED)

add_executable(fmt_demo hello.cpp)
target_link_libraries(fmt_demo PRIVATE fmt::fmt)
```