---
title: Create simple pub/sun node
tags:
    - ros2
    - cpp
    - pub
    - node
    - tutorial
---

{{ page_folder_links() }}

Basic ros2 node using cpp, split the code into header file in implementation


## Publisher
Separate node file into header and implementation file, use main file as entry point


```
cpp_demo
├── CMakeLists.txt
├── include
│   └── cpp_demo
│       └── string_publisher.hpp
├── package.xml
└── src
    ├── pub_demo_main.cpp
    └── string_publisher.cpp
```

1. Code Reusability
      - The class (e.g., your publisher node) can be reused in other projects or tests without modification.
      - Only the main file is responsible for launching the node, while the logic is encapsulated in the class.
2. Separation of Concerns
      - The main function handles application startup and shutdown.
      - The class handles the node’s logic (publishing, subscribing, etc.).
3. Easier Testing
      - You can write unit tests for your class without running the whole ROS node.
      - The class can be instantiated and tested independently from the ROS2 runtime.
4. Cleaner Build Structure
      - Header and implementation files (.hpp/.cpp) keep interfaces and logic organized.
      - The main.cpp file is small and only responsible for node instantiation and spinning.

<details>
    <summary>include/string_publisher.hpp</summary>

```cpp
--8<-- "docs/ROS/ros_cpp/basic/code/string_publisher.hpp"
```
</details>

<details>
    <summary>pub_demo_main.cpp</summary>

```cpp
--8<-- "docs/ROS/ros_cpp/basic/code/pub_demo_main.cpp"
```
</details>

<details>
    <summary>string_publisher.cpp</summary>

```cpp
--8<-- "docs/ROS/ros_cpp/basic/code/string_publisher.cpp"
```
</details>


<details>
    <summary>CMakeLists.txt</summary>

```cpp
--8<-- "docs/ROS/ros_cpp/basic/code/CMakeLists.txt"
```
</details>