---
title: Zenoh cpp bindings
tags:
    - zenoh
    - cpp
    - bindings
---

{{ page_folder_links() }}

## Install
Download deb's file from github release page

!!! note 
    zeboh cpp binding depend in zeno_pico or zeno_c
     

- [Zenoh Pico](https://github.com/eclipse-zenoh/zenoh-pico/releases/download/1.4.0/zenoh-pico-1.4.0-linux-x64-debian.zip)
- [Zenoh cpp bindings](https://github.com/eclipse-zenoh/zenoh-cpp/releases/download/1.4.0/zenohcpp-1.4.0-debian.zip)


---

## Demo: pub / sub
Pub Sub data using zenoh

<details>
    <summary>pub_sub.cpp</summary>

```cpp
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/pub_sub.cpp"
```
</details>


<details>
    <summary>CMakeLists.txt</summary>

```c
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/CMakeLists.txt"
```
</details>

---

## Demo: pub / sub with ROS message serialization

<details>
    <summary>pub_sub.cpp</summary>

```cpp
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/ros_serial_message/pub_sub.cpp"
```
</details>


<details>
    <summary>CMakeLists.txt</summary>

```c
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/ros_serial_message/CMakeLists.txt"
```
</details>

---

## Demo: Subscribe to ros message with zenoh bridge

```bash title="terminal 1"
# Public message using ros
ros2 topic pub my_int32_topic std_msgs/msg/Int32 "{data: 10}"
```

```bash title="terminal 2"
# Run zenoh bridge
zenoh-bridge-ros2dds
```

```bash title="terminal 3"
# Run subscriber
./sub
```

<details>
    <summary>sub.cpp</summary>

```cpp
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/zenoh_bridge/sub.cpp"
```
</details>


<details>
    <summary>CMakeLists.txt</summary>

```c
--8<-- "docs/ROS/ros_world/zenoh/cpp_bindings/code/zenoh_bridge/CMakeLists.txt"
```
</details>

---

## Resource
- [read the doc (1.4)](https://zenoh-cpp.readthedocs.io/en/1.4.0/index.html)