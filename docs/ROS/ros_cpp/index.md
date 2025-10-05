---
title: ROS2 Cpp
tags:
    - ros
    - cpp
---


{{ page_folder_links() }}

<div class="grid-container">
    <div class="grid-item">
        <a href="dev">
                <img src="images/vscode.png"  width="150" height="150">
                <p>VSCode dev environment</p>
        </a>
                   </div>
        <div class="grid-item">
            <a href="pluginlib">
                <img src="images/ros_pluginlib.png"  width="150" height="150">
                <p>Pluginlib</p>
            </a>
        </div>
        <div class="grid-item">
            <a href="cmake_tips">
                <img src="images/cmake.png"  width="150" height="150">
                <p>cmake tips</p>
            </a>
        </div>
    
</div>

---

## Simple Node
Simple cpp node with `CMakeLists.txt` 
Using clang as compiler

!!! tip "set clang using update-alternative"
    ```bash
    sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-18 100
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-18 100
    ```
     
- using `fmt` library to better string format


<details>
    <summary>Simple node</summary>

```cpp
--8<-- "docs/ROS/ros_cpp/code/pub_demo.cpp"
```
</details>



<details>
    <summary>CMakeLists.txt</summary>

```cmake
--8<-- "docs/ROS/ros_cpp/code/CMakeLists.txt"
```
</details>

<details>
    <summary>VSCode c_cpp_properties.json</summary>

```json
--8<-- "docs/ROS/ros_cpp/code/c_cpp_properties.json"
```
</details>

<div class="grid-container">
    <div class="grid-item">
            <a href="basic">
                <p>More basic examples</p>
            </a>
        </div>
        <div class="grid-item">
            <a href="">
                <p>---</p>
            </a>
        </div>
        <div class="grid-item">
            <a href="">
                <p>---</p>
            </a>
        </div>
    
</div>