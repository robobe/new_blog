---
title: Gazebo custom plugin with subscriber and publisher
tags:
    - gazebo
    - plugin
    - harmonic
    - publisher
    - subscriber
---

## Subscriber

<details>
<summary>Subscriber</summary>
```
--8<-- "docs/Simulation/Gazebo/custom_plugins/pub_sub/code/simple_sub.cc"
```
</details>

<details>
<summary>CMakeLists.txt</summary>

```c
cmake_minimum_required(VERSION 3.22)
project(SimpleParamReader)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(SimpleSub SHARED src/simple_sub/simple_sub.cc)

target_link_libraries(SimpleSub
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(TARGETS SimpleSub
  LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/bin)
```
</details>


## Publisher

<details>
<summary>Publisher</summary>
```
--8<-- "docs/Simulation/Gazebo/custom_plugins/pub_sub/code/simple_pub.cc"
```
</details>