---
tags:
    - ros
    - interfaces
    - message
---

# ROS2 interfaces

Messages are the fundamental units of data that are exchanged between nodes using topics. They are part of the ROS 2 communication interfaces, which also include services and actions.

## msg and srv files

A message in ROS 2 is a typed data structure 
Each line define one field `<type> <name>`
message (service and action) files can include

- primitive types: int32, float64, string
- compound predefine message or custom message
- arrays: int32[]


---

## Create custom message

### cmake

```c
cmake_minimum_required(VERSION 3.8)
project(media_manager_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files

)
set(srv_files
  "srv/GetMediaFileList.srv"
  )


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

```

### package.xml
Add this lines before `<export>` tag

```xml
  <depend>rosidl_default_generators</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```