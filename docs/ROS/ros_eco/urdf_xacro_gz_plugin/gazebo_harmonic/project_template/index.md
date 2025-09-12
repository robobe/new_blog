---
tags:
    - ros
    - gazebo
    - project
    - template
---

# ROS2 Gazebo Project

---

## Resources
- [Guide to ros_gz_project_template for ROS 2 and Gazebo Development](https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/){:target="_blank"}
- [github: ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template/tree/main){:target="_blank"}


```
├── loc_bringup
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── gazebo.launch.py
│   └── package.xml
└── loc_gazebo
    ├── CMakeLists.txt
    ├── hooks
    │   ├── loc_gazebo.dsv.in
    │   └── loc_gazebo.sh.in
    ├── package.xml
    └── worlds
        └── world.sdf
```


### gazebo package
```title="loc_gazebo.dsv"
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share;@CMAKE_INSTALL_PREFIX@/share/loc_gazebo/worlds
```

```title="loc_gazebo.sh"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/@PROJECT_NAME@/worlds"
```

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(loc_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

```

---

### Bringup package

```python title="gazebo.launch.py"
--8<-- "docs/ROS/ros_eco/urdf_xacro_gz_plugin/gazebo_harmonic/project_template/code/gazebo.launch.py"
```
