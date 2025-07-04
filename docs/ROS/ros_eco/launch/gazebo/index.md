---
title: Launch Gazebo and spawn robot
tags:
    - ros
    - gazebo
    - launch
---
{{ page_folder_links() }}

Launch gazebo and spawn robot from urdf/xacro file

## Basic

launch gazebo with custom world
gazebo search for worlds using `GZ_SIM_RESOURCE_PATH` environment

### Launch gazebo world
```python title="gazebo.launch.py"
--8<-- "docs/ROS/ros_eco/launch/gazebo/code/gazebo.launch.py"
```


### Launch gazebo and spawn robot
```python title="sim.launch.py"
--8<-- "docs/ROS/ros_eco/launch/gazebo/code/sim.launch.py"
```

### Declare GZ_SIM_RESOURCE_PATH

Use dsv to set environment variable

[ament dsv](/ROS/ros_eco/build_system/ament)

```bash
.
├── CMakeLists.txt
├── hooks
│   ├── robot_loc_gazebo.dsv.in
│   └── robot_loc_gazebo.sh.in
```

```bash title="robot_loc_gazebo.dsv.in"
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share;@CMAKE_INSTALL_PREFIX@/share/robot_loc_gazebo/worlds
```

```bash title="obot_loc_gazebo.sh.in"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/@PROJECT_NAME@/worlds"
```

```c title="CMakeLists.txt"
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
```
