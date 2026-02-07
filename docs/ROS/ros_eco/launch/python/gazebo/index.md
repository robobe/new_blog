---
title: Launch Gazebo and spawn robot
tags:
    - ros
    - gazebo
    - launch
    - ExecuteProcess
---
{{ page_folder_links() }}

Launch gazebo and spawn robot from urdf/xacro file or sdf


<div class="grid-container">
    <div class="grid-item">
        <a href="#harmonic">
            <p>Harmonic</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#gazebo-classic">
            Gazebo Classic
        </a>
    </div>
    <div class="grid-item">
        <a href="-">
        </a>
    </div>
</div>

---

## Harmonic

launch gazebo with custom world
gazebo search for worlds using `GZ_SIM_RESOURCE_PATH` environment

### Launch gazebo world
```python title="gazebo.launch.py"
--8<-- "docs/ROS/ros_eco/launch/python/gazebo/code/gazebo.launch.py"
```


### Launch gazebo and spawn robot
```python title="sim.launch.py"
--8<-- "docs/ROS/ros_eco/launch/python/gazebo/code/sim.launch.py"
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

---

## Gazebo classic
Using ExecuteProcess with gazebo environment variables
- Spawn robot (using sdf model)

```python
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
)


PKG = "cable_sim"
WORLD_NAME = "empty.world"


def generate_launch_description():
    model_env = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_path = "/workspace/models"
    model_path = model_path if model_env == "" else model_path + ":" + model_env

    plugin_env = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    resource_path = "/workspace/worlds"
    resource_path = (
        resource_path if plugin_env == "" else resource_path + ":" + plugin_env
    )

    plugin_env = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    plugin_path = "/workspace/plugins"
    plugin_path = plugin_path if plugin_env == "" else plugin_path + ":" + plugin_env

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            "empty.world",
        ],
        output="screen",
        additional_env={
            "GAZEBO_MODEL_PATH": model_path,
            "GAZEBO_RESOURCE_PATH": resource_path,
            "GAZEBO_PLUGIN_PATH": plugin_path,
        },
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-file",
            "/workspace/models/simple_box/model.sdf",
            "-entity",
            "robot",
            # optional pose:
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.2",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    return LaunchDescription([gazebo, spawn_robot])

```