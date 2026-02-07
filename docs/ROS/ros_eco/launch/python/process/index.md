---
title: Launch process
tags:
    - ros
    - launch
    - process
    - ExecuteProcess
---

Starts any OS process (not necessarily a ROS node)
Think: subprocess.Popen, but launch-managed

Useful for:
- non-ROS binaries
- Gazebo, simulators, tools
- shell scripts
- CLI utilities

---

```python
ExecuteProcess(
    cmd=['my_binary', '--arg1', 'value'],
    cwd='/tmp',                     # working directory
    output='screen',                 # screen | log | both
    additional_env={}
)
```

| Feature             | `env` | `additional_env` |
| ------------------- | ----- | ---------------- |
| Keeps existing vars | ❌     | ✅                |
| Safe default        | ❌     | ✅                |
| ROS-friendly        | ❌     | ✅                |
| Typical usage       | Rare  | Common           |


---

## Demo: Run binary
Run ardupilot SITL


```python

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


PKG = 'ardupilot_bringup'

def generate_launch_description():
    package_share_dir = get_package_share_directory(PKG)
    sitl_bin = f"{package_share_dir}/bin/arducopter"
    param_path = f"{package_share_dir}/params/copter_default.param"

    ld = LaunchDescription()
    sitl = ExecuteProcess(
            cmd=[sitl_bin, '--model', 'quad', '--speedup', '1', '--slave', '0',
                 '--defaults', param_path,
                 '--sim-address', '127.0.0.1', '-I0'],
            output='screen'
        )
    
    ld.add_action(sitl)
    return ld
```

---

## Demo: Run gazebo classic
Run gazebo classic and set GAZEBO environment variables


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



    return LaunchDescription([gazebo])

```