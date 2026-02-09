---
title: ROS2 Launch Substitute
tags:
    - ros
    - launch
    - substitute
---
 
A Launch Substitution is a promise for a value that will only be known at launch runtime, not when the Python file is evaluated.

Substitutions are only meaningful in places where launch will resolve them later, such as:

- node name
- parameters
- remappings
- log messages
- conditions
- command arguments



| Substitution           | Meaning                          |
| ---------------------- | -------------------------------- |
| (`LaunchConfiguration`)(/ROS/ros_eco/launch/python/launch_argument/)  | Value from launch arguments      |
| `EnvironmentVariable`  | Value from env at runtime        |
| `PathJoinSubstitution` | Build paths safely               |
| `TextSubstitution`     | Literal text                     |
| `Command`              | Run a command and capture output |
| `FindPackageShare`     | Locate package at runtime        |


---

## Simple Demo

```python
Node(
    package='demo_nodes_cpp',
    executable='talker',
    name=LaunchConfiguration('node_name')
)

```

```python title="log substitute"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'robot_name',
            default_value='atlas'
        )
    )

    ld.add_action(
        LogInfo(
            msg=['Robot name is: ', LaunchConfiguration('robot_name')]
        )
    )

    return ld

```