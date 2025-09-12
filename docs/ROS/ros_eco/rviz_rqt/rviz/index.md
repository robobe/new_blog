---
tags:
    - ros
    - rviz
---
# rviz2
rviz2 is a visualization tool in ROS 2.

It provides a 3D GUI to visualize:

- Robot models (URDF)
- Sensor data (e.g., LIDAR, cameras)
- TF transforms
- Path planning
- Maps and occupancy grids

## install

```bash
sudo apt install ros-jazzy-rviz2
sudo apt install ros-${ROS_DISTRO}-rviz2
```

## usage

```bash title="load with config"
# rviz2 -d /path/to/config
```

```python title="launch file"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

PKG_BRINGUP = "turtlebot_bringup"


def generate_launch_description():
    ld =  LaunchDescription()
    
    config_file = PathJoinSubstitution([
        get_package_share_directory(PKG_BRINGUP),
        'config',
        'config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_file]
    )

    ld.add_action(rviz_node)
    return ld

```

---

## Resource
- [ Visualizations in ROS with RViz](https://docs.m2stud.io/cs/ros_additional/06-L3-rviz/)