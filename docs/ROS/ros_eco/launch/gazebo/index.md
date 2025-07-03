---
title: gazebo launch file
tags:
    - ros
    - gazebo
    - launch
---


{{ page_folder_links() }}

## Basic

launch gazebo with custom world
gazebo search for worlds using `GZ_SIM_RESOURCE_PATH` environment

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

WORLD_NAME="my_world.sdf"
ROS_GZ_PKG = "ros_gz_sim"

def generate_launch_description():
    ld = LaunchDescription()


    gz_args = " ".join([
        "-r",
        "-v4",
        WORLD_NAME
    ])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(ROS_GZ_PKG), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={
                        'gz_args': gz_args,
                        "on_exit_shutdown": "true"}.items()
             )

    
    ld.add_action(gazebo)
    return ld
```