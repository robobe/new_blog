---
tags:
    - ros
    - xacro
---

# XACRO

Xacro (XML Macros) Xacro is an XML macro language. With xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.
[more](http://wiki.ros.org/xacro)


## Install ROS support

```bash 
sudo apt install ros-humble-xacro
```

---

## Usage
### cli

```bash
xacro hello.urdf.xacro > hello.urdf
```

### from launch file

```python title="load xacro and spawn in gazebo classic"
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

PKG_DESCRIPTION = "gz_tutorial_description"

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PKG_DESCRIPTION))
    xacro_file = os.path.join(pkg_path,'urdf','my_robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])
```

---

## XACRO terms

- [match]()
- [property]()
- [args]()
- [include]()
- [condition]()
- [macro]()
- [loops]()