---
tags:
    - ros2
    - launch
    - python
    - PathJoinSubstitution
    - parameters
    - ParameterValue
---

# PathJoinSubstitution and parameter config file

Using `PathJoinSubstitution` to build path to parameter yaml file.

## Project

```bash
├── CMakeLists.txt
├── config
│   └── params.yaml
├── launch
│   └── param_yaml_demo.launch.py
├── launch_tutorial
│   ├── __init__.py
│   └── simple_node.py
├── package.xml
└── src

```

<details>
    <summary>param.yaml</summary>

```yaml title="config/params.yaml"
/**:
  ros__parameters:
    arg1: "from param file"
```
</details>


<details>
    <summary>param_yaml_demo.launch.py</summary>

```python title="launch/param_yaml_demo.launch"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    config_file = PathJoinSubstitution([
        get_package_share_directory('launch_tutorial'),
        'config',
        'params.yaml'
    ])

    node = Node(
        package='launch_tutorial',
        executable='simple_node.py',
        name='simple',
        output='screen',
        parameters=[config_file])
    
    ld.add_action(node)
    return ld

```
</details>



<details>
    <summary>simple_node.py</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.declare_parameter('arg1', "hello default")
        self.get_logger().info("Hello ROS2")
        self.get_logger().info(self.get_parameter("arg1").value)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
</details>

---

## Demo: Launch gz ros bridge

Launch gz bridge with yaml file that set from argument
using: 

- DeclareLaunchArgument
- LaunchConfiguration
- PathJoinSubstitution
- ParameterValue


```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

PKG_BRINGUP = 'ardupilot_bringup'
BRIDGE_CONFIG = "bridge.yaml"
CONFIG_FOLDER = "config"

def generate_launch_description():
    ld = LaunchDescription()

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=BRIDGE_CONFIG,
        description='Name of the bridge config file'
    )
    

    config_file = LaunchConfiguration('config_file')

    bridge_file = PathJoinSubstitution([
        get_package_share_directory(PKG_BRINGUP),
        CONFIG_FOLDER,
        config_file
    ])

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {'use_sim_time': True},
            {'config_file': ParameterValue(bridge_file, value_type=str)}
        ],
    )

    ld.add_action(config_file_arg)
    ld.add_action(ros_gz_bridge)


    return ld
    ```