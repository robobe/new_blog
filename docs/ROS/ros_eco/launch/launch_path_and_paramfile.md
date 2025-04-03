---
tags:
    - ros2
    - launch
    - python
    - PathJoinSubstitution
    - parameters
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

