---
title: ROS2 Parameters
tags:
    - ros2
    - parameters
---
{{ page_folder_links() }}

Using ROS2 parameter file from cli and launch file
Use multiple parameters files to demonstrate parameter override

## parameter file

### Demo:
Simple python Node with multiple parameters


<details><summary>Node</summary>


```python


import rclpy
from rclpy.node import Node

PARAM1 = "param1"
PARAM2 = "param2"

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        p1 = self.declare_parameter(PARAM1, 1)
        p2 = self.declare_parameter(PARAM2, 2)
        self.get_logger().info(f"P1: {p1.value}")
        self.get_logger().info(f"P1: {p2.value}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>




#### Param files

```yaml title="params1.yaml"
minimal:
  ros__parameters:
    param1: 10
    param2: 20
```

```yaml title="params2.yaml"
/**:
  ros__parameters:
    param1: 100
    param2: 200
```

!!! tip "wildcard"
    The `params2.yaml` use wildcard as node name matching
     

---

#### usage

!!! note "wildcard"
    Wildcards can be used for node names and namespaces.  
    - `*`  matches a single token delimited by slashes (/). 
    - `**` matches zero or more tokens delimited by slashes. 
    - Partial matches are not allowed (e.g. foo*).
    
    ```yaml
    /**:
        ros__parameters:
            wildcard_full: "Full wildcard for any namespaces and any node names"

    /**/parameter_blackboard:
        ros__parameters:
            wildcard_namespace: "Wildcard for a specific node name under any namespace"

    /*:
        ros__parameters:
            wildcard_nodename_root_namespace: "Wildcard for any node names, but only in root namespace"
    ```
     
```bash title="use default params value declare in code"
# run node without load any parameter file
ros2 run ros_py param_demo.py 
[INFO] [1740517785.377374134] [minimal]: P1: 1
[INFO] [1740517785.377601145] [minimal]: P1: 2
```

```bash title="load params from file"
# Run node with parameter file
ros2 run ros_py param_demo.py --ros-args --params-file params1.yaml
[INFO] [1740517874.809880531] [minimal]: P1: 10
[INFO] [1740517874.810112715] [minimal]: P1: 20
```

##### load multiple params file
!!! note "wildcard matching"
    params2.yaml has global match `/**`
     
```bash title="load params from multiple files"
ros2 run ros_py param_demo.py \
--ros-args \
--params-file params1.yaml \
--params-file params2.yaml
[INFO] [1740518709.532923265] [minimal]: P1: 100
[INFO] [1740518709.533163666] [minimal]: P1: 200
```



---

### Load multi parameter file from launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    param_file = Path(get_package_share_directory('ros_py')).joinpath('config', 'param_demo.yaml')
    param_file_arm = Path(get_package_share_directory('ros_py')).joinpath('config', 'param_demo.arm.yaml')

    param_files = [param_file, param_file_arm]

    return LaunchDescription([
        Node(
            package='ros_py',
            executable='param_demo.py',
            name='param_demo',
            parameters=param_files,
        )
    ])
```


```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():

    use_arm_config = os.environ.get('USE_ARM_CONFIG', 'false').lower() == 'true'
    param_file = Path(get_package_share_directory('ros_py')).joinpath('config', 'param_demo.yaml')
    param_file_arm = Path(get_package_share_directory('ros_py')).joinpath('config', 'param_demo.arm.yaml')

    param_files = [param_file]
    if use_arm_config:
        param_files.append(param_file_arm)

    return LaunchDescription([
        Node(
            package='ros_py',
            executable='param_demo.py',
            name='param_demo',
            parameters=param_files,
        )
    ])
```

```bash title="usage"
USE_ARM_CONFIG=true ros2 launch ros_py param_demo_simple.launch.py
USE_ARM_CONFIG=false ros2 launch ros_py param_demo_simple.launch.py

# or using export
```

---

## Reference
- [Passing ROS arguments to nodes via the command-line](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)
- [Using the ros2 param command-line tool](https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param.html)