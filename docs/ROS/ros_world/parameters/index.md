---
tags:
    - ros2
    - parameters
---

# ROS Parameters

## parameter file

### Demo:
Simple Node with multiple parameter file
one of the file use wildcard as `node_name`

<details><summary>Post</summary>


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

```yaml title="parameters1.yaml"
minimal:
  ros__parameters:
    param1: 10
    param2: 20
```

```yaml title="parameters2.yaml"
/**:
  ros__parameters:
    param1: 100
    param2: 200
```

---

#### usage

!!! note wildcard
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
ros2 run ros_py param_demo.py 
[INFO] [1740517785.377374134] [minimal]: P1: 1
[INFO] [1740517785.377601145] [minimal]: P1: 2
```

```bash title="load paras from file"
ros2 run ros_py param_demo.py --ros-args --params-file /home/user/workspaces/ros_py/src/ros_py/ros_py/params.yaml
[INFO] [1740517874.809880531] [minimal]: P1: 10
[INFO] [1740517874.810112715] [minimal]: P1: 20
```

```bash
ros2 run ros_py param_demo.py \
--ros-args \
--params-file /home/user/workspaces/ros_py/src/ros_py/ros_py/params2.yaml \
--params-file /home/user/workspaces/ros_py/src/ros_py/ros_py/params.yaml
[INFO] [1740518709.532923265] [minimal]: P1: 100
[INFO] [1740518709.533163666] [minimal]: P1: 200
```


---

## Reference
- [Passing ROS arguments to nodes via the command-line](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)
- [Using the ros2 param command-line tool](https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param.html)