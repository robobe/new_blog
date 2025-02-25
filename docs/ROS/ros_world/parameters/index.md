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

<details><summary>Simple node</summary>
```python
#!/usr/bin/env python3


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
/*/*:
  ros__parameters:
    param1: 100
    param2: 200
```