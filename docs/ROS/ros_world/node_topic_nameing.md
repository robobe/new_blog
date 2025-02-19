---
tags:
    - ros
    - topic
    - nameing
    - remap
    - namespace
---

# ROS2 Topic name rename and namespace

## remap

```
ros2 run package node --ros-args -r old_topic:=new_topic
```

### Demo
- Two nodes
  - pub publish string message in `pub_topic` 
  - sub subscribe to string message in `sub_topic`

<details><summary>pub sub code</summary>

```python title="pub node"
#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "pub_topic"

class PubNode(Node):
    def __init__(self):
        node_name="pub"
        super().__init__(node_name)
        self.pub = self.create_publisher(String, TOPIC,10)
        self.timer = self.create_timer(1.0, self.timer_handler)
        self.get_logger().info("Hello PUB")

    def timer_handler(self):
        msg = String()
        msg.data = "hello"
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python title="sub node"
#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "sub_topic"

class SubNode(Node):
    def __init__(self):
        node_name="pub"
        super().__init__(node_name)
        self.pub = self.create_subscription(String, TOPIC, self.callback, 10)
        self.get_logger().info("Hello SUB")

    def callback(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>
### usage 
- remap subscriber node

```bash title="terminal1"
ros2 run g_stream pub.py
```

```bash title="terminal2"
ros2 run g_stream sub.py --ros-args -r sub_topic:=pub_topic
```

- remap pub node

```bash title="terminal1"
ros2 run g_stream pub.py --ros-args -r pub_topic:=sub_topic
```

```bash title="terminal2"
ros2 run g_stream sub.py 
```

---

## Namespace

### change from CLI

```bash
ros2 run g_stream pub.py --ros-args -r __ns:=/ns1
```

```bash
ros2 topic list
#
/ns1/pub_topic
```