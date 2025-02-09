---
tags:
    - ros
    - packages
    - message_filters
---

# Message Filters
Message_filters is a collection of message "filters" which take messages in. and may or may not output the message at some time in the future, depending on a policy defined for that filter. [more](https://github.com/ros2/message_filters)

[Tutorials](https://github.com/ros2/message_filters/tree/rolling/doc/Tutorials)

## TimeSynchronizer demo
TimeSynchronizer listens on multiple input message and invokes the callback when
it has a collection of messages with matching timestamps.

<details>
<summary>TimeSynchronizer</summary>
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import message_filters
from geometry_msgs.msg import PointStamped

class MyNode(Node):
    def __init__(self):
        node_name="TimeSynchronizer_demo"
        super().__init__(node_name)
        self.get_logger().info("Hello ROS2")
        self.pub1 = self.create_publisher(PointStamped, "topic1", 10)
        self.pub2 = self.create_publisher(PointStamped, "topic2", 10)

        sub1 = message_filters.Subscriber(self, PointStamped, "topic1")
        sub2 = message_filters.Subscriber(self, PointStamped, "topic2")

        ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
        ts.registerCallback(self.cb)
        self.counter = 0
        self.t1 = self.create_timer(1.0, self.timer_cb)
    
    def cb(self, point1: PointStamped, point2: PointStamped):
        self.get_logger().info(f"----{Time.from_msg(point1.header.stamp)}")

    def timer_cb(self):
        self.counter += 1
        point = PointStamped()
        sync_time = self.get_clock().now().to_msg()
        point.header.stamp = sync_time
        point.point.x = 1.0
        self.pub1.publish(point)
        if not (self.counter % 2 == 0):
            point2 = PointStamped()
            point2.header.stamp = sync_time
            point2.point.x = 2.0
            self.pub2.publish(point2)


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

The demo publish two PointStamped messages with the same timestamp 
- one message is published every second 
- other every two seconds.
The TimeSynchronizer will invoke the callback when it has both in the same time , meaning it publish every two seconds.


---

## ApproximateTimeSynchronizer demo
Like the TimeSynchronizer demo, but with a time window. The callback will be invoked when the timestamps of the messages are within the time window. the slop argument is the time window in seconds.

<details>
<summary>ApproximateTimeSynchronizer</summary>
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import time
import message_filters
from geometry_msgs.msg import PointStamped

class MyNode(Node):
    def __init__(self):
        node_name="ApproximateTimeSynchronizer_demo"
        super().__init__(node_name)
        self.get_logger().info("Hello ROS2")
        self.pub1 = self.create_publisher(PointStamped, "topic1", 10)
        self.pub2 = self.create_publisher(PointStamped, "topic2", 10)

        sub1 = message_filters.Subscriber(self, PointStamped, "topic1")
        sub2 = message_filters.Subscriber(self, PointStamped, "topic2")

        ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, slop=0.2)
        ts.registerCallback(self.cb)
        self.counter = 0
        self.t1 = self.create_timer(1.0, self.timer_cb)
    
    def cb(self, point1: PointStamped, point2: PointStamped):
        self.get_logger().info(f"----{Time.from_msg(point1.header.stamp)}")

    def timer_cb(self):
        self.counter += 1
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = 1.0
        self.pub1.publish(point)
        time.sleep(0.1)
        point2 = PointStamped()
        point2.header.stamp = self.get_clock().now().to_msg()
        point2.point.x = 2.0
        self.pub2.publish(point2)


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

The demo publish two PointStamped messages with timestamps that are within 0.1 seconds of each other

The ApproximateTimeSynchronizer will invoke the callback when it has both in the same time window, meaning it publish every second.