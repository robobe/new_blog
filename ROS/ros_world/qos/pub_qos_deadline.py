#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration  # Correct import

TOPIC = "pub_topic"

class PubNode(Node):
    def __init__(self):
        node_name="pub"
        self.counter = 0
        super().__init__(node_name)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # or VOLATILE
            history=HistoryPolicy.KEEP_LAST,  # or KEEP_ALL
            depth=10,  # Queue depth
            liveliness=LivelinessPolicy.AUTOMATIC,  # or MANUAL_BY_TOPIC
        )
        qos_profile.lifespan = Duration(seconds=5, nanoseconds=0)  # Message lifespan
        qos_profile.deadline = Duration(seconds=2, nanoseconds=0)  # Deadline duration
        
        self.pub = self.create_publisher(String, TOPIC, qos_profile)
        self.timer = self.create_timer(2.0, self.timer_handler)
        
        self.get_logger().info("Hello PUB")

    def timer_handler(self):
        msg = String()
        msg.data = f"hello {self.counter}"
        self.counter += 1
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()