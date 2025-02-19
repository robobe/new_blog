#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration  # Correct import
TOPIC = "sub_topic"

class SubNode(Node):
    def __init__(self):
        node_name="pub"
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
        self.pub = self.create_subscription(String, TOPIC, self.callback, qos_profile)
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