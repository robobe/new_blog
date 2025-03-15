#!/usr/bin/env python3

# rclpy issues 940
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration  # Correct import
from rclpy.qos_event import SubscriptionEventCallbacks

TOPIC = "test_topic"

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
        qos_profile.lifespan = Duration(seconds=1, nanoseconds=0)  # Message lifespan
        qos_profile.deadline = Duration(seconds=1, nanoseconds=0)  # Deadline duration
        self.sub = self.create_subscription(String, TOPIC, self.callback, qos_profile, event_callbacks=SubscriptionEventCallbacks(deadline=self.deadline_missed_callback))
        self.get_logger().info("Hello SUB")

    def callback(self, msg):
        self.get_logger().info(msg.data)

    def deadline_missed_callback(self, event):
        self.get_logger().warning("Deadline missed !!")

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()