#!/usr/bin/env python3


import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration  # Correct import
from rclpy.qos_event import SubscriptionEventCallbacks

TOPIC = "test_topic"

SUB_START_DELAY = 5
PUB_RATE = 1


class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")
        
        self.get_logger().info("--Create subscriber--")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # or VOLATILE
            history=HistoryPolicy.KEEP_LAST,  # or KEEP_ALL
            depth=10,  # Queue depth
            liveliness=LivelinessPolicy.AUTOMATIC,  # or MANUAL_BY_TOPIC
        )
        qos_profile.deadline = Duration(seconds=1, nanoseconds=0)  # Deadline duration
        self.sub = self.create_subscription(String, TOPIC, self.callback, qos_profile, event_callbacks=SubscriptionEventCallbacks(deadline=self.deadline_missed_callback))

        self.get_logger().info("Hello SUB")


    def deadline_missed_callback(self, event):
        self.get_logger().warning("Deadline missed !!")

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

class PubNode(Node):
    def __init__(self):
        super().__init__("pub_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # or VOLATILE
            history=HistoryPolicy.KEEP_LAST,  # or KEEP_ALL
            depth=10,  # Queue depth
            liveliness=LivelinessPolicy.AUTOMATIC,  # or MANUAL_BY_TOPIC
        )
        qos_profile.deadline = Duration(seconds=1, nanoseconds=0)  # Deadline duration
        self.counter = 0
        self.pub = self.create_publisher(String, TOPIC, qos_profile)
        self.timer = self.create_timer(PUB_RATE, self.timer_handler)
        
        self.get_logger().info("Hello PUB")

    def timer_handler(self):
        msg = String()
        msg.data = f"hello {self.counter}"
        self.counter += 1
        self.pub.publish(msg)
        if self.counter > 5:
            self.destroy_timer(self.timer)
            self.destroy_publisher(self.pub)

def main(args=None):
    rclpy.init(args=args)
    pub_node = PubNode()
    sub_node = SubNode()
    executer = MultiThreadedExecutor()
    executer.add_node(pub_node)
    executer.add_node(sub_node)
    
    executer.spin()
    

    rclpy.shutdown()

if __name__ == '__main__':
    main()