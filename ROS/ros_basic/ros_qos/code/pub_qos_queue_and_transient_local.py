#!/usr/bin/env python3


import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.duration import Duration  # Correct import

TOPIC = "test_topic"

SUB_START_DELAY = 5
PUB_RATE = 1

class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")
        self.sub_start_delay = self.create_timer(SUB_START_DELAY, self.timer_handler)
        
        self.get_logger().info("Hello SUB")

    def timer_handler(self):
        self.destroy_timer(self.sub_start_delay)
        self.get_logger().info("--Create subscriber--")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # or VOLATILE
            history=HistoryPolicy.KEEP_LAST,  # or KEEP_ALL
            depth=10,  # Queue depth
            liveliness=LivelinessPolicy.AUTOMATIC,  # or MANUAL_BY_TOPIC
        )
        self.create_subscription(String, TOPIC, self.callback, qos_profile=qos_profile)


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
        self.counter = 0
        self.pub = self.create_publisher(String, TOPIC, qos_profile)
        # self.get_logger().info(f"{self.pub.qos_profile}")
        self.timer = self.create_timer(PUB_RATE, self.timer_handler)
        
        self.get_logger().info("Hello PUB")

    def timer_handler(self):
        msg = String()
        msg.data = f"hello {self.counter}"
        self.counter += 1
        self.pub.publish(msg)

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