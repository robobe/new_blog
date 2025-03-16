#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, LivelinessPolicy
from rclpy.duration import Duration

class LivelinessPublisher(Node):
    def __init__(self):
        super().__init__('liveliness_publisher')
        
        qos_profile = QoSProfile(
            depth=10,
            liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,  # Publisher must assert liveliness
            liveliness_lease_duration=Duration(seconds=3)  # Must assert within 3 seconds
        )

        self.publisher_ = self.create_publisher(String, 'liveliness_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every 1s
        # self.liveliness_timer = self.create_timer(2.0, self.assert_liveliness)  # Assert liveliness every 2s
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

    def assert_liveliness(self):
        self.publisher_.assert_liveliness()  # Manually assert that publisher is alive
        self.get_logger().info('Liveliness asserted')

def main(args=None):
    rclpy.init(args=args)
    node = LivelinessPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
