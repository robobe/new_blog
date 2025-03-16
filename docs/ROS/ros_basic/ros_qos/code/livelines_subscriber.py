#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, LivelinessPolicy
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.duration import Duration

class LivelinessSubscriber(Node):
    def __init__(self):
        super().__init__('liveliness_subscriber')

        qos_profile = QoSProfile(
            depth=10,
            liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
            liveliness_lease_duration=Duration(seconds=3)  # Expecting liveliness assertion every 3s
        )

        event_callbacks = SubscriptionEventCallbacks(
            liveliness=self.liveliness_callback
        )

        self.subscription = self.create_subscription(
            String,
            'liveliness_topic',
            self.listener_callback,
            qos_profile,
            event_callbacks=event_callbacks
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def liveliness_callback(self, event):
        if event.alive_count == 0:
            self.get_logger().warn('Liveliness lost! Publisher is inactive.')
        else:
            self.get_logger().info(f'Liveliness changed. Active publishers: {event.alive_count}')

def main(args=None):
    rclpy.init(args=args)
    node = LivelinessSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
