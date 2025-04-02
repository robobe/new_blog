#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_updater import Heartbeat, Updater

class MyNode(Node):
    def __init__(self):
        node_name="node_name"
        super().__init__(node_name)
        updater = Updater(self)
        updater.hwid = "hwid"
        self.task = Heartbeat()
        updater.add(self.task)
        self.get_logger().info("Hello ROS2")

    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()