#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

NAME = "param_demo"

class ParamDemo(Node):
    def __init__(self):
        super().__init__(NAME)
        self.param1 = self.declare_parameter("camera.resolution.width", 640)
        self.t = self.create_timer(1.0, self.timer_handler)

    def timer_handler(self):
        self.get_logger().info(f"{self.get_parameter('camera.resolution.width').value}")

def main(args=None):
    rclpy.init(args=args)
    node = ParamDemo()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()