#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import click

class ClickNode(Node):
    def __init__(self, message, rate):
        super().__init__("click_node")
        self.message = message
        # Declare a parameter for rate (optional ROS 2 integration)
        self.declare_parameter("rate", rate)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        # Timer to log periodically
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info(f"Node started with message: '{self.message}', rate: {self.rate} Hz")

    def timer_callback(self):
        self.get_logger().info(f"Message: {self.message}")

@click.command()
@click.argument("message", type=str)  # Required argument
@click.option("--rate", "-r", default=1.0, type=float, help="Rate in Hz to log the message")
def main(message, rate):
    """A simple ROS 2 node that logs a message at a specified rate."""
    # Initialize ROS 2
    rclpy.init()

    # Create the node with arguments from click
    node = ClickNode(message, rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()  # Click handles the CLI parsing