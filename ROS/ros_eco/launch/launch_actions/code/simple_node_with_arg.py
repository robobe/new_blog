#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys

class SimpleNode(Node):
    def __init__(self, node_name, cli_arg):
        super().__init__(node_name)
        self.cli_arg = cli_arg
        self.get_logger().info(f"Node started with argument: {self.cli_arg}")

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Check if a command-line argument was provided
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> <node_name> <argument>")
        sys.exit(1)

    # Extract the argument (sys.argv[0] is the script name)
    cli_arg = sys.argv[1]

    # Create the node
    node = SimpleNode("simple_node", cli_arg)

    try:
        # Spin the node to keep it alive
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()