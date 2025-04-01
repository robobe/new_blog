#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater

def dummy_diagnostic(stat: diagnostic_updater.DiagnosticStatusWrapper):
   stat.message ="message dummy_diagnostic"
   stat.level = DiagnosticStatus.WARN
   stat.name = "dummy_diagnostic"

   return stat

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        updater = diagnostic_updater.Updater(self)
        updater.add("Function updater", dummy_diagnostic)
        self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()