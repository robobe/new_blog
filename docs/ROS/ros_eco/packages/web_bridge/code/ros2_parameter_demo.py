#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

PARAM1 = "param1"
PARAM2 = "param2"

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        p1 = self.declare_parameter(PARAM1, 1)
        p2 = self.declare_parameter(PARAM2, 2)
        self.get_logger().info(f"P1: {p1.value}")
        self.get_logger().info(f"P1: {p2.value}")
        self.add_on_set_parameters_callback(self.param_callback)


    def param_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")

        return SetParametersResult(successful=True)
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()