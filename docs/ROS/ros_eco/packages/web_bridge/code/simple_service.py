#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_services_default

TOPIC = "trigger_srv"

class MyNode(Node):
    def __init__(self):
        node_name="minimal_srv"
        super().__init__(node_name)
        self.srv = self.create_service(Trigger, TOPIC, self.handler, qos_profile=qos_profile_services_default)
        self.get_logger().info("Hello ROS2")

    def handler(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Trigger service called")
        response.success = True
        response.message = "success"
        print(response)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()