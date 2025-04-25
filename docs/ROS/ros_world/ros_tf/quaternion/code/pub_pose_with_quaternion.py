#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi

class MyNode(Node):
    def __init__(self):
        node_name="pose_demo"
        super().__init__(node_name)
        self.pub = self.create_publisher(PoseStamped, "/my_topic", 10)
        self.my_timer = self.create_timer(1.0, self.__timer_handler)
        
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, pi/2, pi/2)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()