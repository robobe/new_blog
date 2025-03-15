#!/bin/usr/env python

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot_1'
        t.child_frame_id = 'world'
        t.transform.translation.x = 2.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0

        # Set Rotation (Quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.7071  # 90° rotation around Z
        t.transform.rotation.w = 0.7071

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform")

rclpy.init()
node = StaticTFPublisher()
rclpy.spin(node)
rclpy.shutdown()
