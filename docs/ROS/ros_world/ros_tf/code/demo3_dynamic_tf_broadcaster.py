#!/bin/usr/env python

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

HZ = 0.1

class MovingTFPublisher(Node):
    def __init__(self):
        super().__init__('moving_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(HZ, self.publish_transform)
        self.t = 0  # Time step

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot_1'
        t.child_frame_id = 'world'

        # Simulate movement: circular motion
        t.transform.translation.x = 2.0 * math.cos(self.t)
        t.transform.translation.y = 2.0 * math.sin(self.t)
        t.transform.translation.z = 0.0

        # Quaternion Rotation (yaw only)
        qz = math.sin(self.t / 2)
        qw = math.cos(self.t / 2)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.t += 0.1  # Increment time step
        self.get_logger().info("Publish tf")

rclpy.init()
node = MovingTFPublisher()
rclpy.spin(node)
rclpy.shutdown()
