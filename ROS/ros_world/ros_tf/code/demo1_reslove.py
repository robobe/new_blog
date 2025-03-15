#!/bin/usr/env python

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFResolver(Node):
    def __init__(self):
        super().__init__('tf_resolver')

        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to check TF every 1 second
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        from_frame = 'robot_1'
        to_frame = 'world'

        try:
            # Lookup transform from 'world' to 'robot_1'
            transform: TransformStamped = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())

            # Extract translation
            translation = transform.transform.translation
            self.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")

            # Extract rotation (quaternion)
            rotation = transform.transform.rotation
            self.get_logger().info(f"Rotation (quaternion): x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        except tf2_ros.LookupException:
            self.get_logger().warn(f"Transform from {from_frame} to {to_frame} not found!")
        except tf2_ros.ConnectivityException:
            self.get_logger().warn("Connectivity issue!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("Extrapolation error!")

def main():
    rclpy.init()
    node = TFResolver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
