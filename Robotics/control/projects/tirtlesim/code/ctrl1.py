#!/usr/bin/env python3

# ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 10.0, y: 10.0, theta: 1.57}"
# ros2 service call /reset std_srvs/srv/Empty "{}"

#  ^ Y
#  |
#  |     (11, 11)•  top-right corner
#  |
#  |      
#  |
#  +--------------------> X
# (0, 0)
# bottom-left


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf_transformations  # or use math for yaw→quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

VEL_TOPIC = "/turtle1/cmd_vel"
POSE_TOPIC = "/turtle1/pose"
GOAL_POSE_TOPIC = "/goal_pose"

class Controller():
    def __init__(self):
        self.kv = 0.5
        self.k_theta = 4
        self.x_ref = 0.0
        self.y_ref = 0.0

    def compute_control(self, pos_x, pos_y, yaw):
        dx = self.x_ref - pos_x
        dy = self.y_ref - pos_y

        theta_ref = math.atan2(dy, dx)
        d_theta = theta_ref - yaw

        v_theta = self.k_theta * d_theta
        vx = self.kv * math.sqrt(dx**2 + dy**2)
        return vx, v_theta

    def set_points(self, x_ref, y_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.br = TransformBroadcaster(self)
        # Create subscriber
        self.subscription = self.create_subscription(
            Pose,
            POSE_TOPIC,
            self.listener_callback,
            10
        )
        
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            GOAL_POSE_TOPIC,
            self.goal_pose_callback,
            10
        )
        # Create publisher
        self.publisher = self.create_publisher(
            Twist,
            VEL_TOPIC,
            10
        )
        
        self.controller = Controller()
        self.controller.set_points(11.0/2, 11.0/2)  # Set desired reference points
        self.timer = self.create_timer(0.1, self.control_loop)  # Timer to keep node alive
        self.get_logger().info('Twist Publisher Node started')

    def pub_tf(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'

        translation_matrix = np.array([
            [1, 0, -5.5],
            [0, 1, -5.5],
            [0, 0, 1]
        ])

        # Create homogeneous coordinates for the goal position
        world_coords = np.array([msg.x, msg.y, 1])

        # Apply transformation
        turtle_coords = translation_matrix @ world_coords

        x = turtle_coords[0]
        y = turtle_coords[1]

        # Position
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Orientation from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.br.sendTransform(t)

    def control_loop(self):
        vx, v_theta = self.controller.compute_control(self.pos_x, self.pos_y, self.yaw)

        # Create Twist message
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = v_theta

        # Publish the twist message
        self.publisher.publish(twist)
        # self.get_logger().info(f'Published twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    def listener_callback(self, msg: Pose):
        # Create Twist message
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.yaw = msg.theta
        self.pub_tf(msg)

    def goal_pose_callback(self, msg: PoseStamped):
        x_ref = msg.pose.position.x
        y_ref = msg.pose.position.y
        # Translate from world coordinates (0,0 at center) to turtle coordinates (0,0 at bottom-left)
        # World center (0,0) corresponds to turtle coordinates (5.5, 5.5)
        # x_ref = msg.pose.position.x + 5.5
        # y_ref = msg.pose.position.y + 5.5
        # Create translation matrix for coordinate transformation
        translation_matrix = np.array([
            [1, 0, 5.5],
            [0, 1, 5.5],
            [0, 0, 1]
        ])

        # Create homogeneous coordinates for the goal position
        world_coords = np.array([msg.pose.position.x, msg.pose.position.y, 1])

        # Apply transformation
        turtle_coords = translation_matrix @ world_coords

        x_ref = turtle_coords[0]
        y_ref = turtle_coords[1]
        self.controller.set_points(x_ref, y_ref)
        self.get_logger().info(f'Set new goal pose: x={x_ref}, y={y_ref}')

def main(args=None):
    rclpy.init(args=args)
    
    twist_publisher = TwistPublisher()
    
    try:
        rclpy.spin(twist_publisher)
    except KeyboardInterrupt:
        pass
    
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()