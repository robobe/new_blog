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
CONTROL_RATE = 10.0  # Hz

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0.0
        self.integral = 0.0
        self.yaw = 0.0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

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
        
        self.vx_pid = PID(Kp=0.3, Ki=0.0, Kd=0.0)
        self.vtheta_pid = PID(Kp=4.0, Ki=0.0, Kd=0.0)

        
        self.x_ref, self.y_ref = 5.544444561004639, 5.544444561004639  # Initial goal at center
        self.pos_x, self.pos_y = 5.544444561004639, 5.544444561004639  # Initial goal at center
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)  # Timer to keep node alive
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
        dx = self.x_ref - self.pos_x
        dy = self.y_ref - self.pos_y

        theta_ref = math.atan2(dy, dx)
        d_theta = theta_ref - self.yaw
        # print(d_theta, theta_ref, self.yaw, dy, dx)
        vx = self.vx_pid.compute(math.sqrt(dx**2 + dy**2), 1.0 / CONTROL_RATE)
        v_theta = self.vtheta_pid.compute(d_theta, 1.0 / CONTROL_RATE) 
        v_theta = v_theta * -1.0
        vx = vx * -1.0
        self.get_logger().info(f'Control loop: pos=({self.pos_x}, {self.pos_y:.2f}), goal=({self.x_ref:.2f}, {self.y_ref:.2f}), vx={vx:.2f}, v_theta={v_theta:.2f}')    
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

        self.x_ref = turtle_coords[0]
        self.y_ref = turtle_coords[1]
        self.get_logger().info(f'Set new goal pose: x={self.x_ref}, y={self.y_ref}')

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