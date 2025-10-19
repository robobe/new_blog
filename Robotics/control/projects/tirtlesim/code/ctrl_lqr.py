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
POS_OFFSET = 5.544444561004639  # meters

np.set_printoptions(precision=3,suppress=True)
 
# Optional Variables
MAX_LINEAR_VELOCITY = 2.0 # meters per second
MAX_ANGULAR_VELOCITY = 1.5708 # radians per second


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt):
    """
    Discrete-time linear quadratic regulator for a nonlinear system.
 
    Compute the optimal control inputs given a nonlinear system, cost matrices, 
    current state, and a final state.
     
    Compute the control variables that minimize the cumulative cost.
 
    Solve for P using the dynamic programming method.
 
    :param actual_state_x: The current state of the system 
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]
    :param desired_state_xf: The desired state of the system
        3x1 NumPy Array given the state is [x,y,yaw angle] --->
        [meters, meters, radians]   
    :param Q: The state cost matrix
        3x3 NumPy Array
    :param R: The input cost matrix
        2x2 NumPy Array
    :param dt: The size of the timestep in seconds -> float
 
    :return: u_star: Optimal action u for the current state 
        2x1 NumPy Array given the control input vector is
        [linear velocity of the car, angular velocity of the car]
        [meters per second, radians per second]
    """
    # We want the system to stabilize at desired_state_xf.
    x_error = actual_state_x - desired_state_xf
 
    # Solutions to discrete LQR problems are obtained using the dynamic 
    # programming method.
    # The optimal solution is obtained recursively, starting at the last 
    # timestep and working backwards.
    # You can play with this number
    N = 50
 
    # Create a list of N + 1 elements
    P = [None] * (N + 1)
     
    Qf = Q
 
    # LQR via Dynamic Programming
    P[N] = Qf
 
    # For i = N, ..., 1
    for i in range(N, 0, -1):
 
        # Discrete-time Algebraic Riccati equation to calculate the optimal 
        # state cost matrix
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
            R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
 
    # Create a list of N elements
    K = [None] * N
    u = [None] * N
 
    # For i = 0, ..., N - 1
    for i in range(N):
 
        # Calculate the optimal feedback gain K
        K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
 
        u[i] = K[i] @ x_error
 
    # Optimal control input is u_star
    u_star = u[N-1]
 
    return u_star

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
        

        self.final_orientation = 0.0
        self.dt = 1.0 / CONTROL_RATE
        self.x_ref, self.y_ref = POS_OFFSET, POS_OFFSET  # Initial goal at center
        self.pos_x, self.pos_y = POS_OFFSET, POS_OFFSET  # Initial goal at center
        self.actual_state_x = np.array([self.pos_x, self.pos_y,0])
        self.desired_state_xf = np.array([self.x_ref, self.y_ref, 0])
        # State transition matrix A (discrete time)
        self.A = np.eye(3)
        
        # Input cost matrix R
        self.R = np.array([[0.1, 0.0],    # Penalty for linear velocity
                          [0.0, 0.1]])     # Penalty for angular velocity
        
        # Default state cost matrix Q
        self.Q_position = np.array([[2.0, 0.0, 0.0],   # High penalty for X position
                                    [0.0, 2.0, 0.0],    # High penalty for Y position
                                    [0.0, 0.0, 0.5]])   # Low penalty for heading
        
        self.Q_orientation = np.array([[0.1, 0.0, 0.0],   # Low penalty for X position
                                       [0.0, 0.1, 0.0],    # Low penalty for Y position
                                       [0.0, 0.0, 5.0]])   # High penalty for heading
        
        # Control mode: 'position_first' or 'orientation_only'
        self.control_mode = 'position_first'
        
        # Tolerances
        self.position_tolerance = 0.05  # meters
        self.orientation_tolerance = 0.1  # radians
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

    def compute_heading_to_goal(self):
        """Compute the heading angle needed to reach the goal position"""
        dx = self.desired_state_xf[0] - self.actual_state_x[0]
        dy = self.desired_state_xf[1] - self.actual_state_x[1]
        return np.arctan2(dy, dx)
    
    def control_loop(self):
        """Main control loop with two-phase strategy"""
        # Calculate position and orientation errors
        position_error = np.linalg.norm(self.actual_state_x[:2] - self.desired_state_xf[:2])
        orientation_error = abs(normalize_angle(self.actual_state_x[2] - self.final_orientation))
        
        # Check if we've reached the goal
        if position_error < self.position_tolerance and orientation_error < self.orientation_tolerance:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            return
        
        # Two-phase control strategy
        if self.control_mode == 'position_first':
            if position_error > self.position_tolerance:
                # Phase 1: Move to goal position
                # Calculate heading to goal
                heading_to_goal = self.compute_heading_to_goal()
                
                # Create intermediate state with heading pointing to goal
                intermediate_state = np.array([
                    self.desired_state_xf[0],
                    self.desired_state_xf[1],
                    heading_to_goal  # Point towards goal instead of final orientation
                ])
                
                # Use position-focused Q matrix
                Q = self.Q_position
                
                # Get B matrix
                B = self.getB(self.actual_state_x[2], self.dt)
                
                # Compute control
                optimal_control = lqr(
                    self.actual_state_x,
                    intermediate_state,
                    Q,
                    self.R,
                    self.A,
                    B,
                    self.dt
                )
                
                self.get_logger().info(f'Phase 1: Position control, error={position_error:.3f}m')
            else:
                self.get_logger().warn('Position reached within tolerance.')
                # Switch to orientation control
                self.control_mode = 'orientation_only'
                self.get_logger().info('Switching to Phase 2: Orientation control')
        
        if self.control_mode == 'orientation_only':
            if position_error > self.position_tolerance:
                self.get_logger().warn('Position error exceeded tolerance during orientation phase. Switching back to Phase 1.')
                self.control_mode = 'position_first'
                return
            # Phase 2: Rotate to final orientation
            # Only care about orientation now
            Q = self.Q_orientation
            
            # Get B matrix
            B = self.getB(self.actual_state_x[2], self.dt)
            
            # Compute control focusing on orientation
            optimal_control = lqr(
                self.actual_state_x,
                self.desired_state_xf,
                Q,
                self.R,
                self.A,
                B,
                self.dt
            )
            
            # Reduce linear velocity in this phase to maintain position
            optimal_control[0] *= 0.1  # Significantly reduce linear velocity
            
            self.get_logger().info(f'Phase 2: Orientation control, error={orientation_error:.3f}rad')
        
        # Extract and limit control inputs
        v_cmd = np.clip(optimal_control[0], -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        w_cmd = np.clip(optimal_control[1], -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        
        # Create and publish twist message
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.publisher.publish(twist)
        
        # Debug output
        self.get_logger().debug(
            f'Pos error: {position_error:.3f}m | '
            f'Orient error: {orientation_error:.3f}rad | '
            f'Control: v={v_cmd:.2f}, w={w_cmd:.2f} | '
            f'Mode: {self.control_mode}'
        )

    def listener_callback(self, msg: Pose):
        # Create Twist message
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.yaw = msg.theta
        # Update the actual state with real turtle position
        self.actual_state_x = np.array([self.pos_x, self.pos_y, self.yaw])
        
        self.pub_tf(msg)

    def goal_pose_callback(self, msg: PoseStamped):
        # Translate from world coordinates (0,0 at center) to turtle coordinates (0,0 at bottom-left)
        # World center (0,0) corresponds to turtle coordinates (5.5, 5.5)
        x_ref = msg.pose.position.x + POS_OFFSET
        y_ref = msg.pose.position.y + POS_OFFSET
        # Create translation matrix for coordinate transformation
        # translation_matrix = np.array([
        #     [1, 0, POS_OFFSET],
        #     [0, 1, POS_OFFSET],
        #     [0, 0, 1]
        # ])

        # # Create homogeneous coordinates for the goal position
        # world_coords = np.array([msg.pose.position.x, msg.pose.position.y, 1])

        # # Apply transformation
        # turtle_coords = translation_matrix @ world_coords

        # self.x_ref = turtle_coords[0]
        # self.y_ref = turtle_coords[1]
        self.desired_state_xf = np.array([x_ref, y_ref, self.final_orientation])
        self.control_mode == 'position_first'
        self.get_logger().info(f'Set new goal pose: x={x_ref}, y={y_ref}')

    def getB(self, yaw, dt):
        """
        Calculates and returns the B matrix for differential drive robot
        3x2 matix ---> number of states x number of control inputs
    
        For differential drive: 
        - Linear velocity (v) moves robot forward in its heading direction
        - Angular velocity (w) rotates the robot
        
        :param yaw: The yaw angle (rotation angle around the z axis) in radians 
        :param deltat: The change in time from timestep t-1 to t in seconds
        
        :return: B matrix ---> 3x2 NumPy array
        """
        B = np.array([  [np.cos(yaw)*dt, 0],
                        [np.sin(yaw)*dt, 0],
                        [0, dt]])
        return B
    
    def state_space_model(self, A, state_t_minus_1, B, control_input_t_minus_1):
        """
        Calculates the state at time t given the state at time t-1 and
        the control inputs applied at time t-1
        
        :param: A   The A state transition matrix
            3x3 NumPy Array
        :param: state_t_minus_1     The state at time t-1  
            3x1 NumPy Array given the state is [x,y,yaw angle] ---> 
            [meters, meters, radians]
        :param: B   The B state transition matrix
            3x2 NumPy Array
        :param: control_input_t_minus_1     Optimal control inputs at time t-1  
            2x1 NumPy Array given the control input vector is 
            [linear velocity of the car, angular velocity of the car]
            [meters per second, radians per second]
            
        :return: State estimate at time t
            3x1 NumPy Array given the state is [x,y,yaw angle] --->
            [meters, meters, radians]
        """
        # These next 6 lines of code which place limits on the angular and linear 
        # velocities of the robot car can be removed if you desire.
        control_input_t_minus_1[0] = np.clip(control_input_t_minus_1[0],
                                                                                -MAX_LINEAR_VELOCITY,
                                                                                MAX_LINEAR_VELOCITY)
        control_input_t_minus_1[1] = np.clip(control_input_t_minus_1[1],
                                                                                -MAX_ANGULAR_VELOCITY,
                                                                                MAX_ANGULAR_VELOCITY)
        state_estimate_t = (A @ state_t_minus_1) + (B @ control_input_t_minus_1)

        return state_estimate_t




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