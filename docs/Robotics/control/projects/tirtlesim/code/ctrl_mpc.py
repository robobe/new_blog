#!/usr/bin/env python3

"""
Robust MPC Controller using linearized dynamics
This version is more stable and less likely to fail
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from turtlesim.msg import Pose
import tf_transformations
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.linalg import block_diag
from scipy.optimize import minimize
import time

# Topics
VEL_TOPIC = "/turtle1/cmd_vel"
POSE_TOPIC = "/turtle1/pose"
GOAL_POSE_TOPIC = "/goal_pose"

# Control parameters
CONTROL_RATE = 10.0  # Hz
POS_OFFSET = 5.544444561004639  # meters - turtlesim center

# MPC parameters
N = 10  # Prediction horizon (reduced for faster computation)
DT = 0.1  # Time step

# Constraints
V_MAX = 1.5  # Max linear velocity (m/s) - reduced for stability
W_MAX = 1.5  # Max angular velocity (rad/s) - reduced for stability

np.set_printoptions(precision=3, suppress=True)

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

class RobustMPCController(Node):
    def __init__(self):
        super().__init__('robust_mpc_controller')
        
        # TF broadcaster
        self.br = TransformBroadcaster(self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, POSE_TOPIC, self.pose_callback, 10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped, GOAL_POSE_TOPIC, self.goal_callback, 10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, VEL_TOPIC, 10)
        
        # State variables
        self.state = np.array([POS_OFFSET, POS_OFFSET, 0.0])  # [x, y, theta]
        self.goal = np.array([POS_OFFSET, POS_OFFSET, 0.0])
        
        # Previous control (for warm start)
        self.u_prev = np.zeros(2 * N)  # Flattened [v0, w0, v1, w1, ...]
        
        # MPC weights - tuned for better performance
        self.q_x = 5.0      # Position x weight
        self.q_y = 5.0      # Position y weight  
        self.q_theta = 2.0  # Orientation weight
        self.r_v = 0.1      # Linear velocity penalty
        self.r_w = 0.1      # Angular velocity penalty
        self.r_dv = 0.5     # Linear velocity change penalty
        self.r_dw = 0.5     # Angular velocity change penalty
        
        # Build weight matrices
        self.Q = np.diag([self.q_x, self.q_y, self.q_theta])
        self.R = np.diag([self.r_v, self.r_w])
        self.R_delta = np.diag([self.r_dv, self.r_dw])
        
        # Control timer
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)
        
        self.get_logger().info('Robust MPC Controller initialized')
        self.get_logger().info('Using linearized dynamics for stability')
        
    def pose_callback(self, msg: Pose):
        """Update current state from turtle pose"""
        self.state = np.array([msg.x, msg.y, msg.theta])
        self.publish_tf(msg)
        
    def publish_tf(self, msg: Pose):
        """Publish transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        
        t.transform.translation.x = msg.x - POS_OFFSET
        t.transform.translation.y = msg.y - POS_OFFSET
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)
        
    def goal_callback(self, msg: PoseStamped):
        """Set new goal"""
        x_goal = msg.pose.position.x + POS_OFFSET
        y_goal = msg.pose.position.y + POS_OFFSET
        
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        _, _, yaw_goal = tf_transformations.euler_from_quaternion(quaternion)
        
        self.goal = np.array([x_goal, y_goal, yaw_goal])
        
        # Reset previous control for new goal
        self.u_prev = np.zeros(2 * N)
        
        self.get_logger().info(f'New goal: x={x_goal:.2f}, y={y_goal:.2f}, theta={yaw_goal:.2f}')
    
    def get_linear_dynamics(self, x_ref, u_ref):
        """
        Get linearized dynamics matrices around reference point
        For x_dot = f(x, u), linearize to get:
        x_dot ≈ A @ (x - x_ref) + B @ (u - u_ref) + f(x_ref, u_ref)
        """
        theta_ref = x_ref[2]
        v_ref = u_ref[0]
        
        # Continuous time linearized A matrix
        Ac = np.array([
            [0, 0, -v_ref * np.sin(theta_ref)],
            [0, 0,  v_ref * np.cos(theta_ref)],
            [0, 0,  0]
        ])
        
        # Continuous time B matrix
        Bc = np.array([
            [np.cos(theta_ref), 0],
            [np.sin(theta_ref), 0],
            [0,                 1]
        ])
        
        # Discretize using Euler method
        A = np.eye(3) + Ac * DT
        B = Bc * DT
        
        return A, B
    
    def mpc_cost_function(self, u_flat):
        """
        Compute MPC cost for given control sequence
        u_flat: flattened control [v0, w0, v1, w1, ...]
        """
        u = u_flat.reshape(N, 2)
        
        # Initialize cost
        cost = 0.0
        
        # Simulate trajectory
        x = self.state.copy()
        
        for k in range(N):
            # State error
            error = x - self.goal
            error[2] = normalize_angle(error[2])
            
            # State cost
            cost += error.T @ self.Q @ error
            
            # Control cost
            cost += u[k].T @ self.R @ u[k]
            
            # Control change cost (smoothness)
            if k > 0:
                du = u[k] - u[k-1]
                cost += du.T @ self.R_delta @ du
            
            # Predict next state (using nonlinear dynamics)
            x = self.dynamics_step(x, u[k])
        
        # Terminal cost (higher weight)
        error = x - self.goal
        error[2] = normalize_angle(error[2])
        cost += 5 * error.T @ self.Q @ error
        
        return cost
    
    def dynamics_step(self, x, u):
        """Single step of nonlinear dynamics"""
        x_next = np.array([
            x[0] + u[0] * np.cos(x[2]) * DT,
            x[1] + u[0] * np.sin(x[2]) * DT,
            x[2] + u[1] * DT
        ])
        x_next[2] = normalize_angle(x_next[2])
        return x_next
    
    def solve_mpc(self):
        """Solve MPC optimization problem"""
        # Bounds for controls
        bounds = []
        for _ in range(N):
            bounds.extend([
                (-V_MAX, V_MAX),  # v bounds
                (-W_MAX, W_MAX)   # w bounds
            ])
        
        # Initial guess (use previous solution shifted by one time step)
        u0 = np.roll(self.u_prev, -2)
        u0[-2:] = 0  # Zero for the new last control
        
        # Options for optimizer
        options = {
            'maxiter': 50,  # Limit iterations for speed
            'ftol': 1e-4,
            'disp': False
        }
        
        # Solve optimization
        try:
            result = minimize(
                fun=self.mpc_cost_function,
                x0=u0,
                bounds=bounds,
                method='L-BFGS-B',  # Faster than SLSQP for this problem
                options=options
            )
            
            if result.success or result.nit > 0:  # Accept partial solutions
                self.u_prev = result.x
                u_opt = result.x.reshape(N, 2)
                return u_opt[0]  # Return first control
            else:
                self.get_logger().warn('MPC failed, using P-controller')
                return self.p_controller()
                
        except Exception as e:
            self.get_logger().error(f'MPC exception: {e}')
            return self.p_controller()
    
    def p_controller(self):
        """Simple proportional controller as fallback"""
        # Compute errors
        dx = self.goal[0] - self.state[0]
        dy = self.goal[1] - self.state[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Angle to goal
        angle_to_goal = np.arctan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - self.state[2])
        
        # Control logic
        if distance < 0.1:
            # At goal position, just rotate to final orientation
            final_angle_error = normalize_angle(self.goal[2] - self.state[2])
            v = 0.0
            w = np.clip(2.0 * final_angle_error, -W_MAX, W_MAX)
        elif abs(angle_error) > 0.5:
            # Need to turn towards goal
            v = 0.1  # Slow forward motion
            w = np.clip(2.0 * angle_error, -W_MAX, W_MAX)
        else:
            # Move towards goal
            v = np.clip(0.8 * distance, 0, V_MAX)
            w = np.clip(1.0 * angle_error, -W_MAX, W_MAX)
        
        return np.array([v, w])
    
    def control_loop(self):
        """Main control loop"""
        # Check if at goal
        position_error = np.linalg.norm(self.state[:2] - self.goal[:2])
        orientation_error = abs(normalize_angle(self.state[2] - self.goal[2]))
        
        if position_error < 0.05 and orientation_error < 0.05:
            # Stop at goal
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        # Solve MPC
        start_time = time.time()
        u_opt = self.solve_mpc()
        solve_time = (time.time() - start_time) * 1000
        
        # Publish control
        cmd = Twist()
        cmd.linear.x = float(u_opt[0])
        cmd.angular.z = float(u_opt[1])
        self.cmd_pub.publish(cmd)
        
        # Log status
        self.get_logger().info(
            f'State: [{self.state[0]:.2f}, {self.state[1]:.2f}, {self.state[2]:.2f}] | '
            f'Goal: [{self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f}] | '
            f'Error: pos={position_error:.3f}m, ang={orientation_error:.3f}rad | '
            f'Control: v={u_opt[0]:.2f}, w={u_opt[1]:.2f} | '
            f'Solve: {solve_time:.1f}ms'
        )

def main(args=None):
    rclpy.init(args=args)
    
    controller = RobustMPCController()
    
    # Log parameter info
    controller.get_logger().info('='*50)
    controller.get_logger().info('MPC Parameters:')
    controller.get_logger().info(f'  Horizon: {N} steps')
    controller.get_logger().info(f'  Time step: {DT} seconds')
    controller.get_logger().info(f'  Max velocity: {V_MAX} m/s')
    controller.get_logger().info(f'  Max angular velocity: {W_MAX} rad/s')
    controller.get_logger().info('Weights (tune these in code):')
    controller.get_logger().info(f'  Position (q_x, q_y): {controller.q_x}, {controller.q_y}')
    controller.get_logger().info(f'  Orientation (q_theta): {controller.q_theta}')
    controller.get_logger().info(f'  Control effort (r_v, r_w): {controller.r_v}, {controller.r_w}')
    controller.get_logger().info('='*50)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        stop_cmd = Twist()
        controller.cmd_pub.publish(stop_cmd)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()