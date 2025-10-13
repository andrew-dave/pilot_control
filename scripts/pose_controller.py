#!/usr/bin/env python3
"""
Pose Controller Node
====================
Subscribes to Fast-LIO2 odometry and generates left/right wheel velocity commands
to drive the robot to target poses.

Features:
- Subscribes to Fast-LIO2 /Odometry for localization (x, y, yaw)
- Control loop runs at 10 Hz
- Generates left and right wheel velocity commands
- Publishes to ODrive CAN nodes

Usage:
  ros2 run pilot_control pose_controller.py

Topics:
  Subscribed:
    - /Odometry (nav_msgs/Odometry) - Fast-LIO2 localization
    - /set_target_pose (geometry_msgs/PoseStamped) - Target pose input
  Published:
    - /left/control_message (odrive_can/ControlMessage) - Left wheel velocity
    - /right/control_message (odrive_can/ControlMessage) - Right wheel velocity
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from odrive_can.msg import ControlMessage
from std_srvs.srv import Trigger
import math
import numpy as np
from typing import Tuple, Optional


class PoseController(Node):
    def __init__(self):
        super().__init__('pose_controller')
        
        # ============================================================
        # PARAMETERS
        # ============================================================
        
        # Robot kinematics (matching diff_drive_controller)
        self.declare_parameter('wheel_radius', 0.072)      # m
        self.declare_parameter('wheel_base', 0.32)         # m
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        self.declare_parameter('velocity_multiplier', 1.0)
        
        # Control parameters
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('max_linear_velocity', 0.3) # m/s
        self.declare_parameter('max_angular_velocity', 1.5) # rad/s
        self.declare_parameter('position_tolerance', 0.01) # m
        self.declare_parameter('orientation_tolerance', 0.1) # rad (~5.7 degrees)

        self.declare_parameter('Kp_linear', 30)
        self.declare_parameter('Ki_linear', 0)
        self.declare_parameter('Kd_linear', 0)
        self.declare_parameter('Kp_angular', 30)
        self.declare_parameter('Ki_angular', 0)
        self.declare_parameter('Kd_angular', 0)

        # Safety parameters
        self.declare_parameter('enable_controller', True) # Start enabled by default
        self.declare_parameter('odometry_timeout_ms', 500) # Stop if no odometry
        
        # Topic names
        self.declare_parameter('odometry_topic', '/Odometry')
        self.declare_parameter('left_control_topic', '/left/control_message')
        self.declare_parameter('right_control_topic', '/right/control_message')
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        
        self.control_freq = self.get_parameter('control_frequency').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.ori_tolerance = self.get_parameter('orientation_tolerance').value
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Ki_linear = self.get_parameter('Ki_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Ki_angular = self.get_parameter('Ki_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        
        self.controller_enabled = self.get_parameter('enable_controller').value
        self.odom_timeout_ms = self.get_parameter('odometry_timeout_ms').value
        
        odometry_topic = self.get_parameter('odometry_topic').value
        left_control_topic = self.get_parameter('left_control_topic').value
        right_control_topic = self.get_parameter('right_control_topic').value
        
        # ============================================================
        # STATE VARIABLES
        # ============================================================
        
        # Current pose from Fast-LIO2
        self.current_x = 3.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self.pose_initialized = False
        self.last_odom_time = self.get_clock().now()
        
        # Target pose
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        
        # Control outputs
        self.left_wheel_velocity = 0.0   # rad/s (motor turns/s)
        self.right_wheel_velocity = 0.0  # rad/s (motor turns/s)
        
        # ============================================================
        # ROS INTERFACES
        # ============================================================
        
        # Subscriber to Fast-LIO2 odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            10
        )
        
        # Publishers for wheel velocities
        self.left_pub = self.create_publisher(
            ControlMessage,
            left_control_topic,
            10
        )
        
        self.right_pub = self.create_publisher(
            ControlMessage,
            right_control_topic,
            10
        )
        
        # Subscriber to set target pose (PoseStamped topic)
        self.set_target_sub = self.create_subscription(
            PoseStamped,
            '/set_target_pose',
            self.set_target_callback,
            10
        )

        # Service to enable/disable controller
        self.enable_srv = self.create_service(
            Trigger,
            '/enable_controller',
            self.enable_callback
        )
        
        self.disable_srv = self.create_service(
            Trigger,
            '/disable_controller',
            self.disable_callback
        )
        
        # Control loop timer (10 Hz by default)
        control_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # ============================================================
        # LOGGING
        # ============================================================
        
        self.get_logger().info('='*70)
        self.get_logger().info('Pose Controller Node Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Robot Parameters:')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'  Wheel base: {self.wheel_base:.3f} m')
        self.get_logger().info(f'  Gear ratio: {self.gear_ratio:.2f}')
        self.get_logger().info(f'  Invert left/right: {self.invert_left}/{self.invert_right}')
        self.get_logger().info(f'')
        self.get_logger().info(f'Control Parameters:')
        self.get_logger().info(f'  Control frequency: {self.control_freq:.1f} Hz')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel:.2f} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'  Position tolerance: {self.pos_tolerance:.3f} m')
        self.get_logger().info(f'  Orientation tolerance: {self.ori_tolerance:.3f} rad')
        self.get_logger().info(f'')
        self.get_logger().info(f'Topics:')
        self.get_logger().info(f'  Odometry: {odometry_topic}')
        self.get_logger().info(f'  Left control: {left_control_topic}')
        self.get_logger().info(f'  Right control: {right_control_topic}')
        self.get_logger().info(f'')
        self.get_logger().info(f'Services:')
        self.get_logger().info(f'  /set_target_pose - Set new target pose')
        self.get_logger().info(f'  /enable_controller - Enable controller')
        self.get_logger().info(f'  /disable_controller - Disable controller')
        self.get_logger().info(f'')
        self.get_logger().info(f'Controller Status: {"ENABLED" if self.controller_enabled else "DISABLED"}')
        self.get_logger().info('='*70)
        
        if not self.controller_enabled:
            self.get_logger().info('Controller is currently disabled; it will auto-enable on first target pose.')
    
    # ============================================================
    # CALLBACK: ODOMETRY
    # ============================================================
    
    def odometry_callback(self, msg: Odometry):
        """
        Callback for Fast-LIO2 odometry messages.
        Extracts x, y, yaw from the odometry message.
        """
        # Extract position (x, y, z)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # z is available but not used for 2D control
        
        # Extract orientation (quaternion -> yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw (Euler angle around Z-axis)
        self.current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        # Extract velocities (linear and angular)
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y
        self.current_vyaw = msg.twist.twist.angular.z
        
        # Mark pose as initialized
        if not self.pose_initialized:
            self.pose_initialized = True
            self.get_logger().info(
                f'✓ Odometry initialized: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, yaw={math.degrees(self.current_yaw):.1f}°'
            )
        
        # Update last odometry time
        self.last_odom_time = self.get_clock().now()
    
    # ============================================================
    # CALLBACK: CONTROL LOOP (10 Hz)
    # ============================================================
    
    def control_loop(self):
        """
        Main control loop - runs at 10 Hz.
        1. Check if odometry is fresh
        2. Check if controller is enabled
        3. Call control function to compute velocities
        4. Publish wheel velocity commands
        """
        # Check if controller is enabled
        if not self.controller_enabled:
            # Send zero velocity when disabled
            self.publish_zero_velocity()
            return
        
        # Check if odometry is initialized
        if not self.pose_initialized:
            self.get_logger().warn_throttle(
                5.0,
                '⚠️  Waiting for odometry from Fast-LIO2...'
            )
            self.publish_zero_velocity()
            return
        
        # Check odometry timeout
        time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e6
        if time_since_odom > self.odom_timeout_ms:
            self.get_logger().warn_throttle(
                2.0,
                f'⚠️  Odometry timeout ({time_since_odom:.1f} ms > {self.odom_timeout_ms} ms). '
                'Stopping motors.'
            )
            self.publish_zero_velocity()
            return
        
        # Check if target is set
        if not self.has_target:
            # No target - hold position (send zero velocity)
            self.publish_zero_velocity()
            return
        
        # ============================================================
        # CALL CONTROL FUNCTION
        # ============================================================
        
        linear_vel, angular_vel = self.control_function(
            self.current_x, self.current_y, self.current_yaw,
            self.target_x, self.target_y, self.target_yaw
        )
        
        # ============================================================
        # VELOCITY RESCALING 
        # ============================================================
        # Scale down velocities proportionally if either exceeds limits
        linear_scale = abs(linear_vel) / self.max_linear_vel if self.max_linear_vel > 0 else 0
        angular_scale = abs(angular_vel) / self.max_angular_vel if self.max_angular_vel > 0 else 0
        vel_factor = max(linear_scale, angular_scale, 1.0)
        
        linear_vel = linear_vel / vel_factor
        angular_vel = angular_vel / vel_factor
        
        # ============================================================
        # CONVERT TO WHEEL VELOCITIES
        # ============================================================
        
        left_vel, right_vel = self.diff_drive_inverse_kinematics(
            linear_vel, angular_vel
        )
        
        # Store for logging
        self.left_wheel_velocity = left_vel
        self.right_wheel_velocity = right_vel
        
        # ============================================================
        # PUBLISH COMMANDS
        # ============================================================
        
        self.publish_wheel_velocities(left_vel, right_vel)
        
        # ============================================================
        # LOGGING (every 1 second = 10 control cycles)
        # ============================================================
        
        if not hasattr(self, 'control_counter'):
            self.control_counter = 0
        
        self.control_counter += 1
        
        # if self.control_counter >= int(self.control_freq):
        #     self.control_counter = 0
            
        #     # Compute error
        #     dx = self.target_x - self.current_x
        #     dy = self.target_y - self.current_y
        #     distance_error = math.sqrt(dx**2 + dy**2)
        #     yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
        #     self.get_logger().info(
        #         f'Current: ({self.current_x:.2f}, {self.current_y:.2f}, {math.degrees(self.current_yaw):.1f}°) | '
        #         f'Target: ({self.target_x:.2f}, {self.target_y:.2f}, {math.degrees(self.target_yaw):.1f}°) | '
        #         f'Error: {distance_error:.3f}m, {math.degrees(yaw_error):.1f}° | '
        #         f'Cmd: v={linear_vel:.2f}, ω={angular_vel:.2f}'
        #     )
    
    # ============================================================
    # CONTROL FUNCTION (TO BE IMPLEMENTED)
    # ============================================================
    
    def control_function(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_x: float,
        target_y: float,
        target_yaw: float
    ) -> Tuple[float, float]:
        """
        Control function to compute linear and angular velocities.
        
        Args:
            current_x: Current x position (m)
            current_y: Current y position (m)
            current_yaw: Current yaw angle (rad)
            target_x: Target x position (m)
            target_y: Target y position (m)
            target_yaw: Target yaw angle (rad)
        
        Returns:
            Tuple[linear_vel, angular_vel]:
                - linear_vel: Forward velocity command (m/s)
                - angular_vel: Angular velocity command (rad/s)
        
        NOTE: Implement your control algorithm here!
        This is a placeholder that returns zero velocities.
        
        Example control strategies:
        1. Pure Pursuit
        2. PID control
        3. Model Predictive Control (MPC)
        4. Trajectory tracking
        5. Potential field navigation
        """
        
        linear_vel = 0.0
        angular_vel = 0.0
        current_yaw = self.normalize_angle(current_yaw)

        # Compute distance to target
        distance_to_target = np.linalg.norm([target_x - current_x, target_y - current_y])
        
        # Next local waypoint
        if distance_to_target < self.pos_tolerance:
            # Already at target position
            x_next = current_x
            y_next = current_y
        else:
            # Compute direction to target
            x_dir = target_x - current_x
            y_dir = target_y - current_y
            dir_norm = np.linalg.norm([x_dir, y_dir])
            
            # Normalize direction vector
            x_dir = x_dir / dir_norm
            y_dir = y_dir / dir_norm
            
            # Compute next waypoint (one tolerance distance ahead)
            x_next = current_x + x_dir * self.pos_tolerance
            y_next = current_y + y_dir * self.pos_tolerance

        # Error terms
        dx = x_next - current_x
        dy = y_next - current_y
        dyaw = self.normalize_angle(target_yaw - current_yaw)
        
        # Lateral error (perpendicular to robot heading)
        e_lat = np.dot([-np.sin(current_yaw), np.cos(current_yaw)], [dx, dy])
        
        # Forward error (along robot heading)
        v_e = np.dot([np.cos(current_yaw), np.sin(current_yaw)], [dx, dy])
        
        # Angular error with lateral error influence
        # Avoid division by zero when e_lat is very small
        if abs(e_lat) < 1e-6:
            w_e = dyaw
        else:
            w_e = np.arctan(e_lat) + (abs(e_lat) / 10.0) * dyaw

        # PID control (currently just proportional)
        linear_vel = self.Kp_linear * v_e
        angular_vel = self.Kp_angular * w_e

        
        return linear_vel, angular_vel
    
    # ============================================================
    # KINEMATICS: DIFFERENTIAL DRIVE
    # ============================================================
    
    def diff_drive_inverse_kinematics(
        self,
        linear_vel: float,
        angular_vel: float
    ) -> Tuple[float, float]:
        """
        Convert linear and angular velocities to left and right wheel velocities.
        
        Args:
            linear_vel: Forward velocity (m/s)
            angular_vel: Angular velocity (rad/s)
        
        Returns:
            Tuple[left_wheel_rps, right_wheel_rps]: Wheel velocities in motor turns/s
        """
        # Differential drive kinematics
        v_left = linear_vel - (angular_vel * self.wheel_base / 2)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2)
        
        left_wheel_mps = v_left
        right_wheel_mps = v_right
        
        # Convert m/s to wheel turns/s
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        left_wheel_rps = left_wheel_mps / wheel_circumference
        right_wheel_rps = right_wheel_mps / wheel_circumference
        
        # Apply gear ratio and velocity multiplier
        left_wheel_rps *= self.gear_ratio * self.velocity_multiplier
        right_wheel_rps *= self.gear_ratio * self.velocity_multiplier

        
        return left_wheel_rps, right_wheel_rps
    
    # ============================================================
    # PUBLISHING
    # ============================================================
    
    def publish_wheel_velocities(self, left_rps: float, right_rps: float):
        """
        Publish velocity commands to left and right ODrive motors.
        
        Args:
            left_rps: Left wheel velocity (motor turns/s)
            right_rps: Right wheel velocity (motor turns/s)
        """
        # Apply motor inversion
        left_cmd = -left_rps if self.invert_left else left_rps
        right_cmd = -right_rps if self.invert_right else right_rps
        
        # Create left motor command
        left_msg = ControlMessage()
        left_msg.control_mode = 2  # VELOCITY_CONTROL
        left_msg.input_mode = 2    # VEL_RAMP
        left_msg.input_vel = float(left_cmd)
        left_msg.input_torque = 0.0
        left_msg.input_pos = 0.0
        
        # Create right motor command
        right_msg = ControlMessage()
        right_msg.control_mode = 2  # VELOCITY_CONTROL
        right_msg.input_mode = 2    # VEL_RAMP
        right_msg.input_vel = float(right_cmd)
        right_msg.input_torque = 0.0
        right_msg.input_pos = 0.0
        
        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
    
    def publish_zero_velocity(self):
        """
        Send zero velocity commands (coast to stop).
        """
        self.publish_wheel_velocities(0.0, 0.0)
    
    # ============================================================
    # SERVICE CALLBACKS
    # ============================================================
    
    def set_target_callback(self, msg: PoseStamped):
        """
        Topic callback to set a new target pose via PoseStamped message.
        """
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.target_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        self.has_target = True
        # Auto-enable controller on first target if disabled
        if not self.controller_enabled:
            self.controller_enabled = True
            self.get_logger().info('✓ Controller auto-enabled on target reception')
        
        self.get_logger().info(
            f'🎯 New target set: x={self.target_x:.2f}m, y={self.target_y:.2f}m, '
            f'yaw={math.degrees(self.target_yaw):.1f}°'
        )
    
    def enable_callback(self, request, response):
        """
        Service callback to enable the controller.
        """
        self.controller_enabled = True
        response.success = True
        response.message = '✓ Controller ENABLED'
        self.get_logger().info('✓ Controller ENABLED')
        return response
    
    def disable_callback(self, request, response):
        """
        Service callback to disable the controller.
        """
        self.controller_enabled = False
        self.publish_zero_velocity()
        response.success = True
        response.message = '✓ Controller DISABLED'
        self.get_logger().info('✓ Controller DISABLED')
        return response
    
    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================
    
    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle (rotation around Z-axis).
        
        Args:
            qx, qy, qz, qw: Quaternion components
        
        Returns:
            yaw: Yaw angle in radians (-π to π)
        """
        # Yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-π, π] range.
        
        Args:
            angle: Angle in radians
        
        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def __del__(self):
        """
        Cleanup when node is destroyed - send zero velocity.
        """
        self.publish_zero_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted by user')
    finally:
        # Send zero velocity before shutdown
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

