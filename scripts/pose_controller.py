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
- Waypoint navigation: autonomous navigation through CSV-defined waypoints

Usage:
  ros2 run pilot_control pose_controller.py

Topics:
  Subscribed:
    - /Odometry (nav_msgs/Odometry) - Fast-LIO2 localization
    - /set_target_pose (std_msgs/Float64MultiArray) - [x, y, z, yaw(rad)]
  Published:
    - /left/control_message (odrive_can/ControlMessage) - Left wheel velocity
    - /right/control_message (odrive_can/ControlMessage) - Right wheel velocity

Topics:
  - /start_waypoint_navigation (std_msgs/String) - Start autonomous waypoint navigation
    Message: data (string) - Path to CSV file containing waypoints

Waypoint Navigation:
  The waypoint navigation feature allows autonomous navigation through a series of waypoints
  defined in a CSV file. Each waypoint consists of x,y coordinates.

  CSV Format:
    x,y
    1.0,0.0
    2.0,1.0
    3.0,0.0

  Navigation Logic:
  1. Yaw Alignment Phase: Robot first rotates to face the target waypoint
  2. Straight Line Movement: Once aligned, robot moves in a straight line to the waypoint
  3. Waypoint Achievement: When waypoint is reached, automatically proceeds to next waypoint
  4. Completion: Stops when all waypoints are completed

  Usage:
    # Publish CSV file path to topic
    ros2 topic pub /start_waypoint_navigation std_msgs/String "{data: '/path/to/waypoints.csv'}"

    # Or use the test script
    python3 src/pilot_control/scripts/test_waypoint_navigation_topic.py /path/to/waypoints.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from std_msgs.msg import String
import math
import numpy as np
import csv
from typing import Tuple, Optional
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
import threading
import sys
import os
import signal
import time
import select
import termios
import tty


class PoseController(Node):
    def __init__(self):
        super().__init__('pose_controller')
        
        # ============================================================
        # PARAMETERS
        # ============================================================
        
        # Robot kinematics (matching diff_drive_controller)
        self.declare_parameter('wheel_radius', 0.09)      # m
        self.declare_parameter('wheel_base', 0.32)         # m
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        self.declare_parameter('velocity_multiplier', 1.0)
        
        # Control parameters
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('max_linear_velocity', 0.4) # m/s
        self.declare_parameter('max_angular_velocity', 4.0) # rad/s
        self.declare_parameter('position_tolerance', 0.05) # m
        self.declare_parameter('orientation_tolerance', 0.1) # rad (~5.7 degrees)
        self.declare_parameter('min_wheel_rps', 0.2) # rps
        self.declare_parameter('lookahead_distance', 0.4) # m (5cm)
        
        # Error computation parameters
        self.declare_parameter('r_close', 0.01)  # 5mm - start strong yaw correction
        self.declare_parameter('r_far', 0.03)     # 10cm - pure go-to-point steering beyond this
        self.declare_parameter('K_lat', 1.0)    # lateral correction gain
        self.declare_parameter('K_yaw', 1.0)    # yaw correction gain when close
        self.declare_parameter('yaw_tol', 0.0)   # desired yaw accuracy (rad)
        self.declare_parameter('blend_prefixed', 0.0) # blend factor for yaw correction
        
        # Tilt correction parameters (similar to gpr_scan_controller)
        self.declare_parameter('pitch_rad', -0.2617993878)  # ~ -15 deg fallback
        self.declare_parameter('roll_rad', 0.0)  # Roll correction (rad)
        self.declare_parameter('yaw_rad', 0.0)  # Yaw correction (rad)
        self.declare_parameter('accel_topic', '/livox/imu')
        self.declare_parameter('accel_samples', 10)
        
        # IMU coordinate transformation parameters (no axis flips applied)

        self.declare_parameter('Kp_linear', 1.0) # 5.0
        self.declare_parameter('Ki_linear', 0.0)
        self.declare_parameter('Kd_linear', 0.0)
        self.declare_parameter('Kp_angular', 1.0) # 1.0
        self.declare_parameter('Ki_angular', 0.0)
        self.declare_parameter('Kd_angular', 0.0)

        # Safety parameters
        self.declare_parameter('enable_controller', True) # Start enabled by default
        self.declare_parameter('odometry_timeout_ms', 500) # Stop if no odometry
        
        # Topic names
        self.declare_parameter('odometry_topic', '/Odometry_tilt_corrected_diff')  # Use tilt-corrected odometry from odom_tilt_corrector.py
        self.declare_parameter('left_control_topic', '/left/control_message')
        self.declare_parameter('right_control_topic', '/right/control_message')
        self.declare_parameter('corrected_odometry_topic', '/Odometry_tilt_corrected')
        
        # High-rate PWM command layer (optional)
        self.declare_parameter('pwm_send_enabled', False)
        self.declare_parameter('pwm_send_hz', 100.0)
        self.declare_parameter('pwm_pulse_rps', 2.0)
        self.declare_parameter('pwm_window_len', 10)
        self.declare_parameter('pwm_debug_log', False)
        
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
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # Error computation parameters
        self.r_close = self.get_parameter('r_close').value
        self.r_far = self.get_parameter('r_far').value
        self.K_lat = self.get_parameter('K_lat').value
        self.K_yaw = self.get_parameter('K_yaw').value
        self.yaw_tol = self.get_parameter('yaw_tol').value
        self.pitch_rad = float(self.get_parameter('pitch_rad').value)
        self.roll_rad = float(self.get_parameter('roll_rad').value)
        self.yaw_rad = float(self.get_parameter('yaw_rad').value)
        self.accel_topic = str(self.get_parameter('accel_topic').value)
        self.accel_samples_target = max(1, int(self.get_parameter('accel_samples').value))
        self.blend_prefixed = self.get_parameter('blend_prefixed').value
        # IMU coordinate transformation parameters (no axis flips used)
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Ki_linear = self.get_parameter('Ki_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Ki_angular = self.get_parameter('Ki_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        # If I/D gains unset, derive simple defaults from P gain for reasonable behavior
        #if (self.Ki_angular is None) or (float(self.Ki_angular) == 0.0):
        #    self.Ki_angular = 0 * float(self.Kp_angular)
        #if (self.Kd_angular is None) or (float(self.Kd_angular) == 0.0):
        #    self.Kd_angular = 0 * float(self.Kp_angular)
        self.min_wheel_rps = self.get_parameter('min_wheel_rps').value
        
        self.controller_enabled = self.get_parameter('enable_controller').value
        self.odom_timeout_ms = self.get_parameter('odometry_timeout_ms').value
        
        odometry_topic = self.get_parameter('odometry_topic').value
        left_control_topic = self.get_parameter('left_control_topic').value
        right_control_topic = self.get_parameter('right_control_topic').value
        corrected_odom_topic = self.get_parameter('corrected_odometry_topic').value
        
        # PWM parameters
        self.pwm_enabled = bool(self.get_parameter('pwm_send_enabled').value)
        self.pwm_hz = float(self.get_parameter('pwm_send_hz').value)
        self.pwm_pulse_rps = float(self.get_parameter('pwm_pulse_rps').value)
        self.pwm_window_len = max(1, int(self.get_parameter('pwm_window_len').value))
        self.pwm_debug_log = bool(self.get_parameter('pwm_debug_log').value)
        
        # ============================================================
        # STATE VARIABLES
        # ============================================================
        
        # Current pose from Fast-LIO2
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self.pose_initialized = False
        self.last_odom_time = self.get_clock().now()
        # Throttled logging state
        self._last_log_time_ns = {}
        # PWM state
        self._last_left_cmd = 0.0
        self._last_right_cmd = 0.0
        self._pwm_idx = 0
        # Frame origin (initial tilt-corrected pose defines global origin)
        self.frame_origin_set = False
        self.frame_origin_pos = np.zeros(3, dtype=float)
        self.frame_origin_quat = (0.0, 0.0, 0.0, 1.0)
        # Angular PID state
        self._ang_integral = 0.0
        self._ang_prev_err = 0.0
        self._ang_prev_time_ns = None
        self._ang_integral_limit = 20.0  # anti-windup clamp
        # Tilt correction state
        self.accel_initialized = False
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
        self.accel_avg = None  # store averaged acceleration for world-frame leveling
        # Alignment and origin transforms (matrix form)
        self.R_align_world = np.eye(3, dtype=float)  # world -> flat-world
        self.last_raw_q = (0.0, 0.0, 0.0, 1.0)
        self.last_raw_pos = np.zeros(3, dtype=float)
        self.alignment_set = False
        self.origin_set = False
        self.p0_flat = np.zeros(3, dtype=float)
        self.R0_flat_b = np.eye(3, dtype=float)
        self.A_world_to_local = np.eye(3, dtype=float)  # p_local = A * p_world + b
        self.b_world_to_local = np.zeros(3, dtype=float)
        # Yaw-only origin in leveled world
        self.p0_world = np.zeros(3, dtype=float)
        self.yaw0 = 0.0
        # Initialize fallback alignment quaternion with full 3D correction
        # Order: Roll (X), Pitch (Y), Yaw (Z) - applied in sequence
        roll_quat = self.quat_from_axis_angle([1.0, 0.0, 0.0], self.roll_rad)
        pitch_quat = self.quat_from_axis_angle([0.0, 1.0, 0.0], self.pitch_rad)
        yaw_quat = self.quat_from_axis_angle([0.0, 0.0, 1.0], self.yaw_rad)
        # Combine rotations: yaw * pitch * roll (applied in reverse order)
        self.align_quat = self.quat_multiply(yaw_quat, self.quat_multiply(pitch_quat, roll_quat))
        
        # Target pose
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        self.target_achieved = False
        self.zero_velocity_sent = False

        # Waypoint navigation state
        self.waypoint_navigation_active = False
        self.waypoints = []  # List of (x, y) tuples
        self.current_waypoint_index = 0
        self.previous_waypoint = None  # (x, y) of previous waypoint
        self.yaw_alignment_phase = False  # True when aligning yaw before moving
        
        # Starting pose when target was received (for line-following)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.start_pose_set = False
        
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
            qos_profile_sensor_data
        )
        # IMU subscription for gravity-based tilt correction
        self.imu_sub = self.create_subscription(
            Imu,
            self.accel_topic,
            self.imu_callback,
            qos_profile_sensor_data
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
        # Publisher for tilt-corrected odometry
        self.odom_corr_pub = self.create_publisher(
            Odometry,
            corrected_odom_topic,
            10
        )
        # Diagnostic publishers: errors, v/w command, wheel velocities
        self.errors_pub = self.create_publisher(
            Float64MultiArray,
            '/pose_control/errors',
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/pose_control/cmd_twist',
            10
        )
        self.wheels_pub = self.create_publisher(
            Float64MultiArray,
            '/pose_control/wheel_vel',
            10
        )
        # Individually labeled publishers for rqt_plot
        self.err_dx_pub = self.create_publisher(Float64, '/pose_control/error_dx', 10)
        self.err_dy_pub = self.create_publisher(Float64, '/pose_control/error_dy', 10)
        self.err_e_lat_pub = self.create_publisher(Float64, '/pose_control/error_e_lat', 10)
        self.err_v_e_pub = self.create_publisher(Float64, '/pose_control/error_v_e', 10)
        self.err_dyaw_pub = self.create_publisher(Float64, '/pose_control/error_dyaw', 10)
        self.left_rps_pub = self.create_publisher(Float64, '/pose_control/left_rps', 10)
        self.right_rps_pub = self.create_publisher(Float64, '/pose_control/right_rps', 10)
        
        # Subscriber to set target pose (Float64MultiArray: [x,y,z,yaw])
        self.set_target_sub = self.create_subscription(
            Float64MultiArray,
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

        # Topic for waypoint navigation (receives CSV file path as string)
        self.waypoint_nav_sub = self.create_subscription(
            String,
            '/start_waypoint_navigation',
            self.start_waypoint_navigation_callback,
            10
        )

        # Service to trigger graceful shutdown (zero torque → disarm → stop nodes)
        self.soft_shutdown_srv = self.create_service(
            Trigger,
            '/pose_controller/soft_shutdown',
            self._soft_shutdown_service
        )
        
        # Control loop timer (10 Hz by default)
        control_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # High-rate PWM command sender timer
        self.pwm_timer = None
        if self.pwm_enabled and self.pwm_hz > 0.0:
            self.pwm_timer = self.create_timer(1.0 / float(self.pwm_hz), self._pwm_send_step)

        # ============================================================
        # STARTUP: ARM MOTORS (CLOSED-LOOP) VIA ODRIVE CAN SERVICES
        # ============================================================
        self.left_axis_client = self.create_client(AxisState, '/left/request_axis_state')
        self.right_axis_client = self.create_client(AxisState, '/right/request_axis_state')
        self.left_clear_client = self.create_client(Empty, '/left/clear_errors')
        self.right_clear_client = self.create_client(Empty, '/right/clear_errors')
        # Shutdown mapping service (optional)
        from std_srvs.srv import Trigger as _Trigger
        self.shutdown_client = self.create_client(_Trigger, '/shutdown_mapping')

        self._arm_attempts = 0
        self._arm_max_attempts = 5
        self._arm_timer = self.create_timer(1.0, self._attempt_arm_motors)
        
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
        self.get_logger().info(f'  Lookahead distance: {self.lookahead_distance:.3f} m')
        self.get_logger().info(f'  Error computation: r_close={self.r_close:.3f}m, r_far={self.r_far:.3f}m')
        self.get_logger().info(f'  Error gains: K_lat={self.K_lat:.1f}, K_yaw={self.K_yaw:.1f}')
        self.get_logger().info(f'')
        self.get_logger().info(f'Tilt Correction Parameters:')
        self.get_logger().info(f'  Roll correction: {math.degrees(self.roll_rad):.1f}°')
        self.get_logger().info(f'  Pitch correction: {math.degrees(self.pitch_rad):.1f}°')
        self.get_logger().info(f'  Yaw correction: {math.degrees(self.yaw_rad):.1f}°')
        # No IMU axis flips applied; using raw IMU axes
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
        self.get_logger().info(f'Topics:')
        self.get_logger().info(f'  /start_waypoint_navigation (std_msgs/String) - Start autonomous waypoint navigation')
        self.get_logger().info(f'')
        self.get_logger().info(f'Controller Status: {"ENABLED" if self.controller_enabled else "DISABLED"}')
        self.get_logger().info('='*70)
        
        if not self.controller_enabled:
            self.get_logger().info('Controller is currently disabled; it will auto-enable on first target pose.')

        # Start keyboard watcher for ESC-triggered shutdown (if stdin is a TTY)
        try:
            if sys.stdin.isatty():
                self._kb_thread = threading.Thread(target=self._keyboard_watch, daemon=True)
                self._kb_thread.start()
            else:
                self.get_logger().info('Keyboard not attached to this process (no TTY); ESC shutdown disabled')
        except Exception:
            pass
    
    # ============================================================
    # CALLBACK: ODOMETRY
    # ============================================================
    
    def odometry_callback(self, msg: Odometry):
        """
        Callback for tilt-corrected odometry messages (from odom_tilt_corrector.py).
        Extracts x, y, yaw from the already-corrected odometry message.
        """
        # Extract pose from tilt-corrected odometry (already leveled)
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        q_local = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        self.current_yaw = self.quaternion_to_yaw(q_local[0], q_local[1], q_local[2], q_local[3])
        
        # Extract velocities (already transformed to leveled frame)
        self.current_vx = float(msg.twist.twist.linear.x)
        self.current_vy = float(msg.twist.twist.linear.y)
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

        # Republish corrected odometry (pass through)
        try:
            self.odom_corr_pub.publish(msg)
        except Exception:
            pass

    # =============================
    # Tilt Correction Utilities
    # =============================
    @staticmethod
    def quat_normalize(q):
        x, y, z, w = q
        n = math.sqrt(x*x + y*y + z*z + w*w)
        if n <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (x/n, y/n, z/n, w/n)

    @staticmethod
    def quat_from_axis_angle(axis, angle_rad):
        ax = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(ax)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        ax = ax / norm
        s = math.sin(angle_rad * 0.5)
        c = math.cos(angle_rad * 0.5)
        return (ax[0]*s, ax[1]*s, ax[2]*s, c)

    @staticmethod
    def quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        return (x, y, z, w)

    @staticmethod
    def quat_conjugate(q):
        x, y, z, w = q
        return (-float(x), -float(y), -float(z), float(w))

    @staticmethod
    def rotate_vector_by_quat(v, q):
        x, y, z = v
        qx, qy, qz, qw = q
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        vx = x + qw * tx + (qy * tz - qz * ty)
        vy = y + qw * ty + (qz * tx - qx * tz)
        vz = z + qw * tz + (qx * ty - qy * tx)
        return np.array([vx, vy, vz], dtype=float)

    @staticmethod
    def compute_alignment_quat(from_vec, to_vec):
        v1 = np.asarray(from_vec, dtype=float)
        v2 = np.asarray(to_vec, dtype=float)
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 <= 1e-12 or n2 <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        v1 = v1 / n1
        v2 = v2 / n2
        cos_theta = float(np.clip(v1.dot(v2), -1.0, 1.0))
        if cos_theta > 1.0 - 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        if cos_theta < -1.0 + 1e-9:
            axis = np.array([1.0, 0.0, 0.0])
            if abs(v1[0]) > 0.9:
                axis = np.array([0.0, 1.0, 0.0])
            axis = axis - v1 * v1.dot(axis)
            if np.linalg.norm(axis) < 1e-12:
                axis = np.array([0.0, 0.0, 1.0])
            axis = axis / (np.linalg.norm(axis) + 1e-12)
            return PoseController.quat_from_axis_angle(axis, math.pi)
        axis = np.cross(v1, v2)
        s = math.sqrt((1.0 + cos_theta) * 2.0)
        invs = 1.0 / s
        w = 0.5 * s
        x = axis[0] * invs
        y = axis[1] * invs
        z = axis[2] * invs
        return PoseController.quat_normalize((x, y, z, w))

    @staticmethod
    def rpy_to_matrix(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll);  sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)
        cy = math.cos(yaw);   sy = math.sin(yaw)
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        return Rz @ Ry @ Rx

    @staticmethod
    def quat_to_matrix(q):
        x, y, z, w = q
        xx = x * x; yy = y * y; zz = z * z
        xy = x * y; xz = x * z; yz = y * z
        wx = w * x; wy = w * y; wz = w * z
        R = np.array([
            [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
            [    2.0 * (xy + wz),   1.0 - 2.0 * (xx + zz),       2.0 * (yz - wx)],
            [    2.0 * (xz - wy),       2.0 * (yz + wx),     1.0 - 2.0 * (xx + yy)]
        ], dtype=float)
        return R

    @staticmethod
    def matrix_to_quat(R):
        m00, m01, m02 = float(R[0,0]), float(R[0,1]), float(R[0,2])
        m10, m11, m12 = float(R[1,0]), float(R[1,1]), float(R[1,2])
        m20, m21, m22 = float(R[2,0]), float(R[2,1]), float(R[2,2])
        tr = m00 + m11 + m22
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (m21 - m12) / S
            y = (m02 - m20) / S
            z = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / S
            x = 0.25 * S
            y = (m01 + m10) / S
            z = (m02 + m20) / S
        elif m11 > m22:
            S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / S
            x = (m01 + m10) / S
            y = 0.25 * S
            z = (m12 + m21) / S
        else:
            S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / S
            x = (m02 + m20) / S
            y = (m12 + m21) / S
            z = 0.25 * S
        return (x, y, z, w)

    def imu_callback(self, msg: Imu):
        a = np.array([
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        ], dtype=float)
        if not np.all(np.isfinite(a)):
            return
        # No axis flips: use IMU raw axes

        # Initialize gravity average once while stationary; do not set leveling here
        if not self.accel_initialized:
            self.accel_sum += a
            self.accel_count += 1
            if self.accel_count < self.accel_samples_target:
                return
            avg = self.accel_sum / float(max(1, self.accel_count))
            self.accel_avg = avg
            self.accel_initialized = True
            try:
                self.get_logger().info(
                    f'Pose tilt correction initialized with {self.accel_count} samples')
                # Debug: report averaged accel (raw axes)
                try:
                    self.get_logger().info(
                        f'accel_avg={self.accel_avg[0]:.3f},{self.accel_avg[1]:.3f},{self.accel_avg[2]:.3f}; '
                        f'waiting for first odom to set leveling')
                except Exception:
                    pass
            except Exception:
                pass
        else:
            # After initialization, ignore further IMU updates for leveling
            return
    
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
            self.warn_throttled('waiting_odom', '⚠️  Waiting for odometry from Fast-LIO2...', 5.0)
            self.publish_zero_velocity()
            return
        
        # Check odometry timeout
        time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e6
        if time_since_odom > self.odom_timeout_ms:
            self.warn_throttled('odom_timeout', f'⚠️  Odometry timeout ({time_since_odom:.1f} ms > {self.odom_timeout_ms} ms). Stopping motors.', 2.0)
            #self.publish_zero_velocity()
            return
        
        # Check if target is set
        if not self.has_target:
            # No target - send zero velocity once and wait
            if not self.zero_velocity_sent:
                self.publish_zero_velocity()
                self.zero_velocity_sent = True
            return
        
        # Check if target is achieved
        if self.target_achieved:
            # Target achieved - send zero velocity once and wait for new target
            if not self.zero_velocity_sent:
                self.publish_zero_velocity()
                self.zero_velocity_sent = True

            # Check if we need to move to next waypoint
            self.check_waypoint_achievement()
            return
        
        # ============================================================
        # CALL CONTROL FUNCTION
        # ============================================================
        
        # Reset zero velocity flag since we're actively controlling
        self.zero_velocity_sent = False
        
        linear_vel, angular_vel, diag = self.control_function(
            self.current_x, self.current_y, self.current_yaw,
            self.target_x, self.target_y, self.target_yaw
        )
        
        # ============================================================
        # VELOCITY RESCALING 
        # ============================================================
        # Scale down velocities proportionally if either exceeds limits
        linear_scale = abs(linear_vel) / self.max_linear_vel if self.max_linear_vel > 0 else 0.0
        angular_scale = abs(angular_vel) / self.max_angular_vel if self.max_angular_vel > 0 else 0.0
        vel_factor = max(linear_scale, angular_scale)
        if not np.isfinite(vel_factor) or vel_factor < 1.0:
            vel_factor = 1.0
        linear_vel = linear_vel / vel_factor
        angular_vel = angular_vel / vel_factor


        
        # ============================================================
        # CONVERT TO WHEEL VELOCITIES
        # ============================================================
        
        left_vel, right_vel = self.diff_drive_inverse_kinematics(
            linear_vel, angular_vel
        )

        ## Minimum velocity threshold

        
        # Store for logging
        self.left_wheel_velocity = left_vel
        self.right_wheel_velocity = right_vel
        
        # ============================================================
        # PUBLISH COMMANDS (direct or via PWM layer)
        # ============================================================
        
        #if self.pwm_enabled and self.pwm_timer is not None:
        #    # Defer actual publish to high-rate PWM timer
        #    self._last_left_cmd = float(left_vel)
        #    self._last_right_cmd = float(right_vel)
        #else:
        #    self.publish_wheel_velocities(left_vel, right_vel)
        self.publish_wheel_velocities(left_vel, right_vel)

        # ============================================================
        # PUBLISH DIAGNOSTICS
        # ============================================================
        try:
            # Errors
            err_msg = Float64MultiArray()
            # Order: dx, dy, e_lat, v_e, dyaw
            err_msg.data = [
                float(diag.get('dx', 0.0)),
                float(diag.get('dy', 0.0)),
                float(diag.get('e_lat', 0.0)),
                float(diag.get('v_e', 0.0)),
                float(diag.get('dyaw', 0.0)),
            ]
            self.errors_pub.publish(err_msg)

            # Individually labeled errors
            dx_msg = Float64(); dx_msg.data = float(diag.get('dx', 0.0)); self.err_dx_pub.publish(dx_msg)
            dy_msg = Float64(); dy_msg.data = float(diag.get('dy', 0.0)); self.err_dy_pub.publish(dy_msg)
            el_msg = Float64(); el_msg.data = float(diag.get('e_lat', 0.0)); self.err_e_lat_pub.publish(el_msg)
            ve_msg = Float64(); ve_msg.data = float(diag.get('v_e', 0.0)); self.err_v_e_pub.publish(ve_msg)
            yw_msg = Float64(); yw_msg.data = float(diag.get('dyaw', 0.0)); self.err_dyaw_pub.publish(yw_msg)

            # Command v, w
            tw = Twist()
            tw.linear.x = float(linear_vel)
            tw.angular.z = float(angular_vel)
            self.cmd_pub.publish(tw)

            # Wheel velocities
            wheels_msg = Float64MultiArray()
            wheels_msg.data = [float(left_vel), float(right_vel)]
            self.wheels_pub.publish(wheels_msg)

            # Individually labeled wheel velocities
            l_msg = Float64(); l_msg.data = float(left_vel); self.left_rps_pub.publish(l_msg)
            r_msg = Float64(); r_msg.data = float(right_vel); self.right_rps_pub.publish(r_msg)
        except Exception:
            pass
        
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
    # LOCAL WAYPOINT GENERATION
    # ============================================================
    
    def generate_local_waypoint(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_x: float,
        target_y: float,
        target_yaw: float
    ) -> Tuple[float, float]:
        """
        Generate the next local waypoint using line-following with lookahead distance.
        
        The path is a straight line from start pose to target pose. The waypoint is
        the point on this line that is closest to current pose, plus lookahead distance.
        
        Args:
            current_x: Current x position (m)
            current_y: Current y position (m)
            current_yaw: Current yaw angle (rad)
            target_x: Target x position (m)
            target_y: Target y position (m)
            target_yaw: Target yaw angle (rad)
        
        Returns:
            Tuple[next_x, next_y]: Next local waypoint coordinates
        """
        # If starting pose not set, use current pose as starting point
        if not self.start_pose_set:
            self.start_x = current_x
            self.start_y = current_y
            self.start_yaw = current_yaw
            self.start_pose_set = True
        
        # Vector from start to target
        line_vector = np.array([target_x - self.start_x, target_y - self.start_y])
        line_length = np.linalg.norm(line_vector)
        
        # Handle edge case: start and target are the same point
        if line_length < 1e-6:
            return target_x, target_y
        
        # Normalize line direction vector
        line_direction = line_vector / line_length
        
        # Vector from start to current position
        current_vector = np.array([current_x - self.start_x, current_y - self.start_y])
        
        # Project current position onto the line (distance along line parameter)
        distance_along_line = np.dot(current_vector, line_direction)
        
        # Clamp to line segment [0, line_length]
        distance_along_line = np.clip(distance_along_line, 0.0, line_length)
        
        # Point on line closest to current position
        closest_point_on_line = self.start_x + line_direction[0] * distance_along_line, \
                               self.start_y + line_direction[1] * distance_along_line
        
        # Calculate next waypoint: closest point + lookahead distance
        next_distance = distance_along_line + self.lookahead_distance
        
        # If next waypoint goes beyond target, use target as waypoint
        #if next_distance >= line_length:
        #    return target_x, target_y
        
        # Calculate next waypoint coordinates
        next_x = self.start_x + line_direction[0] * next_distance
        next_y = self.start_y + line_direction[1] * next_distance
        
        return next_x, next_y
    
    # ============================================================
    # ERROR COMPUTATION FUNCTION
    # ============================================================
    
    def compute_errors(self, waypoint_x: float, waypoint_y: float, waypoint_yaw: float,
                      current_x: float, current_y: float, current_yaw: float,
                      target_x: float, target_y: float, target_yaw: float,
                      r_goal: float) -> Tuple[float, float, float, float]:
        """
        Compute linear and angular errors for waypoint navigation with smooth transition
        between position and orientation control.
        
        Based on MATLAB reference implementation.
        
        Args:
            waypoint_x: Next waypoint x position (m)
            waypoint_y: Next waypoint y position (m) 
            waypoint_yaw: Target yaw angle (rad)
            current_x: Current x position (m)
            current_y: Current y position (m)
            current_yaw: Current yaw angle (rad)
            r_goal: Distance to the global goal (m)
        
        Returns:
            Tuple[v_e, w_e, e_lat, d_yaw]:
                - v_e: Forward error (m)
                - w_e: Angular velocity command (rad/s)
                - e_lat: Lateral error (m)
                - d_yaw: Yaw error (rad)
        """
        # Initialize
        v_e = 0.0
        w_e = 0.0
        e_lat = 0.0
        d_yaw = 0.0
        
        # Pose deltas
        dx = waypoint_x - current_x
        dy = waypoint_y - current_y
        dx_targ = target_x - current_x
        dy_targ = target_y - current_y
        d_yaw = waypoint_yaw - current_yaw
        yaw = current_yaw
        
        # Wrap yaw error to [-pi, pi]
        while abs(d_yaw) > math.pi:
            d_yaw = d_yaw - math.copysign(2*math.pi, d_yaw)
        
        while abs(yaw) > math.pi:
            yaw = yaw - math.copysign(2*math.pi, yaw)
        
        # Compute lateral and longitudinal errors in robot frame
        e_lat = -math.sin(yaw) * dx + math.cos(yaw) * dy  # lateral error
        v_e = math.cos(yaw) * dx + math.sin(yaw) * dy     # forward distance (toward waypoint)
        v_e_targ = math.cos(yaw) * dx_targ + math.sin(yaw) * dy_targ
        if abs(v_e_targ) < abs(v_e):
            v_e = v_e_targ

        r = math.sqrt(dx*dx + dy*dy)                      # distance to local waypoint (for reference)
        
        # Compute steering components
        # Lateral control: large when far from target
        heading_error = math.atan2(dy,dx) - yaw
        while abs(heading_error) > math.pi:
            heading_error = heading_error - math.copysign(2*math.pi, heading_error)
        w_lat = self.K_lat * heading_error;#*math.sqrt(dx*dx + dy*dy)/self.lookahead_distance;
        e_lat = heading_error;
        
        # Orientation correction: stronger as we get closer
        w_yaw = self.K_yaw * d_yaw
        
        # Distance-based blending (use distance to GLOBAL goal)
        # blend = 0 → full lateral control (far from global goal)
        # blend = 1 → full yaw correction (very close to global goal)
        blend = (self.r_far - r_goal) / max((self.r_far - self.r_close), 1e-6)
        blend = max(min(blend, 1.0), 0.0)  # clamp 0–1

        # Smoothstep for gradual transition: 3*t^2 - 2*t^3
        blend = 3*blend*blend - 2*blend*blend*blend

        # Override blend for waypoint navigation phases
        if self.waypoint_navigation_active:
            if self.yaw_alignment_phase:
                # During yaw alignment, focus on yaw correction
                yaw_error = abs(self.normalize_angle(self.target_yaw - current_yaw))
                if yaw_error < math.radians(5.0):  # Within 5 degrees
                    # Switch to straight line movement
                    self.yaw_alignment_phase = False
                    self.get_logger().info('✓ Yaw aligned, switching to straight line movement')
                    blend = 0.0  # Full lateral control for straight line
                else:
                    blend = 1.0  # Full yaw correction during alignment
            else:
                # During straight line movement, use lateral control
                blend = 0.0
        else:
            blend = self.blend_prefixed # DEBUG: always use yaw correction
        
        # Combine the two steering components
        w_e = (1.0 - blend) * w_lat + blend * w_yaw
        
        # Final yaw fine-tuning
        if r < self.r_close and abs(d_yaw) < self.yaw_tol:
            # Within goal tolerance → no further rotation
            w_e = 0.0
        
        return v_e, w_e, e_lat, d_yaw
    
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
    ) -> Tuple[float, float, dict]:
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

        # Check if target is achieved
        distance_to_target = np.linalg.norm([target_x - current_x, target_y - current_y])
        dyaw = self.normalize_angle(target_yaw - current_yaw)

        # Context-aware waypoint achievement based on navigation phase
        target_achieved = False
        if self.waypoint_navigation_active:
            # During waypoint navigation, only check achievement during straight line movement phase
            # Never mark waypoints as achieved during yaw alignment phase
            if not self.yaw_alignment_phase:
                # During straight line movement: only check position tolerance
                target_achieved = distance_to_target < self.pos_tolerance
        else:
            # Regular navigation: check both tolerances
            target_achieved = distance_to_target < self.pos_tolerance and abs(dyaw) < self.ori_tolerance

        if target_achieved:
            # Target achieved - mark as achieved and return zero velocities
            self.target_achieved = True
            self.zero_velocity_sent = False  # Reset flag for next target
            return linear_vel, angular_vel, {
                'dx': 0.0,
                'dy': 0.0,
                'e_lat': 0.0,
                'v_e': 0.0,
                'dyaw': 0.0,
            }
        
        # Generate local waypoint
        x_next, y_next = self.generate_local_waypoint(
            current_x, current_y, current_yaw,
            target_x, target_y, target_yaw
        )

        # Calculate errors using new error computation function
        # Distance to GLOBAL goal
        r_goal = float(np.linalg.norm([target_x - current_x, target_y - current_y]))
        v_e, w_e, e_lat, d_yaw = self.compute_errors(
            x_next, y_next, target_yaw,
            current_x, current_y, current_yaw,target_x, target_y, target_yaw,
            r_goal
        )

        # PID control for linear velocity (currently proportional)
        linear_vel = self.Kp_linear * v_e
        # Angular PID on w_e with anti-windup and derivative
        now_ns = self.get_clock().now().nanoseconds
        if self._ang_prev_time_ns is None:
            dt = 1.0 / float(max(1e-6, self.control_freq))
        else:
            dt = max(1e-6, (now_ns - self._ang_prev_time_ns) / 1e9)
        err_a = float(w_e)
        # Integrator
        self._ang_integral += err_a * dt
        # Anti-windup
        if self._ang_integral > self._ang_integral_limit:
            self._ang_integral = self._ang_integral_limit
        elif self._ang_integral < -self._ang_integral_limit:
            self._ang_integral = -self._ang_integral_limit
        # Derivative (on measurement)
        d_err = (err_a - self._ang_prev_err) / dt if dt > 0.0 else 0.0
        angular_vel = (
            float(self.Kp_angular) * err_a +
            float(self.Ki_angular) * self._ang_integral +
            float(self.Kd_angular) * d_err
        )
        self._ang_prev_err = err_a
        self._ang_prev_time_ns = now_ns

        
        return linear_vel, angular_vel, {
            'dx': float(x_next - current_x),
            'dy': float(y_next - current_y),
            'e_lat': float(e_lat),
            'v_e': float(v_e),
            'dyaw': float(d_yaw),
        }
    
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
        v_right = linear_vel +(angular_vel * self.wheel_base / 2)
        
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
        left_msg.input_mode = 1    # PASSTHROUGH
        left_msg.input_vel = float(left_cmd)
        left_msg.input_torque = 0.0
        left_msg.input_pos = 0.0
        
        # Create right motor command
        right_msg = ControlMessage()
        right_msg.control_mode = 2  # VELOCITY_CONTROL
        right_msg.input_mode = 1    # PASSTHROUGH
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
        self._last_left_cmd = 0.0
        self._last_right_cmd = 0.0
        self._pwm_idx = 0
        self.publish_wheel_velocities(0.0, 0.0)
    
    # ============================================================
    # SERVICE CALLBACKS
    # ============================================================
    
    def set_target_callback(self, msg: Float64MultiArray):
        """
        Topic callback to set a new target pose via Float64MultiArray [x, y, z, yaw(rad)].
        """
        try:
            data = list(msg.data)
            if len(data) < 4:
                self.get_logger().warning('set_target_pose requires 4 elements: [x,y,z,yaw]')
                return
            self.target_x = float(data[0])
            self.target_y = float(data[1])
            # z currently unused in control but stored for completeness
            self.target_z = float(data[2]) if hasattr(self, 'target_z') else float(data[2])
            self.target_yaw = float(data[3])
        except Exception as e:
            self.get_logger().warning(f'Invalid set_target_pose payload: {e}')
            return
        
        self.has_target = True
        self.target_achieved = False  # Reset target achievement status
        self.zero_velocity_sent = False  # Reset zero velocity flag
        self.start_pose_set = False  # Reset starting pose for new target
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
        self.waypoint_navigation_active = False  # Also disable waypoint navigation
        self.publish_zero_velocity()
        response.success = True
        response.message = '✓ Controller DISABLED'
        self.get_logger().info('✓ Controller DISABLED')
        return response

    def start_waypoint_navigation_callback(self, msg):
        """
        Topic callback to start waypoint navigation with CSV file.
        Receives std_msgs/String message containing CSV file path
        """
        try:
            csv_file_path = msg.data
            if not os.path.exists(csv_file_path):
                self.get_logger().error(f'CSV file not found: {csv_file_path}')
                return

            # Parse waypoints from CSV
            waypoints = self.parse_waypoints_csv(csv_file_path)
            if len(waypoints) < 1:
                self.get_logger().error(f'No valid waypoints found in CSV: {csv_file_path}')
                return

            # Initialize waypoint navigation
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.waypoint_navigation_active = True
            self.yaw_alignment_phase = False

            # For first waypoint, use current position as previous waypoint
            if self.pose_initialized:
                self.previous_waypoint = (self.current_x, self.current_y)
            else:
                self.get_logger().error('Odometry not initialized yet - cannot start waypoint navigation')
                return

            # Enable controller if not already enabled
            if not self.controller_enabled:
                self.controller_enabled = True
                self.get_logger().info('✓ Controller auto-enabled for waypoint navigation')

            # Start navigation to first waypoint
            self.set_next_waypoint_target()

            self.get_logger().info(f'✓ Waypoint navigation started: {len(waypoints)} waypoints loaded from {csv_file_path}')

        except Exception as e:
            self.get_logger().error(f'Error starting waypoint navigation: {str(e)}')

    def parse_waypoints_csv(self, csv_file_path: str) -> list:
        """
        Parse waypoints from CSV file.
        Expected format: CSV with at least x,y columns (header optional)
        """
        waypoints = []
        try:
            with open(csv_file_path, 'r') as f:
                reader = csv.reader(f)
                for row_num, row in enumerate(reader):
                    # Skip empty rows
                    if not row or all(not cell.strip() for cell in row):
                        continue

                    # Try to parse as x,y coordinates
                    if len(row) >= 2:
                        try:
                            x = float(row[0].strip())
                            y = float(row[1].strip())
                            waypoints.append((x, y))
                        except ValueError:
                            # Check if this might be a header row
                            if row_num == 0 and any(keyword in ' '.join(row).lower() for keyword in ['x', 'y', 'waypoint', 'point']):
                                continue  # Skip header
                            else:
                                self.get_logger().warn(f'Skipping invalid row {row_num + 1}: {row}')
                                continue
        except Exception as e:
            self.get_logger().error(f'Error parsing CSV file {csv_file_path}: {str(e)}')
            return []

        self.get_logger().info(f'Parsed {len(waypoints)} waypoints from {csv_file_path}')
        return waypoints

    def set_next_waypoint_target(self):
        """
        Set the next waypoint as target and prepare for navigation.
        """
        if not self.waypoint_navigation_active or self.current_waypoint_index >= len(self.waypoints):
            self.waypoint_navigation_active = False
            self.get_logger().info('✓ All waypoints completed')
            return

        current_wp = self.waypoints[self.current_waypoint_index]
        self.target_x, self.target_y = current_wp

        # Calculate target yaw from previous waypoint to current waypoint
        if self.previous_waypoint is not None:
            dx = self.target_x - self.previous_waypoint[0]
            dy = self.target_y - self.previous_waypoint[1]
            self.target_yaw = math.atan2(dy, dx)
        else:
            # If no previous waypoint, keep current yaw
            self.target_yaw = self.current_yaw if hasattr(self, 'current_yaw') else 0.0

        # Start with yaw alignment phase
        self.yaw_alignment_phase = True
        self.has_target = True
        self.target_achieved = False

        self.get_logger().info(f'🎯 Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                              f'x={self.target_x:.2f}, y={self.target_y:.2f}, yaw_target={math.degrees(self.target_yaw):.1f}°')

    def check_waypoint_achievement(self):
        """
        Check if current waypoint is achieved and move to next one.
        """
        if not self.waypoint_navigation_active:
            return

        # Check if target is achieved
        if self.target_achieved:
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint_index + 1} reached')

            # Update previous waypoint to current
            self.previous_waypoint = (self.target_x, self.target_y)

            # Move to next waypoint
            self.current_waypoint_index += 1

            if self.current_waypoint_index < len(self.waypoints):
                self.set_next_waypoint_target()
            else:
                # All waypoints completed
                self.waypoint_navigation_active = False
                self.has_target = False
                self.get_logger().info('✓ All waypoints completed successfully')

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
    def quaternion_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
        """
        Convert quaternion to roll, pitch, yaw (radians).
        Returns (roll, pitch, yaw).
        """
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
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
        try:
            # Avoid publishing after shutdown/destruction
            if hasattr(self, 'left_pub') and hasattr(self, 'right_pub'):
                self.publish_zero_velocity()
        except Exception:
            pass

    # ============================================================
    # ESC KEY WATCHER AND GRACEFUL SHUTDOWN
    # ============================================================
    def _keyboard_watch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if sys.stdin in rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':  # ESC
                        self._on_escape_shutdown()
                        break
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _on_escape_shutdown(self):
        self.get_logger().warning('ESC pressed → initiating graceful shutdown: zero torque, disarm, stop nodes')
        try:
            self._send_zero_torque()
            self._disarm_odrives()
            self._call_shutdown_service()
        except Exception as e:
            try:
                self.get_logger().warning(f'Shutdown sequence encountered an error: {e}')
            except Exception:
                pass
        # Attempt to signal parent (launch) to stop
        try:
            os.kill(os.getppid(), signal.SIGINT)
        except Exception:
            pass
        # Also stop this node
        try:
            self.publish_zero_velocity()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _soft_shutdown_service(self, request, response):
        del request
        self.get_logger().warning('Soft shutdown service called → zero torque, disarm, stop nodes')
        try:
            self._send_zero_torque()
            self._disarm_odrives()
            self._call_shutdown_service()
            # Signal parent launch to stop
            try:
                os.kill(os.getppid(), signal.SIGINT)
            except Exception:
                pass
            response.success = True
            response.message = 'Shutdown initiated'
        except Exception as e:
            response.success = False
            response.message = f'Error during shutdown: {e}'
        return response

    def _send_zero_torque(self):
        # Publish torque=0 in TORQUE_CONTROL a few times to ensure delivery
        msg_left = ControlMessage()
        msg_left.control_mode = 1  # TORQUE_CONTROL
        msg_left.input_mode = 1    # PASSTHROUGH
        msg_left.input_torque = 0.0
        msg_left.input_vel = 0.0
        msg_left.input_pos = 0.0

        msg_right = ControlMessage()
        msg_right.control_mode = 1  # TORQUE_CONTROL
        msg_right.input_mode = 1    # PASSTHROUGH
        msg_right.input_torque = 0.0
        msg_right.input_vel = 0.0
        msg_right.input_pos = 0.0

        for _ in range(3):
            self.left_pub.publish(msg_left)
            self.right_pub.publish(msg_right)
            time.sleep(0.02)

    def _disarm_odrives(self):
        # Request IDLE (1) for both axes
        try:
            req_idle = AxisState.Request()
            req_idle.axis_requested_state = 1  # IDLE
            self.left_axis_client.call_async(req_idle)
            self.right_axis_client.call_async(req_idle)
            time.sleep(0.2)
        except Exception:
            pass

    def _call_shutdown_service(self):
        try:
            if self.shutdown_client.service_is_ready():
                self.shutdown_client.call_async(type(self.shutdown_client).srv.Request())
            else:
                # Try to wait briefly
                self.shutdown_client.wait_for_service(timeout_sec=0.5)
                if self.shutdown_client.service_is_ready():
                    self.shutdown_client.call_async(type(self.shutdown_client).srv.Request())
        except Exception:
            pass

    # ============================================================
    # STARTUP: ARM MOTORS HELPERS
    # ============================================================
    def _attempt_arm_motors(self):
        if self._arm_attempts >= self._arm_max_attempts:
            self._arm_timer.cancel()
            return
        self._arm_attempts += 1

        # Ensure service availability
        if not (self.left_axis_client.service_is_ready() and self.right_axis_client.service_is_ready() and
                self.left_clear_client.service_is_ready() and self.right_clear_client.service_is_ready()):
            self.warn_throttled('odrive_services', 'Waiting for ODrive CAN services to be ready...', 5.0)
            return

        try:
            # Clear errors first
            self.left_clear_client.call_async(Empty.Request())
            self.right_clear_client.call_async(Empty.Request())

            # Request CLOSED_LOOP_CONTROL (8 per ODrive enums)
            req_left = AxisState.Request()
            req_left.axis_requested_state = 8
            req_right = AxisState.Request()
            req_right.axis_requested_state = 8

            self.left_axis_client.call_async(req_left)
            self.right_axis_client.call_async(req_right)
            self.get_logger().info('Arming ODrive axes (CLOSED_LOOP_CONTROL requested)')
            # Stop timer after successful dispatch
            self._arm_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f'Arm attempt failed: {e}')

    # ============================================================
    # LOGGING: SIMPLE THROTTLED WARNING
    # ============================================================
    def warn_throttled(self, key: str, message: str, period_sec: float) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_log_time_ns.get(key, 0)
        if last_ns == 0 or (now_ns - last_ns) >= int(period_sec * 1e9):
            self._last_log_time_ns[key] = now_ns
            self.get_logger().warning(message)

    # ============================================================
    # HIGH-RATE PWM COMMAND SENDER
    # ============================================================
    def _pwm_send_step(self):
        # Gate: only send PWM if controller wants motion and odometry is valid
        if (not self.controller_enabled) or (not self.pose_initialized):
            self.publish_zero_velocity()
            self._pwm_idx = 0
            return

        if (not self.has_target) or self.target_achieved:
            self.publish_zero_velocity()
            self._pwm_idx = 0
            return

        amp = max(1e-6, float(self.pwm_pulse_rps))
        N = max(1, int(self.pwm_window_len))

        def duty_and_sign(cmd: float):
            mag = abs(float(cmd))
            duty = min(1.0, mag / amp)
            sgn = 0.0 if mag == 0.0 else (1.0 if cmd > 0.0 else -1.0)
            return duty, sgn

        dL, sL = duty_and_sign(self._last_left_cmd)
        dR, sR = duty_and_sign(self._last_right_cmd)

        onL = int(round(dL * N))
        onR = int(round(dR * N))

        sendL = sL * amp if self._pwm_idx < onL else 0.0
        sendR = sR * amp if self._pwm_idx < onR else 0.0

        self.publish_wheel_velocities(sendL, sendR)

        self._pwm_idx += 1
        if self._pwm_idx >= N:
            if self.pwm_debug_log:
                try:
                    self.get_logger().info(
                        f"PWM window: cmds(L,R)={self._last_left_cmd:.3f},{self._last_right_cmd:.3f} rps | "
                        f"duty(L,R)={dL:.2f},{dR:.2f} | amp={amp:.2f} rps | send(L,R)={sendL:.2f},{sendR:.2f}")
                except Exception:
                    pass
            self._pwm_idx = 0


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted by user')
    finally:
        # Send zero velocity before shutdown
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

