#!/usr/bin/env python3
"""
GPR Scan Controller Node
=========================
Manages synchronized GPR scanning with Fast-LIO odometry logging.

Features:
- Start/stop GPR scanning with single key command
- Controls GPR linear actuator (Arduino) via service
- Rotates GPR wheel at constant velocity
- Logs Fast-LIO odometry at high frequency (50 Hz)
- Synchronized start/stop of all components

Usage:
  ros2 run pilot_control gpr_scan_controller

Services:
  /gpr_scan/toggle (std_srvs/Trigger) - Toggle scan on/off
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
from std_srvs.srv import Trigger
import os
import time
from datetime import datetime
from collections import deque
import threading
import csv
import math
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped


class GPRScanController(Node):
    def __init__(self):
        super().__init__('gpr_scan_controller')
        
        # Declare parameters
        self.declare_parameter('gpr_wheel_radius', 0.03)      # 60mm diameter virtual wheel
        self.declare_parameter('gpr_gear_ratio', 1.0)         # Direct drive
        self.declare_parameter('velocity_multiplier', 1.0)
        self.declare_parameter('gpr_scan_velocity_mps', 0.5)  # 0.5 m/s scanning speed
        self.declare_parameter('invert_third', True)
        self.declare_parameter('fastlio_odom_topic', '/Odometry')
        self.declare_parameter('log_frequency_hz', 50.0)      # 50 Hz logging
        self.declare_parameter('log_directory', os.path.expanduser('~/gpr_scans'))
        # Fast-LIO filtering
        self.declare_parameter('fastlio_filter_window', 5)
        # Tilt correction parameters (match laser_map_rotator behavior)
        self.declare_parameter('pitch_rad', -0.2617993878)  # ~ -15 deg default fallback
        self.declare_parameter('accel_topic', '/livox/imu')
        self.declare_parameter('accel_samples', 10)
        # Namespaces and arming behavior
        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('right_ns', 'right')
        self.declare_parameter('gpr_ns', 'gpr')
        self.declare_parameter('auto_arm_on_start', True)
        
        # Get parameters
        self.gpr_wheel_radius = self.get_parameter('gpr_wheel_radius').value
        self.gpr_gear_ratio = self.get_parameter('gpr_gear_ratio').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        self.gpr_scan_velocity = self.get_parameter('gpr_scan_velocity_mps').value
        self.invert_third = self.get_parameter('invert_third').value
        self.fastlio_topic = self.get_parameter('fastlio_odom_topic').value
        self.log_freq = self.get_parameter('log_frequency_hz').value
        self.log_dir = self.get_parameter('log_directory').value
        self.fastlio_filter_window = int(self.get_parameter('fastlio_filter_window').value) or 1
        self.pitch_rad = float(self.get_parameter('pitch_rad').value)
        self.accel_topic = str(self.get_parameter('accel_topic').value)
        self.accel_samples_target = max(1, int(self.get_parameter('accel_samples').value))
        self.left_ns = self.get_parameter('left_ns').value
        self.right_ns = self.get_parameter('right_ns').value
        self.gpr_ns = self.get_parameter('gpr_ns').value
        self.auto_arm_on_start = bool(self.get_parameter('auto_arm_on_start').value)
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # State variables
        self.scanning = False
        self.scan_start_time = None
        self.current_log_file = None
        self.log_file_handle = None
        self.csv_writer = None
        self.log_count = 0
        self.lock = threading.Lock()
        self.stopping = False  # Flag for post-stop logging
        self.post_stop_time = None  # Time when motor stopped
        self.post_stop_duration = 1.5  # Log for 1.5 seconds after stop
        self.logging_active = False  # Flag for 50Hz logging
        self.start_sequence_time = None  # Time when start sequence began
        self.gpr_motor_start_delay = 0.5  # Delay before starting GPR motor (seconds)
        self.motor_enabled = False  # Gate to prevent motion before start event
        
        # GPR motor state (from ODrive feedback)
        self.gpr_position = 0.0
        self.gpr_velocity = 0.0
        self.gpr_timestamp_us = 0  # Timestamp in microseconds
        self.gpr_data_available = False

        # Cached Fast-LIO odometry (for logging at GPR tick rate)
        self.fastlio_timestamp_us = 0
        self.fastlio_pose = {
            'pos_x': 0.0, 'pos_y': 0.0, 'pos_z': 0.0,
            'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0, 'quat_w': 0.0,
            'vel_lin_x': 0.0, 'vel_lin_y': 0.0, 'vel_lin_z': 0.0,
            'vel_ang_x': 0.0, 'vel_ang_y': 0.0, 'vel_ang_z': 0.0,
        }
        # Filtered pose (moving average)
        self.fastlio_pose_filt = dict(self.fastlio_pose)
        # Buffer for filtering (stores dict snapshots)
        self.fastlio_buffer = deque(maxlen=max(1, self.fastlio_filter_window))
        
        # Event tracking
        self.current_event = None
        self.gpr_motor_started = False  # Flag to track if GPR motor has started
        # Left/Right ODrive wheel encoder positions (turns)
        self.left_position = 0.0
        self.right_position = 0.0
        
        # Timers (initialized to None, will be created when needed)
        self.motor_start_timer = None
        self.stop_timer = None
        
        # Publishers
        self.gpr_motor_pub = self.create_publisher(
            ControlMessage, '/gpr/control_message', 10)
        
        # Subscribers
        self.fastlio_sub = self.create_subscription(
            Odometry, self.fastlio_topic, self.fastlio_callback, 10)

        # IMU subscription for gravity-based tilt correction
        self.imu_sub = self.create_subscription(
            Imu, self.accel_topic, self.imu_callback, 10)

        # Tilt correction state (alignment quaternion aligning gravity to -Z)
        # Initialize with fixed pitch about Y until IMU averages are ready
        self.accel_initialized = False
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
        self.align_quat = self.quat_from_axis_angle([0.0, 1.0, 0.0], self.pitch_rad)  # (x,y,z,w)
        
        # Subscribe to GPR motor status for position/velocity feedback
        self.gpr_status_sub = self.create_subscription(
            ControllerStatus, '/gpr/controller_status', self.gpr_status_callback, 10)
        # Subscribe to left/right wheel controller status for encoder positions
        self.left_status_sub = self.create_subscription(
            ControllerStatus, '/left/controller_status', self.left_status_callback, 10)
        self.right_status_sub = self.create_subscription(
            ControllerStatus, '/right/controller_status', self.right_status_callback, 10)

        # Buffer recent GPR samples (timestamp, position, velocity) at 100 Hz
        self.gpr_samples = deque(maxlen=300)
        
        # Service clients (Arduino control)
        self.line_start_client = self.create_client(Trigger, '/gpr_line_start')
        self.line_stop_client = self.create_client(Trigger, '/gpr_line_stop')

        # ODrive Axis arming service client (GPR only)
        self.gpr_axis_client = self.create_client(AxisState, f'/{self.gpr_ns}/request_axis_state')

        # One-shot arming timer
        if self.auto_arm_on_start:
            self.arm_timer = self.create_timer(0.2, self._arm_once)
        else:
            self.arm_timer = None
        
        # Service server (toggle scan)
        self.toggle_service = self.create_service(
            Trigger, '/gpr_scan/toggle', self.toggle_scan_callback)
        
        # Timer for motor control (20 Hz)
        self.motor_timer = self.create_timer(0.05, self.update_gpr_motor)
        
        # Calculate motor velocity in turns/s
        self.gpr_circumference = 2.0 * 3.14159 * self.gpr_wheel_radius
        
        self.get_logger().info('=== GPR Scan Controller Started ===')
        self.get_logger().info(f'GPR wheel radius: {self.gpr_wheel_radius:.4f} m')
        self.get_logger().info(f'GPR wheel circumference: {self.gpr_circumference:.4f} m')
        self.get_logger().info(f'Scan velocity: {self.gpr_scan_velocity:.3f} m/s')
        self.get_logger().info(f'Log frequency: {self.log_freq:.1f} Hz')
        self.get_logger().info(f'GPR motor start delay: {self.gpr_motor_start_delay:.1f} s')
        self.get_logger().info(f'Post-stop logging duration: {self.post_stop_duration:.1f} s')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'Service available: /gpr_scan/toggle')
        self.get_logger().info('')
        self.get_logger().info('💡 Press assigned key in teleop to start/stop scanning')

    def left_status_callback(self, msg: ControllerStatus):
        try:
            self.left_position = float(msg.pos_estimate)
        except Exception:
            pass

    def right_status_callback(self, msg: ControllerStatus):
        try:
            self.right_position = float(msg.pos_estimate)
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
        # Hamilton product (x,y,z,w)
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        return (x, y, z, w)

    @staticmethod
    def quat_conjugate(q):
        x, y, z, w = q
        return (-x, -y, -z, w)

    @staticmethod
    def rotate_vector_by_quat(v, q):
        # v: 3-vector, q: (x,y,z,w)
        x, y, z = v
        qx, qy, qz, qw = q
        # Compute q * v * q_conj
        # Optimize without creating full quaternions
        # t = 2 * cross(q_vec, v)
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        # v' = v + qw * t + cross(q_vec, t)
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
            # 180-degree rotation around any axis orthogonal to v1
            axis = np.array([1.0, 0.0, 0.0])
            if abs(v1[0]) > 0.9:
                axis = np.array([0.0, 1.0, 0.0])
            axis = axis - v1 * v1.dot(axis)
            if np.linalg.norm(axis) < 1e-12:
                axis = np.array([0.0, 0.0, 1.0])
            axis = axis / (np.linalg.norm(axis) + 1e-12)
            return GPRScanController.quat_from_axis_angle(axis, math.pi)
        axis = np.cross(v1, v2)
        s = math.sqrt((1.0 + cos_theta) * 2.0)
        invs = 1.0 / s
        w = 0.5 * s
        x = axis[0] * invs
        y = axis[1] * invs
        z = axis[2] * invs
        return GPRScanController.quat_normalize((x, y, z, w))

    def imu_callback(self, msg: Imu):
        if self.accel_initialized:
            return
        a = np.array([
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z)
        ], dtype=float)
        if not np.all(np.isfinite(a)):
            return
        # Match laser_map_rotator axis conventions
        a[0] = -a[0]
        a[2] = -a[2]
        self.accel_sum += a
        self.accel_count += 1
        if self.accel_count < self.accel_samples_target:
            return
        avg = self.accel_sum / float(self.accel_count)
        norm = np.linalg.norm(avg)
        if norm < 1e-3:
            self.get_logger().warn('Average acceleration magnitude too small; waiting for more samples...')
            return
        # Align avg acceleration to -Z
        q_align = self.compute_alignment_quat(avg, np.array([0.0, 0.0, -1.0]))
        self.align_quat = q_align
        self.accel_initialized = True
        self.get_logger().info(
            f'IMU tilt correction initialized with {self.accel_count} samples; '
            f'align_quat=(x={q_align[0]:.4f}, y={q_align[1]:.4f}, z={q_align[2]:.4f}, w={q_align[3]:.4f})'
        )

    def _arm_once(self):
        """Attempt to arm GPR axis into CLOSED_LOOP once service is available."""
        try:
            if not self.gpr_axis_client.wait_for_service(timeout_sec=0.1):
                return

            req = AxisState.Request()
            req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            self.gpr_axis_client.call_async(req)

            if self.arm_timer is not None:
                self.arm_timer.cancel()
                self.arm_timer = None
            self.get_logger().info('Sent CLOSED_LOOP arming to /gpr axis')
        except Exception as exc:
            self.get_logger().warn(f"Arming failed: {exc}")
    
    def gpr_status_callback(self, msg):
        """Store latest GPR motor status"""
        self.gpr_position = msg.pos_estimate  # turns
        self.gpr_velocity = msg.vel_estimate  # turns/s
        # Use acquisition timestamp from the CAN node (microseconds since epoch, ROS time domain)
        try:
            self.gpr_timestamp_us = int(getattr(msg, 'stamp_us'))
        except Exception as e:
            self.get_logger().warn(f"GPR stamp_us unavailable; using callback time. Error: {e}")
            current_time = self.get_clock().now()
            seconds, nanoseconds = current_time.seconds_nanoseconds()
            self.gpr_timestamp_us = seconds * 1_000_000 + nanoseconds // 1000
        self.gpr_data_available = True
        
        # Buffer this GPR sample for nearest-neighbor matching on Fast-LIO ticks
        try:
            self.gpr_samples.append((self.gpr_timestamp_us, self.gpr_position, self.gpr_velocity))
        except Exception:
            pass

        # 100 Hz logging on GPR status callback
        if not self.logging_active or not self.csv_writer:
            return

        # Determine event
        if self.current_event:
            event = self.current_event
            self.current_event = None
        elif self.stopping:
            event = 'POST_STOP'
        elif not self.motor_enabled:
            event = 'PRE_MOTOR'
        elif self.scanning:
            event = 'SCANNING'
        else:
            event = 'UNKNOWN'

        # Use latest Fast-LIO timestamp (may be older than GPR stamp)
        fast_ts = self.fastlio_timestamp_us

        try:
            data_row = [
                self.log_count,
                event,
                fast_ts,
                self.gpr_timestamp_us,
                f"{self.fastlio_pose_filt['pos_x']:.6f}",
                f"{self.fastlio_pose_filt['pos_y']:.6f}",
                f"{self.fastlio_pose_filt['pos_z']:.6f}",
                f"{self.fastlio_pose_filt['quat_x']:.6f}",
                f"{self.fastlio_pose_filt['quat_y']:.6f}",
                f"{self.fastlio_pose_filt['quat_z']:.6f}",
                f"{self.fastlio_pose_filt['quat_w']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_x']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_y']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_z']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_x']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_y']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_z']:.6f}",
                f'{self.gpr_position:.6f}',
                f'{self.gpr_velocity:.6f}',
                f'{self.left_position:.6f}',
                f'{self.right_position:.6f}',
            ]
            self.csv_writer.writerow(data_row)
            self.log_count += 1
        except Exception as e:
            self.get_logger().error(f'Error logging GPR row: {e}')
    
    def toggle_scan_callback(self, request, response):
        """Toggle GPR scanning on/off"""
        with self.lock:
            if not self.scanning:
                # Start scanning
                success = self.start_scan()
                if success:
                    response.success = True
                    response.message = f'GPR scan started. Logging to: {self.current_log_file}'
                else:
                    response.success = False
                    response.message = 'Failed to start GPR scan'
            else:
                # Stop scanning
                self.stop_scan()
                response.success = True
                response.message = f'GPR scan stopped. Logged {self.log_count} samples'
        
        return response
    
    def start_scan(self):
        """Start GPR scanning sequence with continuous 50Hz logging"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('STARTING GPR SCAN')
        self.get_logger().info('='*70)
        
        # Step 1: Create log file (CSV format for better performance)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_log_file = os.path.join(self.log_dir, f'gpr_scan_{timestamp}.csv')
        
        try:
            # Open with line buffering for minimal latency
            self.log_file_handle = open(self.current_log_file, 'w', buffering=1)
            
            # Write metadata as comments
            self.log_file_handle.write(f'# GPR Scan Data - Comprehensive High-Frequency Logging\n')
            self.log_file_handle.write(f'# Start Time: {timestamp}\n')
            self.log_file_handle.write(f'# GPR Velocity (m/s): {self.gpr_scan_velocity}\n')
            self.log_file_handle.write(f'# Log Frequency (Hz): {self.log_freq}\n')
            self.log_file_handle.write(f'# GPR Wheel Radius (m): {self.gpr_wheel_radius}\n')
            self.log_file_handle.write(f'# Fast-LIO Topic: {self.fastlio_topic}\n')
            self.log_file_handle.write(f'# GPR Motor Start Delay: {self.gpr_motor_start_delay} s\n')
            self.log_file_handle.write(f'# Post-Stop Duration: {self.post_stop_duration} s\n')
            self.log_file_handle.write(f'#\n')
            self.log_file_handle.write(f'# Timestamp Format: Microseconds (µs) since epoch\n')
            self.log_file_handle.write(f'# Time Sync: Both fastlio_time_us and gpr_time_us use ROS time\n')
            self.log_file_handle.write(f'#\n')
            self.log_file_handle.write(f'# Event Types:\n')
            self.log_file_handle.write(f'#   KEY_PRESS_START - Scan key pressed, 50Hz logging begins\n')
            self.log_file_handle.write(f'#   ARDUINO_START - Linear actuator starting\n')
            self.log_file_handle.write(f'#   PRE_MOTOR - Waiting for motor start (50Hz logging active)\n')
            self.log_file_handle.write(f'#   GPR_MOTOR_START - GPR motor velocity command sent\n')
            self.log_file_handle.write(f'#   SCANNING - Normal scanning operation (50Hz logging)\n')
            self.log_file_handle.write(f'#   KEY_PRESS_STOP - Stop key pressed (50Hz continues)\n')
            self.log_file_handle.write(f'#   MOTOR_STOPPING - Stopping motor (50Hz continues)\n')
            self.log_file_handle.write(f'#   POST_STOP - After motor stopped (50Hz continues)\n')
            self.log_file_handle.write(f'#   ARDUINO_STOP - Linear actuator stopping\n')
            self.log_file_handle.write(f'#\n')
            
            # Create CSV writer
            self.csv_writer = csv.writer(self.log_file_handle, lineterminator='\n')
            
            # Write header row
            self.csv_writer.writerow([
                'seq', 'event', 'fastlio_time_us', 'gpr_time_us',
                'pos_x', 'pos_y', 'pos_z',
                'quat_x', 'quat_y', 'quat_z', 'quat_w',
                'vel_lin_x', 'vel_lin_y', 'vel_lin_z',
                'vel_ang_x', 'vel_ang_y', 'vel_ang_z',
                'gpr_position', 'gpr_velocity',
                'left_position', 'right_position'
            ])
            
            self.get_logger().info(f'✓ Log file created: {self.current_log_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {e}')
            return False
        
        # Step 2: Log KEY_PRESS_START and enable 50Hz logging IMMEDIATELY
        #self.current_event = 'KEY_PRESS_START'
        self.logging_active = True
        self.start_sequence_time = time.time()
        self.gpr_motor_started = False
        self.log_count = 0  # Reset log count for new scan
        # Immediate event log
        self.log_event_now('KEY_PRESS_START')
        self.get_logger().info('✓ KEY_PRESS_START logged - 50Hz logging ACTIVE')
        
        # Step 3: Start linear actuator (Arduino - line up) - non-blocking
        self.get_logger().info('⏳ Starting linear actuator...')
        #self.current_event = 'ARDUINO_START'
        # Immediate event log
        self.log_event_now('ARDUINO_START')
        self.current_event = None
        if self.line_start_client.wait_for_service(timeout_sec=1.0):
            future = self.line_start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.get_logger().info('✓ Linear actuator started')
            else:
                self.get_logger().warn('⚠️  Linear actuator service call failed')
        else:
            self.get_logger().warn('⚠️  Linear actuator service not available')
        
        # Step 4: Start timer to trigger GPR motor after delay
        # During this time, 50Hz logging continues with PRE_MOTOR event
        self.get_logger().info(f'⏳ 50Hz logging active, waiting {self.gpr_motor_start_delay}s before starting GPR motor...')
        
        # Use a timer to start the motor after delay (non-blocking)
        self.motor_start_timer = self.create_timer(
            self.gpr_motor_start_delay, 
            self.delayed_gpr_motor_start
        )
        
        # Set initial state
        self.scanning = True
        self.scan_start_time = time.time()
        
        self.get_logger().info('')
        self.get_logger().info('🟢 GPR SCAN SEQUENCE STARTED - 50Hz LOGGING ACTIVE')
        self.get_logger().info('='*70)
        
        return True
    
    def delayed_gpr_motor_start(self):
        """Start GPR motor after delay - called by timer"""
        # Cancel the one-shot timer
        if self.motor_start_timer:
            self.motor_start_timer.cancel()
            self.motor_start_timer = None
        
        # Log GPR_MOTOR_START event, then enable motor publishing
        self.log_event_now('GPR_MOTOR_START')
        self.current_event = None
        self.motor_enabled = True
        self.get_logger().info('⏳ Starting GPR motor rotation...')
        
        # Calculate motor velocity in turns/s
        wheel_turns_per_sec = self.gpr_scan_velocity / self.gpr_circumference
        motor_turns_per_sec = wheel_turns_per_sec * self.gpr_gear_ratio * self.velocity_multiplier
        
        # Apply inversion if needed
        if self.invert_third:
            motor_turns_per_sec = -motor_turns_per_sec
        
        # Send velocity command
        msg = ControlMessage()
        msg.control_mode = 2  # VELOCITY_CONTROL
        msg.input_mode = 2    # VEL_RAMP
        msg.input_vel = motor_turns_per_sec
        msg.input_torque = 0.0
        msg.input_pos = 0.0
        
        self.gpr_motor_pub.publish(msg)
        self.gpr_motor_started = True
        self.get_logger().info(f'✓ GPR motor started at {self.gpr_scan_velocity:.3f} m/s ({motor_turns_per_sec:.3f} turns/s)')
        self.get_logger().info('✓ Full scanning active - 50Hz logging continues')
    
    def stop_scan(self):
        """Stop GPR scanning sequence with continuous 50Hz logging through stop"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('STOPPING GPR SCAN')
        self.get_logger().info('='*70)
        
        # Step 1: Log KEY_PRESS_STOP (50Hz logging continues)
        #self.current_event = 'KEY_PRESS_STOP'
        self.log_event_now('KEY_PRESS_STOP')
        self.current_event = None
        self.get_logger().info('✓ KEY_PRESS_STOP logged - 50Hz logging continues')
        
        # Step 2: Log MOTOR_STOPPING and stop GPR motor
        #self.current_event = 'MOTOR_STOPPING'
        self.log_event_now('MOTOR_STOPPING')
        self.current_event = None
        self.scanning = False
        self.stopping = True  # Enable post-stop logging phase
        self.post_stop_time = time.time()  # Mark when motor stopped
        
        # Send zero velocity command
        msg = ControlMessage()
        msg.control_mode = 2  # VELOCITY_CONTROL
        msg.input_mode = 1    # PASSTHROUGH
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        msg.input_pos = 0.0
        
        self.gpr_motor_pub.publish(msg)
        self.get_logger().info('✓ GPR motor velocity set to ZERO')
        self.get_logger().info(f'⏳ Continuing 50Hz logging for {self.post_stop_duration}s...')
        
        # Step 3: Wait for post-stop logging duration (50Hz continues)
        # Use a timer to trigger the final cleanup after post-stop duration
        self.stop_timer = self.create_timer(
            self.post_stop_duration,
            self.delayed_stop_cleanup
        )
    
    def delayed_stop_cleanup(self):
        """Complete stop sequence after post-stop logging - called by timer"""
        # Cancel the one-shot timer
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
        
        self.get_logger().info('✓ Post-stop logging duration complete')
        
        # Log ARDUINO_STOP event before stopping actuator
        #self.current_event = 'ARDUINO_STOP'
        self.log_event_now('ARDUINO_STOP')
        self.current_event = None
        
        # Step 4: Stop linear actuator
        self.get_logger().info('⏳ Stopping linear actuator...')
        if self.line_stop_client.wait_for_service(timeout_sec=1.0):
            future = self.line_stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.get_logger().info('✓ Linear actuator stopped')
            else:
                self.get_logger().warn('⚠️  Linear actuator stop failed')
        else:
            self.get_logger().warn('⚠️  Linear actuator service not available')
        
        # Step 5: Disable logging and close log file
        self.stopping = False
        self.logging_active = False
        
        if self.log_file_handle:
            try:
                self.log_file_handle.close()
                self.log_file_handle = None
                self.csv_writer = None
                
                scan_duration = time.time() - self.scan_start_time if self.scan_start_time else 0
                
                self.get_logger().info(f'✓ Log file closed: {self.current_log_file}')
                self.get_logger().info(f'📊 Scan statistics:')
                self.get_logger().info(f'   Total duration: {scan_duration:.1f} seconds')
                self.get_logger().info(f'   Samples logged: {self.log_count}')
                if scan_duration > 0:
                    self.get_logger().info(f'   Average rate: {self.log_count/scan_duration:.1f} Hz')
            except Exception as e:
                self.get_logger().error(f'Error closing log file: {e}')
        
        self.get_logger().info('')
        self.get_logger().info('🔴 GPR SCAN STOPPED - Logging ended')
        self.get_logger().info('='*70)
    
    def update_gpr_motor(self):
        """Send velocity command to GPR motor"""
        if self.scanning and self.motor_enabled:
            # Calculate motor velocity in turns/s
            wheel_turns_per_sec = self.gpr_scan_velocity / self.gpr_circumference
            motor_turns_per_sec = wheel_turns_per_sec * self.gpr_gear_ratio * self.velocity_multiplier
            
            # Apply inversion if needed
            if self.invert_third:
                motor_turns_per_sec = -motor_turns_per_sec
            
            # Send velocity command
            msg = ControlMessage()
            msg.control_mode = 2  # VELOCITY_CONTROL
            msg.input_mode = 2    # VEL_RAMP
            msg.input_vel = -motor_turns_per_sec
            msg.input_torque = 0.0
            msg.input_pos = 0.0
            
            self.gpr_motor_pub.publish(msg)
        else:
            # Send zero velocity when not scanning
            msg = ControlMessage()
            msg.control_mode = 2  # VELOCITY_CONTROL
            msg.input_mode = 1    # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            msg.input_pos = 0.0
            
            self.gpr_motor_pub.publish(msg)
    
    def fastlio_callback(self, msg):
        """On each Fast-LIO odometry, update filtered pose (logging moved to GPR callback)."""
        # Use measurement time from header (ROS time domain)
        self.fastlio_timestamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000

        # Apply gravity-based tilt correction immediately to pose
        raw_pos = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z)
        ], dtype=float)
        raw_q = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        q_align = self.align_quat  # (x,y,z,w)
        # Rotate position and orientation into corrected frame
        corr_pos = self.rotate_vector_by_quat(raw_pos, q_align)
        corr_q = self.quat_multiply(q_align, raw_q)
        corr_q = self.quat_normalize(corr_q)

        # Cache corrected pose and twist
        self.fastlio_pose['pos_x']     = float(corr_pos[0])
        self.fastlio_pose['pos_y']     = float(corr_pos[1])
        self.fastlio_pose['pos_z']     = float(corr_pos[2])
        self.fastlio_pose['quat_x']    = corr_q[0]
        self.fastlio_pose['quat_y']    = corr_q[1]
        self.fastlio_pose['quat_z']    = corr_q[2]
        self.fastlio_pose['quat_w']    = corr_q[3]
        self.fastlio_pose['vel_lin_x'] = msg.twist.twist.linear.x
        self.fastlio_pose['vel_lin_y'] = msg.twist.twist.linear.y
        self.fastlio_pose['vel_lin_z'] = msg.twist.twist.linear.z
        self.fastlio_pose['vel_ang_x'] = msg.twist.twist.angular.x
        self.fastlio_pose['vel_ang_y'] = msg.twist.twist.angular.y
        self.fastlio_pose['vel_ang_z'] = msg.twist.twist.angular.z

        # Update moving average buffer
        try:
            self.fastlio_buffer.append(dict(self.fastlio_pose))
            n = len(self.fastlio_buffer)
            if n > 0:
                # Average scalar components
                def avg(key):
                    return sum(sample[key] for sample in self.fastlio_buffer) / float(n)
                self.fastlio_pose_filt['pos_x']     = avg('pos_x')
                self.fastlio_pose_filt['pos_y']     = avg('pos_y')
                self.fastlio_pose_filt['pos_z']     = avg('pos_z')
                self.fastlio_pose_filt['vel_lin_x'] = avg('vel_lin_x')
                self.fastlio_pose_filt['vel_lin_y'] = avg('vel_lin_y')
                self.fastlio_pose_filt['vel_lin_z'] = avg('vel_lin_z')
                self.fastlio_pose_filt['vel_ang_x'] = avg('vel_ang_x')
                self.fastlio_pose_filt['vel_ang_y'] = avg('vel_ang_y')
                self.fastlio_pose_filt['vel_ang_z'] = avg('vel_ang_z')
                # Average quaternion and renormalize
                qx = avg('quat_x')
                qy = avg('quat_y')
                qz = avg('quat_z')
                qw = avg('quat_w')
                norm = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
                if norm > 1e-9:
                    self.fastlio_pose_filt['quat_x'] = qx / norm
                    self.fastlio_pose_filt['quat_y'] = qy / norm
                    self.fastlio_pose_filt['quat_z'] = qz / norm
                    self.fastlio_pose_filt['quat_w'] = qw / norm
                else:
                    # Fallback to latest raw quaternion
                    self.fastlio_pose_filt['quat_x'] = self.fastlio_pose['quat_x']
                    self.fastlio_pose_filt['quat_y'] = self.fastlio_pose['quat_y']
                    self.fastlio_pose_filt['quat_z'] = self.fastlio_pose['quat_z']
                    self.fastlio_pose_filt['quat_w'] = self.fastlio_pose['quat_w']
        except Exception:
            # On any error, fall back to raw
            self.fastlio_pose_filt = dict(self.fastlio_pose)

        # Logging now happens in gpr_status_callback at ~100 Hz

    def log_event_now(self, event: str):
        """Write an immediate event row using latest cached data (no waiting for next tick)."""
        if not self.csv_writer or not self.logging_active:
            return

        try:
            # Target timestamp to align with: prefer latest Fast-LIO stamp; fallback to now
            target_ts = self.fastlio_timestamp_us
            if target_ts == 0:
                now = self.get_clock().now()
                s, ns = now.seconds_nanoseconds()
                target_ts = s * 1_000_000 + ns // 1000

            # Nearest-neighbor GPR sample to the target timestamp
            g_ts = 0
            g_pos = 0.0
            g_vel = 0.0
            if self.gpr_samples:
                try:
                    nearest = min(self.gpr_samples, key=lambda s: abs(s[0] - target_ts))
                    g_ts, g_pos, g_vel = nearest
                except Exception:
                    pass

            data_row = [
                self.log_count,
                event,
                self.fastlio_timestamp_us if self.fastlio_timestamp_us != 0 else target_ts,
                g_ts,
                f"{self.fastlio_pose_filt['pos_x']:.6f}",
                f"{self.fastlio_pose_filt['pos_y']:.6f}",
                f"{self.fastlio_pose_filt['pos_z']:.6f}",
                f"{self.fastlio_pose_filt['quat_x']:.6f}",
                f"{self.fastlio_pose_filt['quat_y']:.6f}",
                f"{self.fastlio_pose_filt['quat_z']:.6f}",
                f"{self.fastlio_pose_filt['quat_w']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_x']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_y']:.6f}",
                f"{self.fastlio_pose_filt['vel_lin_z']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_x']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_y']:.6f}",
                f"{self.fastlio_pose_filt['vel_ang_z']:.6f}",
                f'{g_pos:.6f}',
                f'{g_vel:.6f}'
            ]
            self.csv_writer.writerow(data_row)
            self.log_count += 1
        except Exception as e:
            self.get_logger().error(f'Error logging event: {e}')
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if self.logging_active:
            # If logging is active, we need to clean up
            self.logging_active = False
            if self.log_file_handle:
                try:
                    self.log_file_handle.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = GPRScanController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted by user')
        if node.logging_active:
            # Clean up logging if still active
            node.logging_active = False
            if node.log_file_handle:
                node.log_file_handle.close()
                node.get_logger().info('✓ Log file closed due to interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

