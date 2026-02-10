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
import subprocess
import signal
import shutil


class GPRScanController(Node):
    def __init__(self):
        super().__init__('gpr_scan_controller')
        
        # Declare parameters
        self.declare_parameter('gpr_wheel_radius', 0.03)      # 60mm diameter virtual wheel
        self.declare_parameter('gpr_gear_ratio', 1.0)         # Direct drive
        self.declare_parameter('velocity_multiplier', 1.0)
        self.declare_parameter('gpr_scan_velocity_mps', 0.5)  # 0.5 m/s scanning speed
        self.declare_parameter('invert_third', True)
        self.declare_parameter('fastlio_odom_topic', '/Odometry_tilt_corrected_diff')  # Use tilt-corrected odometry from diff_drive_controller
        self.declare_parameter('log_frequency_hz', 50.0)      # 50 Hz logging
        self.declare_parameter('log_directory', os.path.expanduser('~/gpr_scans'))
        self.declare_parameter('gpr_scan_data_directory', '')  # Session-specific GPR_scan_data folder
        # Fast-LIO filtering
        self.declare_parameter('fastlio_filter_window', 5)
        # Namespaces and arming behavior
        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('right_ns', 'right')
        self.declare_parameter('gpr_ns', 'gpr')
        self.declare_parameter('auto_arm_on_start', True)
        # Rosbag recording parameters (no point clouds - keep it lightweight)
        self.declare_parameter('rosbag_topics', [
            '/Odometry',
            '/Odometry_tilt_corrected_diff',
            '/cmd_vel',
            '/left/controller_status',
            '/right/controller_status',
            '/gpr/controller_status',
            '/path',
            '/projected_map',
            '/tf',
            '/tf_static',
        ])
        
        # Get parameters
        self.gpr_wheel_radius = self.get_parameter('gpr_wheel_radius').value
        self.gpr_gear_ratio = self.get_parameter('gpr_gear_ratio').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        self.gpr_scan_velocity = self.get_parameter('gpr_scan_velocity_mps').value
        self.invert_third = self.get_parameter('invert_third').value
        self.fastlio_topic = self.get_parameter('fastlio_odom_topic').value
        self.log_freq = self.get_parameter('log_frequency_hz').value
        self.log_dir = self.get_parameter('log_directory').value
        
        # Check for session-specific GPR scan data directory
        gpr_scan_data_dir = self.get_parameter('gpr_scan_data_directory').value
        if gpr_scan_data_dir:
            self.log_dir = gpr_scan_data_dir  # Override with session-specific path
        
        # Rosbag directory: Use RAM disk for recording, copy to permanent storage later
        # With 32GB RAM, we can buffer rosbags in memory to avoid disk I/O bottleneck
        self.rosbag_dir_final = os.path.dirname(self.log_dir) if gpr_scan_data_dir else self.log_dir
        self.rosbag_dir = '/tmp/rosbag_recording'  # RAM disk location
        os.makedirs(self.rosbag_dir, exist_ok=True)
        
        self.fastlio_filter_window = int(self.get_parameter('fastlio_filter_window').value) or 1
        self.left_ns = self.get_parameter('left_ns').value
        self.right_ns = self.get_parameter('right_ns').value
        self.gpr_ns = self.get_parameter('gpr_ns').value
        self.auto_arm_on_start = bool(self.get_parameter('auto_arm_on_start').value)
        self.rosbag_topics = self.get_parameter('rosbag_topics').value
        
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
        self.paused = False  # Pause state for data collection coordinator
        self.rosbag_paused = False  # Rosbag pause state
        
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
        # Last Fast-LIO timestamp that was written to CSV (to avoid duplicates)
        self.last_fastlio_logged_ts_us = 0
        
        # Rosbag recording state
        self.rosbag_recording = False
        self.rosbag_process = None
        self.rosbag_path = None
        
        # Timers (initialized to None, will be created when needed)
        self.motor_start_timer = None
        self.stop_timer = None
        
        # Publishers
        self.gpr_motor_pub = self.create_publisher(
            ControlMessage, '/gpr/control_message', 10)
        
        # Subscribers
        self.fastlio_sub = self.create_subscription(
            Odometry, self.fastlio_topic, self.fastlio_callback, 10)
        
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
        self.power_off_client = self.create_client(Trigger, '/gpr_power_off')

        # ODrive Axis arming service client (GPR only)
        self.gpr_axis_client = self.create_client(AxisState, f'/{self.gpr_ns}/request_axis_state')

        # One-shot arming timer
        if self.auto_arm_on_start:
            self.arm_timer = self.create_timer(0.2, self._arm_once)
        else:
            self.arm_timer = None
        
        # Service servers
        self.toggle_service = self.create_service(
            Trigger, '/gpr_scan/toggle', self.toggle_scan_callback)
        self.rosbag_toggle_service = self.create_service(
            Trigger, '/rosbag/toggle', self.rosbag_toggle_callback)
        self.power_off_service = self.create_service(
            Trigger, '/gpr_scan/power_off', self.power_off_callback)
        
        # Start/Pause/Resume/Stop services for data collection coordinator
        self.start_service = self.create_service(
            Trigger, '/gpr_scan/start', self.start_callback)
        self.pause_service = self.create_service(
            Trigger, '/gpr_scan/pause', self.pause_callback)
        self.resume_service = self.create_service(
            Trigger, '/gpr_scan/resume', self.resume_callback)
        self.stop_service = self.create_service(
            Trigger, '/gpr_scan/stop', self.stop_callback)
        
        # Rosbag start/pause/resume/stop services
        self.rosbag_start_service = self.create_service(
            Trigger, '/rosbag/start', self.rosbag_start_callback)
        self.rosbag_pause_service = self.create_service(
            Trigger, '/rosbag/pause', self.rosbag_pause_callback)
        self.rosbag_resume_service = self.create_service(
            Trigger, '/rosbag/resume', self.rosbag_resume_callback)
        self.rosbag_stop_service = self.create_service(
            Trigger, '/rosbag/stop', self.rosbag_stop_callback)
        
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
        self.get_logger().info(f'GPR CSV directory: {self.log_dir}')
        self.get_logger().info(f'Rosbag RAM disk: {self.rosbag_dir}')
        self.get_logger().info(f'Rosbag final destination: {self.rosbag_dir_final}')
        self.get_logger().info(f'Services available:')
        self.get_logger().info(f'  - /gpr_scan/toggle')
        self.get_logger().info(f'  - /gpr_scan/power_off')
        self.get_logger().info(f'  - /gpr_scan/start, /gpr_scan/pause, /gpr_scan/resume, /gpr_scan/stop')
        self.get_logger().info(f'  - /rosbag/toggle')
        self.get_logger().info(f'  - /rosbag/start, /rosbag/pause, /rosbag/resume, /rosbag/stop')
        self.get_logger().info('')
        self.get_logger().info('💡 Press G in teleop to start/stop GPR scanning')
        self.get_logger().info('💡 Press B in teleop to start/stop rosbag recording')

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

        # 100 Hz trigger on GPR status; log only when new Fast-LIO data is available,
        # except for immediate event rows which we force to log on this tick.
        # Skip logging if paused
        if not self.logging_active or not self.csv_writer or self.paused:
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

        # Decide how to fill Fast-LIO fields: if no new sample, leave Fast-LIO fields empty
        # but still log GPR/encoder data.
        new_fastlio = (fast_ts != 0 and fast_ts != self.last_fastlio_logged_ts_us)

        try:
            data_row = [
                self.log_count,
                event,
                (fast_ts if new_fastlio else ''),
                self.gpr_timestamp_us,
                (f"{self.fastlio_pose['pos_x']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['pos_y']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['pos_z']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['quat_x']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['quat_y']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['quat_z']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['quat_w']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_lin_x']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_lin_y']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_lin_z']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_ang_x']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_ang_y']:.6f}" if new_fastlio else ''),
                (f"{self.fastlio_pose['vel_ang_z']:.6f}" if new_fastlio else ''),
                f'{self.gpr_position:.6f}',
                f'{self.gpr_velocity:.6f}',
                f'{self.left_position:.6f}',
                f'{self.right_position:.6f}',
            ]
            self.csv_writer.writerow(data_row)
            self.log_count += 1
            if new_fastlio:
                self.last_fastlio_logged_ts_us = fast_ts
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
    
    def power_off_callback(self, request, response):
        """Power off GPR via Arduino"""
        self.get_logger().info('Sending GPR power off command...')
        if self.power_off_client.wait_for_service(timeout_sec=1.0):
            future = self.power_off_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                response.success = True
                response.message = 'GPR power off command sent'
                self.get_logger().info('✓ GPR power off command sent')
            else:
                response.success = False
                response.message = 'GPR power off service call failed'
                self.get_logger().warn('⚠️  GPR power off service call failed')
        else:
            response.success = False
            response.message = 'GPR power off service not available'
            self.get_logger().warn('⚠️  GPR power off service not available')
        return response
    
    def rosbag_toggle_callback(self, request, response):
        """Toggle rosbag recording on/off"""
        with self.lock:
            if not self.rosbag_recording:
                # Start recording
                success = self.start_rosbag_recording()
                if success:
                    response.success = True
                    response.message = f'Rosbag recording started: {self.rosbag_path}'
                else:
                    response.success = False
                    response.message = 'Failed to start rosbag recording'
            else:
                # Stop recording
                success = self.stop_rosbag_recording()
                if success:
                    response.success = True
                    response.message = f'Rosbag recording stopped: {self.rosbag_path}'
                else:
                    response.success = False
                    response.message = 'Failed to stop rosbag recording'
        
        return response
    
    # ==================== Start/Pause/Resume/Stop Services ====================
    def start_callback(self, request, response):
        """Start GPR scanning (called by coordinator) - same as toggle when not scanning"""
        with self.lock:
            if self.scanning:
                response.success = True
                response.message = 'GPR scan already running'
                return response
            
            # Start scanning
            success = self.start_scan()
            if success:
                response.success = True
                response.message = f'GPR scan started. Logging to: {self.current_log_file}'
            else:
                response.success = False
                response.message = 'Failed to start GPR scan'
        
        return response
    
    def pause_callback(self, request, response):
        """Pause GPR scanning - stop motor and pause CSV logging"""
        with self.lock:
            if self.paused:
                response.success = True
                response.message = 'Already paused'
                return response
            
            self.paused = True
            
            # Stop GPR motor (set velocity to zero)
            msg = ControlMessage()
            msg.control_mode = 2  # VELOCITY_CONTROL
            msg.input_mode = 1    # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            msg.input_pos = 0.0
            self.gpr_motor_pub.publish(msg)
            
            self.get_logger().info('='*60)
            self.get_logger().info('GPR SCAN PAUSED - Motor stopped, CSV logging paused')
            self.get_logger().info('='*60)
            
            response.success = True
            response.message = 'GPR scan paused'
        return response
    
    def resume_callback(self, request, response):
        """Resume GPR scanning - restart motor and resume CSV logging"""
        with self.lock:
            if not self.paused:
                response.success = True
                response.message = 'Not paused'
                return response
            
            self.paused = False
            
            self.get_logger().info('='*60)
            self.get_logger().info('GPR SCAN RESUMED - Motor and CSV logging resumed')
            self.get_logger().info('='*60)
            
            response.success = True
            response.message = 'GPR scan resumed'
        return response
    
    def stop_callback(self, request, response):
        """Stop GPR scanning completely - close files, lower actuator, and save"""
        with self.lock:
            self.get_logger().info('='*60)
            self.get_logger().info('GPR SCAN STOPPING (via coordinator)')
            self.get_logger().info('='*60)
            
            # Stop motor
            msg = ControlMessage()
            msg.control_mode = 2  # VELOCITY_CONTROL
            msg.input_mode = 1    # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            msg.input_pos = 0.0
            self.gpr_motor_pub.publish(msg)
            self.get_logger().info('  ✓ GPR motor stopped')
            
            # Reset state
            self.scanning = False
            self.paused = False
            self.logging_active = False
            self.motor_enabled = False
            
            # Close log file
            if self.log_file_handle:
                try:
                    self.log_file_handle.close()
                    self.get_logger().info(f'  ✓ GPR CSV file closed: {self.current_log_file}')
                except Exception as e:
                    self.get_logger().error(f'  ✗ Error closing log file: {e}')
                self.log_file_handle = None
                self.csv_writer = None
            
            # Lower linear actuator (line down)
            self.get_logger().info('  Lowering linear actuator...')
            if self.line_stop_client.wait_for_service(timeout_sec=1.0):
                future = self.line_stop_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() and future.result().success:
                    self.get_logger().info('  ✓ Linear actuator lowered')
                else:
                    self.get_logger().warn('  ⚠️  Linear actuator stop failed')
            else:
                self.get_logger().warn('  ⚠️  Linear actuator service not available')
            
            response.success = True
            response.message = f'GPR scan stopped. Logged {self.log_count} samples'
            self.get_logger().info(f'✓ GPR scan STOPPED - {self.log_count} samples logged')
            self.get_logger().info('='*60)
        return response
    
    def rosbag_start_callback(self, request, response):
        """Start rosbag recording (called by coordinator)"""
        with self.lock:
            if self.rosbag_recording:
                response.success = True
                response.message = f'Rosbag already recording: {self.rosbag_path}'
                return response
            
            # Start recording
            success = self.start_rosbag_recording()
            if success:
                response.success = True
                response.message = f'Rosbag recording started: {self.rosbag_path}'
            else:
                response.success = False
                response.message = 'Failed to start rosbag recording'
        
        return response
    
    def rosbag_pause_callback(self, request, response):
        """Pause rosbag recording by sending SIGSTOP"""
        with self.lock:
            if not self.rosbag_recording or self.rosbag_process is None:
                response.success = False
                response.message = 'No rosbag recording active'
                return response
            
            if self.rosbag_paused:
                response.success = True
                response.message = 'Rosbag already paused'
                return response
            
            try:
                # Send SIGSTOP to pause the process
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGSTOP)
                self.rosbag_paused = True
                self.get_logger().info('✓ Rosbag recording PAUSED')
                response.success = True
                response.message = 'Rosbag recording paused'
            except Exception as e:
                self.get_logger().error(f'Failed to pause rosbag: {e}')
                response.success = False
                response.message = f'Failed to pause rosbag: {e}'
        return response
    
    def rosbag_resume_callback(self, request, response):
        """Resume rosbag recording by sending SIGCONT"""
        with self.lock:
            if not self.rosbag_recording or self.rosbag_process is None:
                response.success = False
                response.message = 'No rosbag recording active'
                return response
            
            if not self.rosbag_paused:
                response.success = True
                response.message = 'Rosbag not paused'
                return response
            
            try:
                # Send SIGCONT to resume the process
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGCONT)
                self.rosbag_paused = False
                self.get_logger().info('✓ Rosbag recording RESUMED')
                response.success = True
                response.message = 'Rosbag recording resumed'
            except Exception as e:
                self.get_logger().error(f'Failed to resume rosbag: {e}')
                response.success = False
                response.message = f'Failed to resume rosbag: {e}'
        return response
    
    def rosbag_stop_callback(self, request, response):
        """Stop rosbag recording and save"""
        with self.lock:
            if not self.rosbag_recording:
                response.success = True
                response.message = 'No rosbag recording active'
                return response
            
            # Resume if paused before stopping
            if self.rosbag_paused and self.rosbag_process is not None:
                try:
                    os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGCONT)
                    self.rosbag_paused = False
                except:
                    pass
            
            success = self.stop_rosbag_recording()
            if success:
                response.success = True
                response.message = f'Rosbag recording stopped: {self.rosbag_path}'
            else:
                response.success = False
                response.message = 'Failed to stop rosbag recording'
        return response
    
    def start_rosbag_recording(self):
        """Start rosbag recording"""
        try:
            # Generate bag filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f'rosbag_{timestamp}'
            self.rosbag_path = os.path.join(self.rosbag_dir, bag_name)
            
            # Build ros2 bag record command
            cmd = ['ros2', 'bag', 'record', '-o', self.rosbag_path]
            
            # Add topics
            for topic in self.rosbag_topics:
                cmd.append(topic)
            
            # Storage backend: Use MCAP for better performance
            cmd.extend(['--storage', 'mcap'])
            
            # Enable compression - RAM disk eliminates disk I/O bottleneck
            # Compression reduces RAM usage and final file size
            cmd.extend(['--compression-mode', 'file'])
            cmd.extend(['--compression-format', 'zstd'])
            
            # Use default storage profile (removed 'resilient' for better write performance)
            # Recording to RAM disk eliminates disk I/O bottleneck
            
            self.get_logger().info(f'Starting rosbag recording to RAM: {self.rosbag_path}')
            self.get_logger().info('Using MCAP storage with zstd compression')
            self.get_logger().info('Recording to RAM disk - will move to permanent storage on stop')
            
            # Start the process with lower priority (nice value)
            self.rosbag_process = subprocess.Popen(
                ['nice', '-n', '10'] + cmd,  # Run with lower CPU priority
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            
            self.rosbag_recording = True
            self.get_logger().info(f'✓ Rosbag recording started (low priority, no compression)')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag recording: {str(e)}')
            return False
    
    def stop_rosbag_recording(self):
        """Stop rosbag recording and copy from RAM to permanent storage"""
        try:
            if self.rosbag_process is not None:
                # Send SIGINT to the process group
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
                
                # Wait for process to finish
                self.rosbag_process.wait(timeout=10)
                
                self.get_logger().info(f'✓ Rosbag recording stopped: {self.rosbag_path}')
                
                # Copy from RAM disk to permanent storage
                bag_name = os.path.basename(self.rosbag_path)
                final_path = os.path.join(self.rosbag_dir_final, bag_name)
                
                self.get_logger().info(f'Copying rosbag from RAM to permanent storage...')
                self.get_logger().info(f'Source: {self.rosbag_path}')
                self.get_logger().info(f'Destination: {final_path}')
                
                # Use shutil.move for efficient transfer
                shutil.move(self.rosbag_path, final_path)
                
                self.get_logger().info(f'✓ Rosbag moved to: {final_path}')
                
                self.rosbag_process = None
                self.rosbag_recording = False
                
                return True
            else:
                self.get_logger().warn('No rosbag recording process to stop')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Rosbag process did not stop gracefully, forcing termination')
            try:
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGKILL)
                self.rosbag_process = None
                self.rosbag_recording = False
            except:
                pass
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to stop rosbag recording: {str(e)}')
            return False
    
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
        
        # Tag next GPR status row with GPR_MOTOR_START to log at ~100 Hz aligned to GPR timing
        self.current_event = 'GPR_MOTOR_START'
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
        
        # Step 2: Tag MOTOR_STOPPING to be logged on the very next GPR status tick (~100 Hz)
        # This aligns the event timestamp with the GPR stream and avoids delays from
        # searching the nearest prior sample.
        self.current_event = 'MOTOR_STOPPING'
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
        # Don't send velocity commands when paused
        if self.paused:
            return
        
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
        """On each odometry message, cache pose data (already tilt-corrected by diff_drive_controller)."""
        # Use measurement time from header (ROS time domain)
        self.fastlio_timestamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000

        # Use odometry directly (already tilt-corrected by diff_drive_controller)
        self.fastlio_pose['pos_x']     = float(msg.pose.pose.position.x)
        self.fastlio_pose['pos_y']     = float(msg.pose.pose.position.y)
        self.fastlio_pose['pos_z']     = float(msg.pose.pose.position.z)
        self.fastlio_pose['quat_x']    = float(msg.pose.pose.orientation.x)
        self.fastlio_pose['quat_y']    = float(msg.pose.pose.orientation.y)
        self.fastlio_pose['quat_z']    = float(msg.pose.pose.orientation.z)
        self.fastlio_pose['quat_w']    = float(msg.pose.pose.orientation.w)
        self.fastlio_pose['vel_lin_x'] = float(msg.twist.twist.linear.x)
        self.fastlio_pose['vel_lin_y'] = float(msg.twist.twist.linear.y)
        self.fastlio_pose['vel_lin_z'] = float(msg.twist.twist.linear.z)
        self.fastlio_pose['vel_ang_x'] = float(msg.twist.twist.angular.x)
        self.fastlio_pose['vel_ang_y'] = float(msg.twist.twist.angular.y)
        self.fastlio_pose['vel_ang_z'] = float(msg.twist.twist.angular.z)

        # Filtering disabled: use raw pose directly (no moving average).
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
        
        # Stop rosbag recording if active
        if self.rosbag_recording:
            try:
                self.stop_rosbag_recording()
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
        if node.rosbag_recording:
            # Stop rosbag recording if active
            node.stop_rosbag_recording()
            node.get_logger().info('✓ Rosbag recording stopped due to interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

