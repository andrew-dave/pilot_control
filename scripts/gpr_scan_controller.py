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
from std_srvs.srv import Trigger
import os
import time
from datetime import datetime
import threading
import csv
from collections import deque


class GPRScanController(Node):
    def __init__(self):
        super().__init__('gpr_scan_controller')
        
        # Declare parameters
        self.declare_parameter('gpr_wheel_radius', 0.03)      # 60mm diameter virtual wheel
        self.declare_parameter('gpr_gear_ratio', 1.0)         # Direct drive
        self.declare_parameter('velocity_multiplier', 1.0)
        self.declare_parameter('gpr_scan_velocity_mps', 0.5)  # 0.5 m/s scanning speed
        self.declare_parameter('invert_third', False)
        self.declare_parameter('fastlio_odom_topic', '/Odometry')
        self.declare_parameter('log_frequency_hz', 50.0)      # 50 Hz logging
        self.declare_parameter('log_directory', os.path.expanduser('~/gpr_scans'))
        
        # Get parameters
        self.gpr_wheel_radius = self.get_parameter('gpr_wheel_radius').value
        self.gpr_gear_ratio = self.get_parameter('gpr_gear_ratio').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        self.gpr_scan_velocity = self.get_parameter('gpr_scan_velocity_mps').value
        self.invert_third = self.get_parameter('invert_third').value
        self.fastlio_topic = self.get_parameter('fastlio_odom_topic').value
        self.log_freq = self.get_parameter('log_frequency_hz').value
        self.log_dir = self.get_parameter('log_directory').value
        
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
        self.post_stop_samples = 0  # Counter for post-stop samples
        self.post_stop_target = 5  # Number of samples to log after stop
        
        # GPR motor state (from ODrive feedback)
        self.gpr_position = 0.0
        self.gpr_velocity = 0.0
        self.gpr_timestamp_us = 0  # Timestamp in microseconds
        self.gpr_data_available = False
        
        # Pre-scan buffer (circular buffer for last N samples before scan start)
        self.pre_scan_buffer_size = 5
        self.pre_scan_buffer = deque(maxlen=self.pre_scan_buffer_size)
        
        # Publishers
        self.gpr_motor_pub = self.create_publisher(
            ControlMessage, '/gpr/control_message', 10)
        
        # Subscribers
        self.fastlio_sub = self.create_subscription(
            Odometry, self.fastlio_topic, self.fastlio_callback, 10)
        
        # Subscribe to GPR motor status for position/velocity feedback
        self.gpr_status_sub = self.create_subscription(
            ControllerStatus, '/gpr/controller_status', self.gpr_status_callback, 10)
        
        # Service clients (Arduino control)
        self.line_start_client = self.create_client(Trigger, '/gpr_line_start')
        self.line_stop_client = self.create_client(Trigger, '/gpr_line_stop')
        
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
        self.get_logger().info(f'Pre-scan buffer: {self.pre_scan_buffer_size} samples')
        self.get_logger().info(f'Post-stop samples: {self.post_stop_target}')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'Service available: /gpr_scan/toggle')
        self.get_logger().info('')
        self.get_logger().info('💡 Press assigned key in teleop to start/stop scanning')
    
    def gpr_status_callback(self, msg):
        """Store latest GPR motor status"""
        self.gpr_position = msg.pos_estimate  # turns
        self.gpr_velocity = msg.vel_estimate  # turns/s
        # Create timestamp from current time in microseconds
        current_time = self.get_clock().now()
        seconds, nanoseconds = current_time.seconds_nanoseconds()
        self.gpr_timestamp_us = seconds * 1_000_000 + nanoseconds // 1000  # Convert to microseconds
        self.gpr_data_available = True
    
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
        """Start GPR scanning sequence"""
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
            self.log_file_handle.write(f'# GPR Scan Data - Comprehensive Logging\n')
            self.log_file_handle.write(f'# Start Time: {timestamp}\n')
            self.log_file_handle.write(f'# GPR Velocity (m/s): {self.gpr_scan_velocity}\n')
            self.log_file_handle.write(f'# Log Frequency (Hz): {self.log_freq}\n')
            self.log_file_handle.write(f'# GPR Wheel Radius (m): {self.gpr_wheel_radius}\n')
            self.log_file_handle.write(f'# Fast-LIO Topic: {self.fastlio_topic}\n')
            self.log_file_handle.write(f'# Pre-Scan Buffer: {self.pre_scan_buffer_size} samples\n')
            self.log_file_handle.write(f'# Post-Stop Samples: {self.post_stop_target}\n')
            self.log_file_handle.write(f'#\n')
            self.log_file_handle.write(f'# Timestamp Format: Microseconds (µs) since epoch\n')
            self.log_file_handle.write(f'# Time Sync: Both fastlio_time_us and gpr_time_us use ROS time\n')
            self.log_file_handle.write(f'#\n')
            self.log_file_handle.write(f'# Event Types:\n')
            self.log_file_handle.write(f'#   PRE_SCAN - Samples before scan started (from buffer)\n')
            self.log_file_handle.write(f'#   KEY_PRESS_START - Scan key pressed\n')
            self.log_file_handle.write(f'#   PRE_MOTOR_START_1/2 - Just before motor starts\n')
            self.log_file_handle.write(f'#   MOTOR_STARTED - Motor velocity command sent\n')
            self.log_file_handle.write(f'#   SCANNING - Normal scanning operation\n')
            self.log_file_handle.write(f'#   KEY_PRESS_STOP - Stop key pressed\n')
            self.log_file_handle.write(f'#   MOTOR_STOPPING_1/2 - While stopping motor\n')
            self.log_file_handle.write(f'#   POST_STOP - After motor stopped\n')
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
                'gpr_position', 'gpr_velocity'
            ])
            
            self.get_logger().info(f'✓ Log file created: {self.current_log_file}')
            
            # Write buffered pre-scan samples
            if len(self.pre_scan_buffer) > 0:
                self.get_logger().info(f'✓ Writing {len(self.pre_scan_buffer)} pre-scan samples')
                for sample in self.pre_scan_buffer:
                    self.csv_writer.writerow(sample)
                    self.log_count += 1
                self.get_logger().info(f'✓ Pre-scan samples written')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {e}')
            return False
        
        # Log KEY_PRESS_START event
        self.log_event('KEY_PRESS_START')
        
        # Step 2: Start linear actuator (Arduino - line up)
        self.get_logger().info('⏳ Starting linear actuator...')
        if self.line_start_client.wait_for_service(timeout_sec=1.0):
            future = self.line_start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.get_logger().info('✓ Linear actuator started')
            else:
                self.get_logger().warn('⚠️  Linear actuator service call failed')
        else:
            self.get_logger().warn('⚠️  Linear actuator service not available')
        
        # Log PRE_MOTOR_START events (capture a couple of samples before motor starts)
        self.log_event('PRE_MOTOR_START_1')
        time.sleep(0.05)  # Wait for one sample
        self.log_event('PRE_MOTOR_START_2')
        time.sleep(0.05)  # Wait for one more sample
        
        # Step 3: Start GPR motor rotation
        self.scan_start_time = time.time()
        self.scanning = True
        
        # Send velocity command immediately
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
        self.get_logger().info(f'✓ GPR motor started at {self.gpr_scan_velocity:.3f} m/s ({motor_turns_per_sec:.3f} turns/s)')
        
        # Log MOTOR_STARTED event
        self.log_event('MOTOR_STARTED')
        
        # Step 4: Start data logging (happens in fastlio_callback)
        self.get_logger().info(f'✓ Fast-LIO logging started at {self.log_freq:.1f} Hz')
        
        self.get_logger().info('')
        self.get_logger().info('🟢 GPR SCAN ACTIVE')
        self.get_logger().info('='*70)
        
        return True
    
    def stop_scan(self):
        """Stop GPR scanning sequence"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('STOPPING GPR SCAN')
        self.get_logger().info('='*70)
        
        # Log KEY_PRESS_STOP event
        self.log_event('KEY_PRESS_STOP')
        
        # Log MOTOR_STOPPING events (capture samples while stopping)
        self.log_event('MOTOR_STOPPING_1')
        time.sleep(0.05)  # Wait for one sample
        self.log_event('MOTOR_STOPPING_2')
        
        # Step 1: Stop GPR motor
        self.scanning = False
        self.stopping = True  # Enable post-stop logging
        self.post_stop_samples = 0
        
        # Send zero velocity command immediately
        msg = ControlMessage()
        msg.control_mode = 2  # VELOCITY_CONTROL
        msg.input_mode = 1    # PASSTHROUGH
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        msg.input_pos = 0.0
        
        self.gpr_motor_pub.publish(msg)
        self.get_logger().info('✓ GPR motor stopped')
        
        # Wait for post-stop samples to be logged
        self.get_logger().info(f'⏳ Capturing post-stop samples ({self.post_stop_target})...')
        max_wait = 2.0  # Maximum 2 seconds to wait for post-stop samples
        wait_start = time.time()
        while self.stopping and (time.time() - wait_start) < max_wait:
            time.sleep(0.05)  # Check every 50ms
        
        if self.stopping:
            self.get_logger().warn(f'⚠️  Timeout waiting for post-stop samples')
            self.stopping = False
        
        # Step 2: Stop linear actuator
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
        
        # Step 3: Close log file
        if self.log_file_handle:
            try:
                self.log_file_handle.close()
                self.log_file_handle = None
                self.csv_writer = None
                
                scan_duration = time.time() - self.scan_start_time if self.scan_start_time else 0
                
                self.get_logger().info(f'✓ Log file closed: {self.current_log_file}')
                self.get_logger().info(f'📊 Scan statistics:')
                self.get_logger().info(f'   Duration: {scan_duration:.1f} seconds')
                self.get_logger().info(f'   Samples logged: {self.log_count}')
                if scan_duration > 0:
                    self.get_logger().info(f'   Average rate: {self.log_count/scan_duration:.1f} Hz')
            except Exception as e:
                self.get_logger().error(f'Error closing log file: {e}')
        
        self.get_logger().info('')
        self.get_logger().info('🔴 GPR SCAN STOPPED')
        self.get_logger().info('='*70)
    
    def update_gpr_motor(self):
        """Send velocity command to GPR motor"""
        if self.scanning:
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
        else:
            # Send zero velocity when not scanning
            msg = ControlMessage()
            msg.control_mode = 2  # VELOCITY_CONTROL
            msg.input_mode = 1    # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            msg.input_pos = 0.0
            
            self.gpr_motor_pub.publish(msg)
    
    def log_event(self, event_name):
        """Log a special event with current Fast-LIO and GPR data"""
        # This will be captured in the next fastlio_callback
        # We store the event name so it can be included
        self.current_event = event_name
    
    def fastlio_callback(self, msg):
        """Log Fast-LIO odometry data with GPR position - CSV format for minimal latency"""
        # Extract timestamp (ROS time) in microseconds
        fastlio_timestamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        
        # Build data row
        # Determine event type
        if hasattr(self, 'current_event'):
            event = self.current_event
        elif self.stopping:
            event = 'POST_STOP'
        elif self.scanning:
            event = 'SCANNING'
        else:
            event = 'PRE_SCAN'
        
        data_row = [
            self.log_count,
            event,
            fastlio_timestamp_us,  # Timestamp in microseconds (integer)
            self.gpr_timestamp_us if self.gpr_data_available else 0,  # Timestamp in microseconds (integer)
            f'{msg.pose.pose.position.x:.6f}',
            f'{msg.pose.pose.position.y:.6f}',
            f'{msg.pose.pose.position.z:.6f}',
            f'{msg.pose.pose.orientation.x:.6f}',
            f'{msg.pose.pose.orientation.y:.6f}',
            f'{msg.pose.pose.orientation.z:.6f}',
            f'{msg.pose.pose.orientation.w:.6f}',
            f'{msg.twist.twist.linear.x:.6f}',
            f'{msg.twist.twist.linear.y:.6f}',
            f'{msg.twist.twist.linear.z:.6f}',
            f'{msg.twist.twist.angular.x:.6f}',
            f'{msg.twist.twist.angular.y:.6f}',
            f'{msg.twist.twist.angular.z:.6f}',
            f'{self.gpr_position:.6f}' if self.gpr_data_available else '0.0',
            f'{self.gpr_velocity:.6f}' if self.gpr_data_available else '0.0'
        ]
        
        # Clear event after using it
        if hasattr(self, 'current_event'):
            delattr(self, 'current_event')
        
        # Store in pre-scan buffer if not scanning
        if not self.scanning and not self.stopping:
            self.pre_scan_buffer.append(data_row)
            return
        
        # Write to CSV if we have a writer (scanning or stopping)
        if self.csv_writer:
            try:
                self.csv_writer.writerow(data_row)
                self.log_count += 1
                
                # Handle post-stop logging
                if self.stopping:
                    self.post_stop_samples += 1
                    if self.post_stop_samples >= self.post_stop_target:
                        self.get_logger().info(f'✓ Post-stop samples captured ({self.post_stop_target})')
                        self.stopping = False
                
                # Log progress every 50 samples (only during active scanning)
                if self.scanning and self.log_count % 50 == 0:
                    elapsed = time.time() - self.scan_start_time
                    self.get_logger().info(
                        f'📝 Logged {self.log_count} samples | '
                        f'{elapsed:.1f}s | '
                        f'{self.log_count/elapsed:.1f} Hz'
                    )
            
            except Exception as e:
                self.get_logger().error(f'Error logging data: {e}')
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if self.scanning:
            self.stop_scan()


def main(args=None):
    rclpy.init(args=args)
    node = GPRScanController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted by user')
        if node.scanning:
            node.stop_scan()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

