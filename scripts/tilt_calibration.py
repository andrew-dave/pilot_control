#!/usr/bin/env python3
"""
Tilt Calibration Script - Estimates LiDAR mount tilt using IMU gravity alignment.

This script is FULLY STANDALONE - it automatically starts the LiDAR driver,
collects IMU data, computes calibration, and shuts down.

Run this ONCE with the robot stationary on level ground.
It collects IMU accelerometer samples to determine the gravity vector,
then computes the rotation matrix needed to align the LiDAR frame with
the robot's body frame (ground-aligned).

Algorithm:
- Starts Livox MID360 driver (if not already running)
- Collects IMU samples and averages them (gravity in body frame)
- Computes R_align using pitch-only correction from body-frame gravity
- Applies R_flip coordinate flip
- Saves R_map = R_flip @ R_align
- Shuts down the driver

Usage:
    ros2 run pilot_control tilt_calibration
    
    # Or with custom parameters:
    ros2 run pilot_control tilt_calibration --ros-args -p num_samples:=200 -p start_lidar:=false

The calibration result is saved to:
    /R_DATA/tilt_calibration/tilt_correction_matrices_<index>.npz

Where <index> is auto-incremented (0, 1, 2, ...).
The launch file automatically loads the latest (highest index) file.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy as np
import math
import os
import sys
import re
import glob
import signal
import subprocess
import time
import threading
from datetime import datetime


# Default calibration directory
TILT_CALIBRATION_DIR = '/R_DATA/tilt_calibration'


def get_latest_calibration_file(calibration_dir=TILT_CALIBRATION_DIR):
    """
    Find the latest (highest index) tilt_correction_matrices_*.npz file.
    
    Args:
        calibration_dir: Directory containing calibration files
        
    Returns:
        Path to the latest calibration file, or empty string if none found
    """
    if not os.path.exists(calibration_dir):
        return ''
    
    pattern = os.path.join(calibration_dir, 'tilt_correction_matrices_*.npz')
    files = glob.glob(pattern)
    
    if not files:
        return ''
    
    # Extract indices and find the maximum
    max_index = -1
    latest_file = ''
    
    for f in files:
        basename = os.path.basename(f)
        match = re.match(r'tilt_correction_matrices_(\d+)\.npz', basename)
        if match:
            index = int(match.group(1))
            if index > max_index:
                max_index = index
                latest_file = f
    
    return latest_file


def get_next_calibration_index(calibration_dir=TILT_CALIBRATION_DIR):
    """
    Get the next available index for a new calibration file.
    
    Args:
        calibration_dir: Directory containing calibration files
        
    Returns:
        Next available index (0 if no files exist)
    """
    if not os.path.exists(calibration_dir):
        return 0
    
    pattern = os.path.join(calibration_dir, 'tilt_correction_matrices_*.npz')
    files = glob.glob(pattern)
    
    if not files:
        return 0
    
    # Extract indices and find the maximum
    max_index = -1
    
    for f in files:
        basename = os.path.basename(f)
        match = re.match(r'tilt_correction_matrices_(\d+)\.npz', basename)
        if match:
            index = int(match.group(1))
            if index > max_index:
                max_index = index
    
    return max_index + 1


class LivoxDriverManager:
    """Manages the Livox LiDAR driver subprocess."""
    
    def __init__(self, logger):
        self.logger = logger
        self.process = None
        self.started_by_us = False
        
    def is_imu_available(self):
        """Check if IMU topic is already being published."""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return '/livox/imu' in result.stdout
        except Exception:
            return False
    
    def start(self):
        """Start the Livox driver if not already running."""
        if self.is_imu_available():
            self.logger.info('LiDAR driver already running (IMU topic available)')
            self.started_by_us = False
            return True
        
        self.logger.info('Starting Livox MID360 driver...')
        
        try:
            # Try to find the config file
            config_paths = [
                '/home/raj/livox_ws/src/livox_ros_driver2/config/MID360_config.json',
                '/opt/ros/humble/share/livox_ros_driver2/config/MID360_config.json',
            ]
            
            # Also try to find via ros2 pkg
            try:
                result = subprocess.run(
                    ['ros2', 'pkg', 'prefix', 'livox_ros_driver2'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0:
                    pkg_path = result.stdout.strip()
                    config_paths.insert(0, f'{pkg_path}/share/livox_ros_driver2/config/MID360_config.json')
            except Exception:
                pass
            
            config_file = None
            for path in config_paths:
                if os.path.exists(path):
                    config_file = path
                    break
            
            if not config_file:
                self.logger.error('Could not find MID360_config.json')
                self.logger.error(f'Searched: {config_paths}')
                return False
            
            self.logger.info(f'Using config: {config_file}')
            
            # Start the driver node directly
            cmd = [
                'ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node',
                '--ros-args',
                '-p', 'xfer_format:=1',
                '-p', 'multi_topic:=0',
                '-p', 'data_src:=0',
                '-p', 'publish_freq:=10.0',
                '-p', 'output_data_type:=0',
                '-p', 'frame_id:=livox_frame',
                '-p', f'user_config_path:={config_file}',
            ]
            
            # Start process with output suppressed (to keep calibration output clean)
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            
            self.started_by_us = True
            
            # Wait for IMU topic to become available
            self.logger.info('Waiting for LiDAR to initialize...')
            for i in range(30):  # Wait up to 30 seconds
                time.sleep(1)
                if self.is_imu_available():
                    self.logger.info('LiDAR driver started successfully')
                    return True
                if i % 5 == 4:
                    self.logger.info(f'Still waiting for IMU... ({i+1}s)')
            
            self.logger.error('Timeout waiting for LiDAR driver to start')
            self.stop()
            return False
            
        except FileNotFoundError:
            self.logger.error('livox_ros_driver2 package not found')
            self.logger.error('Make sure the Livox ROS2 driver is installed')
            return False
        except Exception as e:
            self.logger.error(f'Failed to start LiDAR driver: {e}')
            return False
    
    def stop(self):
        """Stop the Livox driver if we started it."""
        if self.process and self.started_by_us:
            self.logger.info('Stopping LiDAR driver...')
            try:
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except Exception:
                try:
                    # Force kill if SIGTERM didn't work
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.process = None
            self.logger.info('LiDAR driver stopped')


class TiltCalibration(Node):
    def __init__(self, lidar_manager=None):
        super().__init__('tilt_calibration')
        
        self.lidar_manager = lidar_manager

        # Parameters
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('num_samples', 100)  # Number of IMU samples to average
        self.declare_parameter('timeout_sec', 30.0)  # Timeout in seconds
        self.declare_parameter('calibration_dir', TILT_CALIBRATION_DIR)  # Calibration directory
        self.declare_parameter('start_lidar', True)  # Whether to start LiDAR driver

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.calibration_dir = str(self.get_parameter('calibration_dir').value)
        self.start_lidar = bool(self.get_parameter('start_lidar').value)

        # State - IMU averaging
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
        self.accel_avg = None
        self.accel_initialized = False
        
        # State - Calibration
        self.calibration_complete = False
        self.calibration_success = False

        # Determine output path with auto-incrementing index
        # Create calibration directory if it doesn't exist
        os.makedirs(self.calibration_dir, exist_ok=True)
        
        # Get next available index
        self.calibration_index = get_next_calibration_index(self.calibration_dir)
        self.output_file = os.path.join(
            self.calibration_dir, 
            f'tilt_correction_matrices_{self.calibration_index}.npz'
        )

        # Subscription - only IMU needed (no odometry required!)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )

        # Timeout timer
        self.timeout_timer = self.create_timer(self.timeout_sec, self.timeout_callback)

        self.get_logger().info('='*60)
        self.get_logger().info('TILT CALIBRATION (Standalone)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Samples to collect: {self.num_samples}')
        self.get_logger().info(f'Timeout: {self.timeout_sec}s')
        self.get_logger().info(f'Calibration directory: {self.calibration_dir}')
        self.get_logger().info(f'Output file: {self.output_file} (index={self.calibration_index})')
        
        # Show existing calibrations
        latest = get_latest_calibration_file(self.calibration_dir)
        if latest:
            self.get_logger().info(f'Previous latest: {os.path.basename(latest)}')
        else:
            self.get_logger().info('No previous calibrations found')
        
        self.get_logger().info('')
        self.get_logger().info('IMPORTANT: Keep the robot STATIONARY on LEVEL GROUND!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Collecting IMU samples... (0/{self.num_samples})')

    @staticmethod
    def compute_tilt_correction_matrix_body(a_body):
        """
        Compute PURE PITCH correction around Y-axis from BODY-FRAME gravity.
        
        This works directly with IMU measurements without needing odometry.
        The IMU measures gravity in the sensor's body frame. We compute the
        pitch angle needed to make gravity point straight down [0, 0, -1].
        
        Assumes LiDAR is mounted with tilt only in the forward direction (pitch),
        not sideways (roll).
        
        Input: a_body = gravity direction in body frame (normalized)
        Output: (R_align, pitch_angle) rotation matrix for pitch correction
        """
        a_body = np.array(a_body, dtype=float)
        a_body = a_body / (np.linalg.norm(a_body) + 1e-12)
        
        # Target: gravity pointing down [0, 0, -1] in corrected frame
        # In body frame, gravity points roughly in +Z direction (sensor facing forward-down)
        
        # Already aligned? (gravity pointing down)
        if a_body[2] < -0.9999:
            return np.eye(3, dtype=float), 0.0
        
        # Extract ONLY pitch (rotation around Y-axis)
        # Project gravity onto XZ plane (forward-backward tilt)
        # Ignore the Y component (assume no roll, just sensor noise)
        # 
        # The pitch angle is the angle from -Z axis to gravity in XZ plane
        pitch = np.arctan2(a_body[0], -a_body[2])
        
        # Build pure pitch rotation matrix around Y-axis
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        
        R_pitch = np.array([[cp,  0.0, sp],
                            [0.0, 1.0, 0.0],
                            [-sp, 0.0, cp]], dtype=float)
        
        return R_pitch, pitch

    # ---------------- IMU callback ----------------
    def imu_callback(self, msg: Imu):
        if self.calibration_complete or self.accel_initialized:
            return

        # Extract acceleration
        a = np.array([
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        ], dtype=float)

        # Validate - skip invalid samples
        if not np.all(np.isfinite(a)):
            return

        # Accumulate samples
        self.accel_sum += a
        self.accel_count += 1

        # Progress update every 10 samples
        if self.accel_count % 10 == 0:
            self.get_logger().info(
                f'Collecting IMU samples... ({self.accel_count}/{self.num_samples})')

        # Check if we have enough samples
        if self.accel_count >= self.num_samples:
            self.accel_avg = self.accel_sum / float(self.accel_count)
            self.accel_initialized = True
            self.get_logger().info(
                f'IMU accel averaged (N={self.accel_count}): '
                f'[{self.accel_avg[0]:.3f}, {self.accel_avg[1]:.3f}, {self.accel_avg[2]:.3f}]'
            )
            # Compute calibration immediately - no need to wait for odometry!
            self.compute_calibration()

    # ---------------- Timeout callback ----------------
    def timeout_callback(self):
        if not self.calibration_complete:
            self.get_logger().error(
                f'Timeout! IMU samples: {self.accel_count}/{self.num_samples}')
            self.get_logger().error(f'Check that IMU is publishing on {self.imu_topic}')
            self.calibration_complete = True
            self.calibration_success = False
            rclpy.shutdown()

    # ---------------- Compute calibration ----------------
    def compute_calibration(self):
        """Compute tilt correction matrices from collected IMU samples."""
        if self.calibration_complete:
            return
            
        self.calibration_complete = True

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('COMPUTING CALIBRATION')
        self.get_logger().info('='*60)

        # Normalize body-frame gravity
        a_body_norm = self.accel_avg / (np.linalg.norm(self.accel_avg) + 1e-12)
        self.get_logger().info(
            f'Gravity in body frame (normalized): '
            f'[{a_body_norm[0]:.4f}, {a_body_norm[1]:.4f}, {a_body_norm[2]:.4f}]')

        # Fixed coordinate system flip (same as original odom_tilt_corrector)
        R_flip = np.array([[-1.0, 0.0, 0.0],
                           [ 0.0, 1.0, 0.0],
                           [ 0.0, 0.0,-1.0]], dtype=float)

        # Compute pitch-only correction from body-frame gravity directly
        # No need for odometry - we work in body frame
        R_align, pitch_angle = self.compute_tilt_correction_matrix_body(a_body_norm)
        pitch_deg = math.degrees(pitch_angle)

        # Final transformation: R_map = R_flip @ R_align
        R_map = R_flip @ R_align

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Pitch angle: {pitch_deg:.2f}°')

        # Verify: gravity should now point to [0, 0, -1] after correction
        # Note: We apply R_map to body-frame gravity for verification
        a_corrected = R_map @ a_body_norm
        self.get_logger().info(
            f'Gravity after correction: '
            f'[{a_corrected[0]:.4f}, {a_corrected[1]:.4f}, {a_corrected[2]:.4f}]')
        self.get_logger().info(f'  (should be close to [0, 0, -1])')

        # Save calibration
        self._save_calibration(R_map, R_align, R_flip, a_body_norm, pitch_angle)

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION COMPLETE')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Saved to: {self.output_file}')
        self.get_logger().info('')
        self.get_logger().info('You can now run robot_complete.launch.py')
        self.get_logger().info('The tilt correction will be loaded automatically.')
        self.get_logger().info('='*60)

        self.calibration_success = True
        raise SystemExit  # Cleanly breaks out of the rclpy.spin() loop

    # ---------------- Save calibration ----------------
    def _save_calibration(self, R_map, R_align, R_flip, a_body, pitch_angle):
        """Save calibration results to .npz file (same format as original odom_tilt_corrector)."""
        try:
            # Ensure output directory exists
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)

            timestamp = datetime.now()
            timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S")

            # Save in format compatible with raw_map_saver.cpp and odom_tilt_corrector.py
            # Same keys as original save_transformation_details()
            np.savez(
                self.output_file,
                R_map=R_map,
                R_align=R_align,
                R_flip=R_flip,
                a_body=a_body,
                pitch_angle=pitch_angle,
                p0_world=np.zeros(3),  # Origin set at runtime
                timestamp=timestamp_str
            )

            # Also save human-readable CSV
            csv_file = self.output_file.replace('.npz', '.csv')
            with open(csv_file, 'w') as f:
                f.write('# Tilt Calibration Results\n')
                f.write(f'# Generated: {timestamp.strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write(f'# Samples collected: {self.accel_count}\n')
                f.write('#\n')
                f.write(f'# Pitch Angle: {math.degrees(pitch_angle):.6f} degrees ({pitch_angle:.6f} radians)\n')
                f.write('#\n')
                f.write('# Acceleration Body Frame (raw IMU average, normalized)\n')
                f.write(f'accel_body_x,{a_body[0]:.6f}\n')
                f.write(f'accel_body_y,{a_body[1]:.6f}\n')
                f.write(f'accel_body_z,{a_body[2]:.6f}\n')
                f.write('#\n')
                f.write('# R_align (Tilt Alignment Matrix - pitch only)\n')
                for i in range(3):
                    f.write(f'{R_align[i,0]:.8f},{R_align[i,1]:.8f},{R_align[i,2]:.8f}\n')
                f.write('#\n')
                f.write('# R_flip (Coordinate Flip Matrix)\n')
                for i in range(3):
                    f.write(f'{R_flip[i,0]:.8f},{R_flip[i,1]:.8f},{R_flip[i,2]:.8f}\n')
                f.write('#\n')
                f.write('# R_map (Final Transformation: R_flip @ R_align)\n')
                for i in range(3):
                    f.write(f'{R_map[i,0]:.8f},{R_map[i,1]:.8f},{R_map[i,2]:.8f}\n')
                f.write('#\n')
                f.write('# Gravity After Correction (should be [0, 0, -1])\n')
                a_corrected = R_map @ a_body
                f.write(f'gravity_corrected_x,{a_corrected[0]:.6f}\n')
                f.write(f'gravity_corrected_y,{a_corrected[1]:.6f}\n')
                f.write(f'gravity_corrected_z,{a_corrected[2]:.6f}\n')

            self.get_logger().info(f'Also saved CSV: {csv_file}')

        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            self.calibration_success = False


def main(args=None):
    # Initialize ROS first to get parameters
    rclpy.init(args=args)
    
    # Create a temporary node just to read the start_lidar parameter
    temp_node = rclpy.create_node('_tilt_calibration_init')
    temp_node.declare_parameter('start_lidar', True)
    start_lidar = bool(temp_node.get_parameter('start_lidar').value)
    temp_node.destroy_node()
    
    lidar_manager = None
    
    print('')
    print('='*60)
    print('TILT CALIBRATION - Standalone Mode')
    print('='*60)
    
    if start_lidar:
        # Create a simple logger for the LiDAR manager
        class SimpleLogger:
            def info(self, msg): print(f'[INFO] {msg}')
            def error(self, msg): print(f'[ERROR] {msg}')
            def warn(self, msg): print(f'[WARN] {msg}')
        
        lidar_manager = LivoxDriverManager(SimpleLogger())
        
        if not lidar_manager.start():
            print('')
            print('='*60)
            print('FAILED TO START LIDAR DRIVER')
            print('='*60)
            print('Options:')
            print('  1. Start the LiDAR driver manually first:')
            print('     ros2 launch livox_ros_driver2 msg_MID360_launch.py')
            print('')
            print('  2. Run calibration with start_lidar:=false:')
            print('     ros2 run pilot_control tilt_calibration --ros-args -p start_lidar:=false')
            print('='*60)
            rclpy.shutdown()
            sys.exit(1)
    else:
        print('[INFO] start_lidar=false, assuming LiDAR driver is already running')
    
    print('')
    
    # Create and run the calibration node
    node = TiltCalibration(lidar_manager)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Calibration interrupted by user')
    except SystemExit:
        pass  # This is our clean exit signal from compute_calibration
    finally:
        # 1. Save the success state
        calibration_success = node.calibration_success

        # 2. Safely shut down ROS 2 context
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

        # 3. Stop LiDAR driver subprocess to release the terminal
        if lidar_manager:
            lidar_manager.stop()

        # 4. Exit with the correct OS code for the Qt QProcess
        if not calibration_success:
            print('\n[ERROR] Calibration failed or was interrupted')
            sys.exit(1)
        else:
            sys.exit(0)


if __name__ == '__main__':
    main()
