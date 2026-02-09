#!/usr/bin/env python3
"""
Tilt Calibration Script - Estimates LiDAR mount tilt using IMU gravity alignment.

This script should be run ONCE with the robot stationary on level ground.
It collects IMU accelerometer samples to determine the gravity vector,
then computes the rotation matrix needed to align the LiDAR frame with
the robot's body frame (ground-aligned).

This implements the same algorithm as the original odom_tilt_corrector.py:
- Collects IMU samples and averages them
- Computes R_align using pitch-only correction from gravity
- Applies R_flip coordinate flip
- Saves R_map = R_flip @ R_align

Usage:
    ros2 run pilot_control tilt_calibration [--ros-args -p num_samples:=100]

The calibration result is saved to:
    <pilot_control>/config/tilt_correction_matrices.npz

All nodes that need tilt correction should load this file.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math
import os
import sys
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class TiltCalibration(Node):
    def __init__(self):
        super().__init__('tilt_calibration')

        # Parameters
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('odometry_topic', '/Odometry')
        self.declare_parameter('num_samples', 100)  # Number of IMU samples to average
        self.declare_parameter('timeout_sec', 30.0)  # Timeout in seconds
        self.declare_parameter('output_file', '')  # Override output path (optional)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_topic = str(self.get_parameter('odometry_topic').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.output_file = str(self.get_parameter('output_file').value)

        # State - IMU averaging
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
        self.accel_avg = None
        self.accel_initialized = False
        
        # State - Odometry (for transforming gravity to world frame)
        self.last_odom_q = None
        self.odom_received = False
        
        # State - Calibration
        self.calibration_complete = False
        self.calibration_success = False

        # Determine output path
        if not self.output_file:
            try:
                # Try source directory first (more reliable for development)
                source_config = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config'
                )
                if os.path.exists(source_config):
                    config_dir = source_config
                else:
                    # Fallback to share directory
                    pkg_share = get_package_share_directory('pilot_control')
                    config_dir = os.path.join(pkg_share, 'config')
                    
                self.output_file = os.path.join(config_dir, 'tilt_correction_matrices.npz')
            except Exception as e:
                self.get_logger().error(f'Could not determine config directory: {e}')
                self.output_file = '/tmp/tilt_correction_matrices.npz'

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )

        # Timeout timer
        self.timeout_timer = self.create_timer(self.timeout_sec, self.timeout_callback)

        self.get_logger().info('='*60)
        self.get_logger().info('TILT CALIBRATION')
        self.get_logger().info('='*60)
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Odometry topic: {self.odom_topic}')
        self.get_logger().info(f'Samples to collect: {self.num_samples}')
        self.get_logger().info(f'Timeout: {self.timeout_sec}s')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info('')
        self.get_logger().info('IMPORTANT: Keep the robot STATIONARY on LEVEL GROUND!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Collecting IMU samples... (0/{self.num_samples})')

    # ---------------- Quaternion/Matrix utilities ----------------
    @staticmethod
    def quat_to_matrix(q):
        """Convert quaternion (x, y, z, w) to rotation matrix."""
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
    def compute_tilt_correction_matrix(a_world):
        """
        Compute PURE PITCH correction around Y-axis ONLY.
        Assumes LiDAR is mounted with tilt only in the forward direction (pitch),
        not sideways (roll).
        
        This gives a rotation matrix that:
        - Corrects the forward/backward tilt (pitch around Y)
        - Preserves yaw completely (no rotation around Z)
        - Ignores any roll component (assumes it's sensor noise)
        
        Input: a_world = gravity direction in world frame (normalized)
        Output: rotation matrix for pitch correction only
        """
        a_world = np.array(a_world, dtype=float)
        a_world = a_world / (np.linalg.norm(a_world) + 1e-12)
        
        # Target: gravity pointing down [0, 0, -1]
        # Already aligned?
        if a_world[2] < -0.9999:
            return np.eye(3, dtype=float), 0.0
        
        # Extract ONLY pitch (rotation around Y-axis)
        # Project gravity onto XZ plane (forward-backward tilt)
        # Ignore the Y component (assume no roll, just sensor noise)
        pitch = np.arctan2(a_world[0], -a_world[2])
        
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
                f'{self.accel_avg[0]:.3f},{self.accel_avg[1]:.3f},{self.accel_avg[2]:.3f} — waiting for odom'
            )
            
            # If we already have odometry, compute calibration
            if self.odom_received:
                self.compute_calibration()

    # ---------------- Odometry callback ----------------
    def odom_callback(self, msg: Odometry):
        if self.calibration_complete:
            return

        # Store the latest orientation
        self.last_odom_q = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry received')
        
        # If IMU is ready, compute calibration
        if self.accel_initialized:
            self.compute_calibration()

    # ---------------- Timeout callback ----------------
    def timeout_callback(self):
        if not self.calibration_complete:
            self.get_logger().error(
                f'Timeout! IMU samples: {self.accel_count}/{self.num_samples}, '
                f'Odom received: {self.odom_received}')
            if self.accel_count < self.num_samples:
                self.get_logger().error(f'Check that IMU is publishing on {self.imu_topic}')
            if not self.odom_received:
                self.get_logger().error(f'Check that odometry is publishing on {self.odom_topic}')
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

        # Transform gravity to world frame using odometry orientation
        R_wb = self.quat_to_matrix(self.last_odom_q)
        a_world = R_wb @ a_body_norm
        self.get_logger().info(
            f'Gravity in world frame: '
            f'[{a_world[0]:.4f}, {a_world[1]:.4f}, {a_world[2]:.4f}]')

        # Fixed coordinate system flip (same as original odom_tilt_corrector)
        R_flip = np.array([[-1.0, 0.0, 0.0],
                           [ 0.0, 1.0, 0.0],
                           [ 0.0, 0.0,-1.0]], dtype=float)

        # Compute pitch-only correction from world-frame gravity
        R_align, pitch_angle = self.compute_tilt_correction_matrix(a_world)
        pitch_deg = math.degrees(pitch_angle)

        # Final transformation: R_map = R_flip @ R_align
        R_map = R_flip @ R_align

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Pitch angle: {pitch_deg:.2f}°')

        # Verify: gravity should now point to [0, 0, -1] after correction
        a_corrected = R_map @ a_world
        self.get_logger().info(
            f'Gravity after correction: '
            f'[{a_corrected[0]:.4f}, {a_corrected[1]:.4f}, {a_corrected[2]:.4f}]')
        self.get_logger().info(f'  (should be close to [0, 0, -1])')

        # Save calibration
        self._save_calibration(R_map, R_align, R_flip, a_world, a_body_norm, pitch_angle)

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION COMPLETE')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Saved to: {self.output_file}')
        self.get_logger().info('')
        self.get_logger().info('You can now restart robot_complete.launch.py')
        self.get_logger().info('The tilt correction will be loaded automatically.')
        self.get_logger().info('='*60)

        self.calibration_success = True
        rclpy.shutdown()

    # ---------------- Save calibration ----------------
    def _save_calibration(self, R_map, R_align, R_flip, a_world, a_body, pitch_angle):
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
                a_world=a_world,
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
                f.write('# Gravity Direction World Frame (before correction)\n')
                f.write(f'gravity_world_x,{a_world[0]:.6f}\n')
                f.write(f'gravity_world_y,{a_world[1]:.6f}\n')
                f.write(f'gravity_world_z,{a_world[2]:.6f}\n')
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
                a_corrected = R_map @ a_world
                f.write(f'gravity_corrected_x,{a_corrected[0]:.6f}\n')
                f.write(f'gravity_corrected_y,{a_corrected[1]:.6f}\n')
                f.write(f'gravity_corrected_z,{a_corrected[2]:.6f}\n')

            self.get_logger().info(f'Also saved CSV: {csv_file}')

        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            self.calibration_success = False


def main(args=None):
    rclpy.init(args=args)
    node = TiltCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Calibration interrupted by user')
    finally:
        if not node.calibration_success:
            node.get_logger().error('Calibration failed or was interrupted')
            sys.exit(1)
        node.destroy_node()


if __name__ == '__main__':
    main()
