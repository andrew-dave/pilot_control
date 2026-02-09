#!/usr/bin/env python3
"""
Tilt Calibration Script - Estimates LiDAR mount tilt using IMU gravity alignment.

This script should be run ONCE with the robot stationary on level ground.
It collects IMU accelerometer samples to determine the gravity vector,
then computes the rotation matrix needed to align the LiDAR frame with
the robot's body frame (ground-aligned).

Usage:
    ros2 run pilot_control tilt_calibration [--ros-args -p num_samples:=100]

The calibration result is saved to:
    <pilot_control>/config/tilt_correction_matrices.npz

All nodes that need tilt correction should load this file.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter('num_samples', 100)  # Number of IMU samples to average
        self.declare_parameter('timeout_sec', 30.0)  # Timeout in seconds
        self.declare_parameter('output_file', '')  # Override output path (optional)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.output_file = str(self.get_parameter('output_file').value)

        # State
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
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

        # Subscription
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )

        # Timeout timer
        self.timeout_timer = self.create_timer(self.timeout_sec, self.timeout_callback)

        self.get_logger().info('='*60)
        self.get_logger().info('TILT CALIBRATION')
        self.get_logger().info('='*60)
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Samples to collect: {self.num_samples}')
        self.get_logger().info(f'Timeout: {self.timeout_sec}s')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info('')
        self.get_logger().info('IMPORTANT: Keep the robot STATIONARY on LEVEL GROUND!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Collecting IMU samples... (0/{self.num_samples})')

    def imu_callback(self, msg: Imu):
        if self.calibration_complete:
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
            self.compute_calibration()

    def timeout_callback(self):
        if not self.calibration_complete:
            self.get_logger().error(
                f'Timeout! Only collected {self.accel_count}/{self.num_samples} samples')
            self.get_logger().error('Check that IMU is publishing on the correct topic')
            self.calibration_complete = True
            self.calibration_success = False
            rclpy.shutdown()

    def compute_calibration(self):
        """Compute tilt correction matrices from collected IMU samples."""
        self.calibration_complete = True

        # Average the acceleration samples
        accel_avg = self.accel_sum / float(self.accel_count)

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(
            f'Averaged acceleration (IMU frame): '
            f'[{accel_avg[0]:.4f}, {accel_avg[1]:.4f}, {accel_avg[2]:.4f}] m/s²')

        # Normalize to get gravity direction in IMU/LiDAR body frame
        a_body_norm = accel_avg / (np.linalg.norm(accel_avg) + 1e-12)

        self.get_logger().info(
            f'Gravity direction (normalized): '
            f'[{a_body_norm[0]:.4f}, {a_body_norm[1]:.4f}, {a_body_norm[2]:.4f}]')

        # Compute pitch correction matrix from gravity measurement
        # This is the same approach used in the original odom_tilt_corrector.py
        R_map, pitch_angle = self.compute_pitch_correction_matrix(a_body_norm)

        pitch_deg = math.degrees(pitch_angle)

        self.get_logger().info('')
        self.get_logger().info(f'Estimated LiDAR mount pitch: {pitch_deg:.2f}°')

        # Verify: gravity should now point to [0, 0, -g] after correction
        g_corrected = R_map @ a_body_norm
        self.get_logger().info(
            f'Gravity after correction: '
            f'[{g_corrected[0]:.4f}, {g_corrected[1]:.4f}, {g_corrected[2]:.4f}]')
        self.get_logger().info(f'  (should be close to [0, 0, -1])')

        # Save calibration
        self._save_calibration(R_map, accel_avg, pitch_deg)

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

    @staticmethod
    def compute_pitch_correction_matrix(gravity_vec):
        """
        Compute PURE PITCH correction around Y-axis from gravity measurement.
        
        This is the same algorithm used in the original odom_tilt_corrector.py.
        
        The LiDAR is mounted tilted forward (pitch). We want to find the rotation
        that makes the measured gravity vector point straight down [0, 0, -1].
        
        Input: gravity_vec = gravity direction measured in LiDAR body frame (normalized)
        Output: (R_map, pitch_angle) where R_map rotates LiDAR frame to level frame
        
        In robot body frame (level), gravity should point to [0, 0, -1] (down in Z).
        """
        g = np.array(gravity_vec, dtype=float)
        g = g / (np.linalg.norm(g) + 1e-12)  # Ensure normalized

        # If gravity already points down in Z, no correction needed
        if abs(g[2] + 1.0) < 1e-6:
            return np.eye(3), 0.0

        # Extract pitch angle: rotation around Y-axis needed to align gravity to -Z
        # Project gravity onto XZ plane to get pitch
        # pitch = angle from -Z axis to gravity vector in XZ plane
        gx, gz = g[0], g[2]
        
        # atan2(gx, -gz) gives the angle from -Z to gravity in XZ plane
        # This is the pitch angle we need to correct
        pitch_angle = math.atan2(gx, -gz)

        # Build rotation matrix around Y-axis by -pitch_angle (to undo the tilt)
        cp = math.cos(-pitch_angle)
        sp = math.sin(-pitch_angle)
        
        R_pitch = np.array([
            [cp,  0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp]
        ], dtype=float)

        return R_pitch, pitch_angle

    def _save_calibration(self, R_map, accel_avg, pitch_deg):
        """Save calibration results to .npz file."""
        try:
            # Ensure output directory exists
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)

            timestamp = datetime.now()
            timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S")

            # Save in format compatible with raw_map_saver.cpp and odom_tilt_corrector.py
            # R_map: rotation matrix to transform from LiDAR frame to gravity-aligned frame
            # p0_world: origin offset (zero for calibration - will be set at runtime)
            np.savez(
                self.output_file,
                R_map=R_map,
                p0_world=np.zeros(3),  # Origin set at runtime by odom_tilt_corrector
                accel_avg=accel_avg,
                pitch_deg=pitch_deg,
                num_samples=self.accel_count,
                timestamp=timestamp_str
            )

            # Also save human-readable CSV
            csv_file = self.output_file.replace('.npz', '.csv')
            with open(csv_file, 'w') as f:
                f.write('# Tilt Calibration Results\n')
                f.write(f'# Generated: {timestamp.strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write(f'# Samples collected: {self.accel_count}\n')
                f.write('#\n')
                f.write(f'# Estimated mount pitch angle:\n')
                f.write(f'pitch_deg,{pitch_deg:.4f}\n')
                f.write('#\n')
                f.write('# Averaged acceleration (m/s²)\n')
                f.write(f'accel_x,{accel_avg[0]:.6f}\n')
                f.write(f'accel_y,{accel_avg[1]:.6f}\n')
                f.write(f'accel_z,{accel_avg[2]:.6f}\n')
                f.write('#\n')
                f.write('# R_map (pitch correction rotation matrix)\n')
                for i in range(3):
                    f.write(f'{R_map[i,0]:.8f},{R_map[i,1]:.8f},{R_map[i,2]:.8f}\n')

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
