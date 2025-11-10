#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math


class OdomTiltCorrector(Node):
    def __init__(self):
        super().__init__('odom_tilt_corrector')

        # Parameters (match pose_controller.py naming)
        self.declare_parameter('odometry_topic', '/Odometry')
        self.declare_parameter('accel_topic', '/livox/imu')
        self.declare_parameter('accel_samples', 10)
        self.declare_parameter('corrected_odometry_topic', '/Odometry_tilt_corrected_diff')

        self.odom_topic = str(self.get_parameter('odometry_topic').value)
        self.accel_topic = str(self.get_parameter('accel_topic').value)
        self.accel_samples_target = max(1, int(self.get_parameter('accel_samples').value))
        self.output_topic = str(self.get_parameter('corrected_odometry_topic').value)

        # State
        self.accel_initialized = False
        self.accel_sum = np.zeros(3, dtype=float)
        self.accel_count = 0
        self.accel_avg = None
        self.pose_initialized = False

        self.alignment_set = False
        self.origin_set = False
        self.R_map = np.eye(3, dtype=float)
        self.p0_world = np.zeros(3, dtype=float)
        self.align_quat = (0.0, 0.0, 0.0, 1.0)

        # Subscriptions (sensor-data QoS for IMU and odometry)
        self.imu_sub = self.create_subscription(
            Imu, self.accel_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)

        self.get_logger().info(
            f'Odom tilt corrector started | odom="{self.odom_topic}" imu="{self.accel_topic}" '
            f'→ out="{self.output_topic}" (N={self.accel_samples_target})'
        )

    # ---------------- Pose controller math (mirrored) ----------------
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
            return OdomTiltCorrector.quat_from_axis_angle(axis, math.pi)
        axis = np.cross(v1, v2)
        s = math.sqrt((1.0 + cos_theta) * 2.0)
        invs = 1.0 / s
        w = 0.5 * s
        x = axis[0] * invs
        y = axis[1] * invs
        z = axis[2] * invs
        return OdomTiltCorrector.quat_normalize((x, y, z, w))

    @staticmethod
    def rpy_to_matrix(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll);  sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)
        cy = math.cos(yaw);   sy = math.sin(yaw)
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

    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ---------------- IMU averaging ----------------
    def imu_callback(self, msg: Imu):
        a = np.array([
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        ], dtype=float)
        if not np.all(np.isfinite(a)):
            return
        if not self.accel_initialized:
            self.accel_sum += a
            self.accel_count += 1
            if self.accel_count < self.accel_samples_target:
                return
            self.accel_avg = self.accel_sum / float(max(1, self.accel_count))
            self.accel_initialized = True
            self.get_logger().info(
                f'IMU accel averaged (N={self.accel_count}): '
                f'{self.accel_avg[0]:.3f},{self.accel_avg[1]:.3f},{self.accel_avg[2]:.3f} — waiting first odom'
            )
        else:
            return

    # ---------------- Odom processing ----------------
    def odom_callback(self, msg: Odometry):
        """
        Callback for Fast-LIO2 odometry messages.
        Extracts x, y, yaw from the odometry message.
        """
        # Gate processing until IMU-based tilt alignment has completed
        if not self.accel_initialized:
            return
        # Extract raw pose
        raw_pos = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        ], dtype=float)
        raw_q = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        self.last_raw_q = raw_q
        self.last_raw_pos = raw_pos
        R_wb = self.quat_to_matrix(raw_q)

        # Establish leveling rotation at first odom using accel_avg and current body orientation
        if self.accel_initialized and not self.alignment_set:
            try:
                a_norm = self.accel_avg / (np.linalg.norm(self.accel_avg) + 1e-12)
                R_wb_curr = self.quat_to_matrix(raw_q)
                a_world = R_wb_curr @ a_norm
            except Exception:
                a_world = self.accel_avg / (np.linalg.norm(self.accel_avg) + 1e-12)
            R_flip = np.array([[-1.0, 0.0, 0.0],
                               [ 0.0, 1.0, 0.0],
                               [ 0.0, 0.0,-1.0]], dtype=float)
            # Perform alignment first in world frame, then apply fixed flip in mapping
            q_align_world = self.compute_alignment_quat(a_world, np.array([0.0, 0.0, -1.0]))
            self.align_quat = q_align_world
            rx, ry, rz = self.quaternion_to_rpy(q_align_world[0], q_align_world[1], q_align_world[2], q_align_world[3])
            R_align = self.rpy_to_matrix(rx, ry, rz)
            self.R_map = R_flip @ R_align
            self.alignment_set = True
            try:
                qx, qy, qz, qw = q_align_world
                roll, pitch, yaw = self.quaternion_to_rpy(qx, qy, qz, qw)
                a_flat_dbg = self.R_map @ a_world
                self.get_logger().info(
                    f'Align+flip at first odom: a_world={a_world[0]:.3f},{a_world[1]:.3f},{a_world[2]:.3f}; '
                    f'a_flat={a_flat_dbg[0]:.3f},{a_flat_dbg[1]:.3f},{a_flat_dbg[2]:.3f}; '
                    f'align_rpy(deg)={math.degrees(roll):.2f},{math.degrees(pitch):.2f},{math.degrees(yaw):.2f}')
            except Exception:
                pass

        # Establish origin in leveled world
        if not self.origin_set and self.alignment_set:
            self.p0_world = raw_pos.copy()
            self.origin_set = True

        # Leveled world position and orientation (matrix-based) with flip+align mapping
        p_local = self.R_map @ (raw_pos - self.p0_world)
        try:
            R_wb_curr = self.quat_to_matrix(raw_q)
            R_fb = self.R_map @ R_wb_curr
            q_local = self.matrix_to_quat(R_fb)
            rL, pL, yL = self.quaternion_to_rpy(q_local[0], q_local[1], q_local[2], q_local[3])
            if not hasattr(self, '_logged_q_choice'):
                self.get_logger().info(
                    f'Orientation mapping: R_fb = R_map * R_wb (roll,pitch deg)={math.degrees(rL):.2f},{math.degrees(pL):.2f}')
                self._logged_q_choice = True
        except Exception:
            q_local = self.quat_multiply(self.align_quat, raw_q)

        self.current_x = float(p_local[0])
        self.current_y = float(p_local[1])
        # z not used
        self.current_yaw = self.quaternion_to_yaw(q_local[0], q_local[1], q_local[2], q_local[3])
        
        # Extract and transform velocities for consistency with pose transformation
        raw_vel = np.array([
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z)
        ], dtype=float)
        vel_local = self.R_map @ raw_vel
        try:
            # One-time diagnostic: check z vs x slope sign changes by logging the ratio
            if not hasattr(self, '_logged_slope_hint') and abs(float(p_local[0])) > 1e-6:
                slope = float(p_local[2]) / float(p_local[0])
                self.get_logger().info(f'Leveled frame slope hint: dz/dx={slope:.4f}')
                self._logged_slope_hint = True
        except Exception:
            pass
        self.current_vx = float(vel_local[0])
        self.current_vy = float(vel_local[1])
        # Angular velocity is not transformed (rotation around Z-axis is preserved)
        self.current_vyaw = msg.twist.twist.angular.z
        
        # Mark pose as initialized
        if not self.pose_initialized:
            self.pose_initialized = True
            self.get_logger().info(
                f'✓ Odometry initialized: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, yaw={math.degrees(self.current_yaw):.1f}°'
            )
            try:
                qx, qy, qz, qw = self.align_quat
                roll, pitch, yaw = self.quaternion_to_rpy(qx, qy, qz, qw)
                self.get_logger().info(
                    f'align_quat: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}) | '
                    f'rpy=({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)'
                )
            except Exception:
                pass
        
        # Update last odometry time
        self.last_odom_time = self.get_clock().now()

        # Publish corrected odometry
        try:
            odom_corr = Odometry()
            odom_corr.header.stamp = msg.header.stamp
            odom_corr.header.frame_id = msg.header.frame_id
            # child_frame_id preserved
            try:
                odom_corr.child_frame_id = msg.child_frame_id
            except Exception:
                odom_corr.child_frame_id = ''
            odom_corr.pose.pose.position.x = self.current_x
            odom_corr.pose.pose.position.y = self.current_y
            odom_corr.pose.pose.position.z = float(p_local[2])
            odom_corr.pose.pose.orientation.x = q_local[0]
            odom_corr.pose.pose.orientation.y = q_local[1]
            odom_corr.pose.pose.orientation.z = q_local[2]
            odom_corr.pose.pose.orientation.w = q_local[3]
            # Rotate linear twist into the flat, tilt-corrected local frame
            # (use the already computed vel_local)
            try:
                odom_corr.twist.twist.linear.x = float(vel_local[0])
                odom_corr.twist.twist.linear.y = float(vel_local[1])
                odom_corr.twist.twist.linear.z = 0.0
            except Exception:
                odom_corr.twist.twist.linear.x = 0.0
                odom_corr.twist.twist.linear.y = 0.0
                odom_corr.twist.twist.linear.z = 0.0
            # Keep angular twist as-is (planar control uses z)
            odom_corr.twist.twist.angular = msg.twist.twist.angular
            self.odom_pub.publish(odom_corr)
        except Exception:
            pass

    # ---------------- Utilities ----------------
    @staticmethod
    def quaternion_to_rpy(qx: float, qy: float, qz: float, qw: float):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdomTiltCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


