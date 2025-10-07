#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState


class MoveOneMeter(Node):
    def __init__(self) -> None:
        super().__init__('move_one_meter')

        # Parameters
        self.declare_parameter('target_distance', 1.0)  # meters
        self.declare_parameter('max_speed', 0.3)        # m/s cruise
        self.declare_parameter('max_accel', 0.5)        # m/s^2
        self.declare_parameter('stop_tolerance', 0.005) # m
        # Drive parameters (match diff_drive_controller defaults)
        self.declare_parameter('wheel_radius', 0.072)   # m
        self.declare_parameter('wheel_base', 0.35)      # m
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        # Topics
        # Odom not used; distance integrates from ODrive feedback
        self.declare_parameter('left_control_topic', '/left/control_message')
        self.declare_parameter('right_control_topic', '/right/control_message')
        self.declare_parameter('left_status_topic', '/left/controller_status')
        self.declare_parameter('right_status_topic', '/right/controller_status')

        self.target_distance: float = float(self.get_parameter('target_distance').value)
        self.max_speed: float = float(self.get_parameter('max_speed').value)
        self.max_accel: float = float(self.get_parameter('max_accel').value)
        self.stop_tol: float = float(self.get_parameter('stop_tolerance').value)
        self.wheel_radius: float = float(self.get_parameter('wheel_radius').value)
        self.wheel_base: float = float(self.get_parameter('wheel_base').value)
        self.gear_ratio: float = float(self.get_parameter('gear_ratio').value)
        self.invert_left: bool = bool(self.get_parameter('invert_left').value)
        self.invert_right: bool = bool(self.get_parameter('invert_right').value)
        self.left_control_topic: str = str(self.get_parameter('left_control_topic').value)
        self.right_control_topic: str = str(self.get_parameter('right_control_topic').value)
        self.left_status_topic: str = str(self.get_parameter('left_status_topic').value)
        self.right_status_topic: str = str(self.get_parameter('right_status_topic').value)

        # Interfaces
        # Low-level motor commands to ODrive
        self.left_motor_pub = self.create_publisher(ControlMessage, self.left_control_topic, 10)
        self.right_motor_pub = self.create_publisher(ControlMessage, self.right_control_topic, 10)
        # ODrive feedback for distance integration
        self.left_status_sub = self.create_subscription(ControllerStatus, self.left_status_topic, self.left_status_cb, 20)
        self.right_status_sub = self.create_subscription(ControllerStatus, self.right_status_topic, self.right_status_cb, 20)
        # Axis arming services
        self.left_axis_client = self.create_client(AxisState, '/left/request_axis_state')
        self.right_axis_client = self.create_client(AxisState, '/right/request_axis_state')

        # State
        self.total_travel: float = 0.0
        self.current_speed_cmd: float = 0.0
        self.finished: bool = False

        # Measured motor velocities (turns/s) from ODrive
        self.current_left_motor_rps: float = 0.0
        self.current_right_motor_rps: float = 0.0

        # Control timer
        self.timer = self.create_timer(0.02, self.control_step)  # 50 Hz
        self.last_control_time = self.get_clock().now()

        # Arm ODrive axes shortly after start
        self.arm_timer = self.create_timer(0.2, self._arm_once)

        self.get_logger().info(
            f"move_one_meter running: target={self.target_distance:.3f} m, vmax={self.max_speed:.2f} m/s, amax={self.max_accel:.2f} m/s^2"
        )

    def left_status_cb(self, msg: ControllerStatus) -> None:
        self.current_left_motor_rps = float(msg.vel_estimate)

    def right_status_cb(self, msg: ControllerStatus) -> None:
        self.current_right_motor_rps = float(msg.vel_estimate)

    def _arm_once(self) -> None:
        # Cancel this timer after first call
        try:
            # Wait for services briefly
            if not self.left_axis_client.wait_for_service(timeout_sec=0.1) or not self.right_axis_client.wait_for_service(timeout_sec=0.1):
                return
            req = AxisState.Request()
            req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            self.left_axis_client.call_async(req)
            self.right_axis_client.call_async(req)
            # Ensure this timer doesn't run again
            if self.arm_timer is not None:
                self.arm_timer.cancel()
        except Exception as exc:
            self.get_logger().warn(f"Arming failed: {exc}")

    def control_step(self) -> None:
        if self.finished:
            return

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_control_time = now

        # Integrate distance purely from ODrive feedback
        left_rps = -self.current_left_motor_rps if self.invert_left else self.current_left_motor_rps
        right_rps = -self.current_right_motor_rps if self.invert_right else self.current_right_motor_rps
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        left_mps = (left_rps / self.gear_ratio) * wheel_circ
        right_mps = (right_rps / self.gear_ratio) * wheel_circ
        vx = 0.5 * (left_mps + right_mps)
        self.total_travel += max(0.0, vx) * dt
        remaining = max(0.0, self.target_distance - self.total_travel)

        # Stop condition
        if remaining <= self.stop_tol:
            self.publish_speed(0.0)
            self.finished = True
            self.get_logger().info(f"Reached target: traveled {self.total_travel:.3f} m")
            # Hold zero for a short moment to ensure controllers decay
            self.create_timer(0.3, lambda: None)
            return

        # Trapezoidal profile target speed limited by stopping distance: v_target = min(vmax, sqrt(2 a d))
        target_speed = min(self.max_speed, math.sqrt(max(0.0, 2.0 * self.max_accel * remaining)))

        # Speed ramp toward target with acceleration limits
        speed_error = target_speed - self.current_speed_cmd
        max_delta = self.max_accel * dt
        delta = max(-max_delta, min(max_delta, speed_error))
        self.current_speed_cmd += delta

        # Publish low-level velocity command to ODrive for straight motion
        self.publish_drive_velocity(self.current_speed_cmd)

    def publish_drive_velocity(self, linear_vx_mps: float) -> None:
        # Straight motion: no angular component
        right_wheel_mps = linear_vx_mps
        left_wheel_mps = linear_vx_mps

        wheel_circ = 2.0 * math.pi * self.wheel_radius
        right_wheel_turns_per_s = right_wheel_mps / wheel_circ
        left_wheel_turns_per_s = left_wheel_mps / wheel_circ

        right_motor_rps_cmd = right_wheel_turns_per_s * self.gear_ratio
        left_motor_rps_cmd = left_wheel_turns_per_s * self.gear_ratio

        msg_right = ControlMessage()
        msg_right.control_mode = 2  # VELOCITY_CONTROL
        msg_right.input_mode = 2    # VEL_RAMP
        msg_right.input_torque = 0.0
        msg_right.input_vel = -right_motor_rps_cmd if self.invert_right else right_motor_rps_cmd

        msg_left = ControlMessage()
        msg_left.control_mode = 2
        msg_left.input_mode = 2
        msg_left.input_torque = 0.0
        msg_left.input_vel = -left_motor_rps_cmd if self.invert_left else left_motor_rps_cmd

        self.right_motor_pub.publish(msg_right)
        self.left_motor_pub.publish(msg_left)

    def send_zero_torque(self) -> None:
        msg = ControlMessage()
        msg.control_mode = 1  # TORQUE_CONTROL
        msg.input_mode = 1    # PASSTHROUGH
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        self.left_motor_pub.publish(msg)
        self.right_motor_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MoveOneMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_torque()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


