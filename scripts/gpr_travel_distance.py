#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState


class GPRTravelDistance(Node):
    def __init__(self) -> None:
        super().__init__('gpr_travel_distance')

        # Motion parameters (linear distance at ground contact)
        self.declare_parameter('target_distance', 1.0)   # meters; sign sets direction
        self.declare_parameter('max_speed', 0.4)         # m/s
        self.declare_parameter('max_accel', 0.8)         # m/s^2
        self.declare_parameter('stop_tolerance', 0.005)  # m

        # Kinematics for the GPR drive wheel
        self.declare_parameter('third_wheel_radius', 0.03)  # m (60 mm dia)
        self.declare_parameter('third_gear_ratio', 1.0)     # motor_turns = wheel_turns * gear
        self.declare_parameter('invert_third', False)

        # Topics/services for the GPR axis
        self.declare_parameter('gpr_control_topic', '/gpr/control_message')
        self.declare_parameter('gpr_status_topic', '/gpr/controller_status')
        self.declare_parameter('gpr_axis_service', '/gpr/request_axis_state')

        # Read parameters
        self.target_distance: float = float(self.get_parameter('target_distance').value)
        self.max_speed: float = float(self.get_parameter('max_speed').value)
        self.max_accel: float = float(self.get_parameter('max_accel').value)
        self.stop_tol: float = float(self.get_parameter('stop_tolerance').value)
        self.third_wheel_radius: float = float(self.get_parameter('third_wheel_radius').value)
        self.third_gear_ratio: float = float(self.get_parameter('third_gear_ratio').value)
        self.invert_third: bool = bool(self.get_parameter('invert_third').value)
        self.gpr_control_topic: str = str(self.get_parameter('gpr_control_topic').value)
        self.gpr_status_topic: str = str(self.get_parameter('gpr_status_topic').value)
        self.gpr_axis_service: str = str(self.get_parameter('gpr_axis_service').value)

        # Interfaces
        self.gpr_pub = self.create_publisher(ControlMessage, self.gpr_control_topic, 10)
        self.gpr_sub = self.create_subscription(ControllerStatus, self.gpr_status_topic, self.status_cb, 20)
        self.gpr_axis_client = self.create_client(AxisState, self.gpr_axis_service)

        # State
        self.total_travel_abs: float = 0.0
        self.current_speed_cmd: float = 0.0  # m/s (linear ground speed)
        self.finished: bool = False
        self.current_motor_rps: float = 0.0  # turns/s
        self.last_control_time = self.get_clock().now()

        # Derived direction (+1 forward, -1 reverse)
        self.direction_sign: float = 1.0 if self.target_distance >= 0.0 else -1.0
        self.remaining: float = abs(self.target_distance)

        # Control loop
        self.timer = self.create_timer(0.02, self.control_step)  # 50 Hz

        # Arm axis shortly after startup (one-shot)
        self.arm_timer = self.create_timer(0.2, self._arm_once)

        self.get_logger().info(
            f"gpr_travel_distance: target={self.target_distance:.3f} m, vmax={self.max_speed:.2f} m/s, amax={self.max_accel:.2f} m/s^2"
        )

    def _arm_once(self) -> None:
        try:
            if not self.gpr_axis_client.wait_for_service(timeout_sec=0.1):
                return
            req = AxisState.Request()
            req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            self.gpr_axis_client.call_async(req)
            if self.arm_timer is not None:
                self.arm_timer.cancel()
        except Exception as exc:
            self.get_logger().warn(f"Arming GPR axis failed: {exc}")

    def status_cb(self, msg: ControllerStatus) -> None:
        self.current_motor_rps = float(msg.vel_estimate)

    def control_step(self) -> None:
        if self.finished:
            return

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_control_time = now

        # Integrate absolute distance from motor speed
        motor_rps_meas = -self.current_motor_rps if self.invert_third else self.current_motor_rps
        wheel_circ = 2.0 * math.pi * self.third_wheel_radius
        wheel_mps = (motor_rps_meas / self.third_gear_ratio) * wheel_circ
        self.total_travel_abs += abs(wheel_mps) * dt

        self.remaining = max(0.0, abs(self.target_distance) - self.total_travel_abs)
        if self.remaining <= self.stop_tol:
            # Controlled stop at zero velocity in velocity mode
            self.publish_velocity_command(0.0)
            self.finished = True
            self.get_logger().info(f"GPR travel complete: {self.total_travel_abs:.3f} m")
            return

        # Trapezoidal profile: v_target = min(vmax, sqrt(2 a d_rem))
        v_target_abs = min(self.max_speed, math.sqrt(max(0.0, 2.0 * self.max_accel * self.remaining)))
        v_target_signed = self.direction_sign * v_target_abs

        # Accel-limited change
        speed_error = v_target_signed - self.current_speed_cmd
        max_delta = self.max_accel * dt
        delta = max(-max_delta, min(max_delta, speed_error))
        self.current_speed_cmd += delta

        # Send velocity command
        self.publish_velocity_command(self.current_speed_cmd)

    def publish_velocity_command(self, linear_vx_mps: float) -> None:
        # Convert desired ground linear velocity to motor rps
        wheel_circ = 2.0 * math.pi * self.third_wheel_radius
        wheel_turns_per_s = linear_vx_mps / wheel_circ
        motor_rps_cmd = wheel_turns_per_s * self.third_gear_ratio
        motor_rps_cmd = -motor_rps_cmd if self.invert_third else motor_rps_cmd

        msg = ControlMessage()
        msg.control_mode = 2  # VELOCITY_CONTROL
        msg.input_mode = 2    # VEL_RAMP
        msg.input_torque = 0.0
        msg.input_vel = float(motor_rps_cmd)
        self.gpr_pub.publish(msg)

    def send_zero_torque(self) -> None:
        msg = ControlMessage()
        msg.control_mode = 1  # TORQUE_CONTROL
        msg.input_mode = 1    # PASSTHROUGH
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        self.gpr_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GPRTravelDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Hold controller at 0 velocity then zero torque for safety
        node.publish_velocity_command(0.0)
        node.send_zero_torque()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


