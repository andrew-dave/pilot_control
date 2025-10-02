#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState


class DriveTravelDistance(Node):
    def __init__(self) -> None:
        super().__init__('drive_travel_distance')

        # Motion parameters
        self.declare_parameter('target_distance', 1.0)   # meters forward (+), back (-)
        self.declare_parameter('max_speed', 0.3)         # m/s
        self.declare_parameter('max_accel', 0.6)         # m/s^2
        self.declare_parameter('stop_tolerance', 0.005)  # m
        self.declare_parameter('velocity_multiplier', 1.0)
        self.declare_parameter('stop_timeout_ms', 500)
        self.declare_parameter('start_delay_s', 0.3)
        self.declare_parameter('feedback_required', False)
        self.declare_parameter('debug', True)

        # Kinematics (match diff_drive_controller)
        self.declare_parameter('wheel_radius', 0.072)
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)

        # Topics/services
        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('right_ns', 'right')

        # Read params
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.stop_tol = float(self.get_parameter('stop_tolerance').value)
        self.velocity_multiplier = float(self.get_parameter('velocity_multiplier').value)
        self.stop_timeout_ms = int(self.get_parameter('stop_timeout_ms').value)
        self.start_delay_s = float(self.get_parameter('start_delay_s').value)
        self.feedback_required = bool(self.get_parameter('feedback_required').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        left_ns = str(self.get_parameter('left_ns').value)
        right_ns = str(self.get_parameter('right_ns').value)

        # Interfaces
        self.left_pub = self.create_publisher(ControlMessage, f'/{left_ns}/control_message', 10)
        self.right_pub = self.create_publisher(ControlMessage, f'/{right_ns}/control_message', 10)
        self.left_sub = self.create_subscription(ControllerStatus, f'/{left_ns}/controller_status', self.left_status_cb, 20)
        self.right_sub = self.create_subscription(ControllerStatus, f'/{right_ns}/controller_status', self.right_status_cb, 20)
        self.left_axis = self.create_client(AxisState, f'/{left_ns}/request_axis_state')
        self.right_axis = self.create_client(AxisState, f'/{right_ns}/request_axis_state')

        # State
        self.last_control_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.received_left = False
        self.received_right = False
        self.last_left_time = self.get_clock().now()
        self.last_right_time = self.get_clock().now()
        self.left_pos_start = 0.0
        self.right_pos_start = 0.0
        self.left_pos_latest = 0.0
        self.right_pos_latest = 0.0
        self.have_left_start = False
        self.have_right_start = False
        self.total_travel_abs = 0.0
        self.current_speed_cmd = 0.0
        self.finished = False

        # Direction from requested distance
        self.direction_sign = 1.0 if self.target_distance >= 0.0 else -1.0
        self.remaining = abs(self.target_distance)

        # Timers
        self.timer = self.create_timer(0.02, self.control_step)
        self.arm_timer = self.create_timer(0.2, self._arm_once)

        self.get_logger().info(
            f"drive_travel_distance: target={self.target_distance:.3f} m vmax={self.max_speed:.2f} amax={self.max_accel:.2f} feedback_required={self.feedback_required}")

    def _arm_once(self) -> None:
        try:
            if not self.left_axis.wait_for_service(timeout_sec=0.2) or not self.right_axis.wait_for_service(timeout_sec=0.2):
                if self.debug:
                    self.get_logger().info("Waiting for left/right request_axis_state services...")
                return
            req = AxisState.Request()
            req.axis_requested_state = 8
            self.left_axis.call_async(req)
            self.right_axis.call_async(req)
            if self.debug:
                self.get_logger().info("Sent CLOSED_LOOP arming requests to left/right axes")
            if self.arm_timer:
                self.arm_timer.cancel()
        except Exception as exc:
            self.get_logger().warn(f"Arming failed: {exc}")

    def left_status_cb(self, msg: ControllerStatus) -> None:
        self.received_left = True
        self.last_left_time = self.get_clock().now()
        pos = float(msg.pos_estimate)
        if not self.have_left_start:
            self.left_pos_start = pos
            self.have_left_start = True
        self.left_pos_latest = pos
        if self.debug:
            self.get_logger().info(f"left status: pos={pos:.5f}")

    def right_status_cb(self, msg: ControllerStatus) -> None:
        self.received_right = True
        self.last_right_time = self.get_clock().now()
        pos = float(msg.pos_estimate)
        if not self.have_right_start:
            self.right_pos_start = pos
            self.have_right_start = True
        self.right_pos_latest = pos
        if self.debug:
            self.get_logger().info(f"right status: pos={pos:.5f}")

    def control_step(self) -> None:
        if self.finished:
            return

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_control_time = now

        # Startup delays/guards
        if (now - self.start_time).nanoseconds * 1e-9 < self.start_delay_s:
            if self.debug:
                self.get_logger().info("waiting: start_delay_s not elapsed; commanding 0.0")
            self.publish_velocity_command(0.0)
            return

        if self.feedback_required and not (self.received_left and self.received_right):
            if self.debug:
                self.get_logger().info("waiting: no left/right status yet; commanding 0.0")
            self.publish_velocity_command(0.0)
            return

        # Watchdog (active only once any feedback seen or feedback_required=True)
        if (self.received_left or self.feedback_required) and (self.received_right or self.feedback_required):
            ms_since_left = (now - self.last_left_time).nanoseconds * 1e-6
            ms_since_right = (now - self.last_right_time).nanoseconds * 1e-6
            if ms_since_left > float(self.stop_timeout_ms) or ms_since_right > float(self.stop_timeout_ms):
                if self.debug:
                    self.get_logger().info(
                        f"stale feedback: L={ms_since_left:.1f}ms R={ms_since_right:.1f}ms > {self.stop_timeout_ms}ms; 0.0")
                self.publish_velocity_command(0.0)
                return
        else:
            if self.debug:
                self.get_logger().info("no feedback yet; proceeding, watchdog disabled")

        # Distance from encoder delta (average of left/right)
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        distance_left = 0.0
        distance_right = 0.0
        if self.have_left_start:
            motor_turns_left = (self.left_pos_latest - self.left_pos_start)
            motor_turns_left = -motor_turns_left if self.invert_left else motor_turns_left
            distance_left = abs((motor_turns_left / self.gear_ratio) * wheel_circ)
        if self.have_right_start:
            motor_turns_right = (self.right_pos_latest - self.right_pos_start)
            motor_turns_right = -motor_turns_right if self.invert_right else motor_turns_right
            distance_right = abs((motor_turns_right / self.gear_ratio) * wheel_circ)

        # Use average distance for straight motion
        have_dist = self.have_left_start or self.have_right_start
        if have_dist:
            samples = []
            if self.have_left_start:
                samples.append(distance_left)
            if self.have_right_start:
                samples.append(distance_right)
            self.total_travel_abs = sum(samples) / len(samples)
            if self.debug:
                self.get_logger().info(
                    f"encoder: L={distance_left:.4f}m R={distance_right:.4f}m avg={self.total_travel_abs:.4f}m")

        self.remaining = max(0.0, abs(self.target_distance) - self.total_travel_abs)
        if self.remaining <= self.stop_tol:
            if self.debug:
                self.get_logger().info("target reached: commanding 0.0 and finishing")
            self.publish_velocity_command(0.0)
            self.finished = True
            self.get_logger().info(f"Drive travel complete: {self.total_travel_abs:.3f} m")
            return

        # Trapezoidal profile
        v_target_abs = min(self.max_speed, math.sqrt(max(0.0, 2.0 * self.max_accel * self.remaining)))
        v_target_signed = self.direction_sign * v_target_abs
        if self.debug:
            self.get_logger().info(
                f"profile: remaining={self.remaining:.3f} v_target={v_target_signed:.3f} cur_cmd={self.current_speed_cmd:.3f}")

        # Accel limit
        speed_error = v_target_signed - self.current_speed_cmd
        max_delta = self.max_accel * dt
        delta = max(-max_delta, min(max_delta, speed_error))
        self.current_speed_cmd += delta
        if self.debug:
            self.get_logger().info(f"accel-limit: delta={delta:.3f} new_cmd={self.current_speed_cmd:.3f}")

        # Publish straight velocity
        self.publish_velocity_command(self.current_speed_cmd)

    def publish_velocity_command(self, linear_vx_mps: float) -> None:
        # Convert straight m/s to wheel/motor rps and publish to both sides
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        wheel_turns_per_s = linear_vx_mps / wheel_circ
        motor_rps_cmd = wheel_turns_per_s * self.gear_ratio * self.velocity_multiplier

        right_cmd = -motor_rps_cmd if self.invert_right else motor_rps_cmd
        left_cmd = -motor_rps_cmd if self.invert_left else motor_rps_cmd

        msg_r = ControlMessage()
        msg_r.control_mode = 2
        msg_r.input_mode = 2
        msg_r.input_torque = 0.0
        msg_r.input_vel = float(right_cmd)

        msg_l = ControlMessage()
        msg_l.control_mode = 2
        msg_l.input_mode = 2
        msg_l.input_torque = 0.0
        msg_l.input_vel = float(left_cmd)

        self.right_pub.publish(msg_r)
        self.left_pub.publish(msg_l)

        if self.debug:
            self.get_logger().info(
                f"publish: vx={linear_vx_mps:.3f} -> L={left_cmd:.3f} rps R={right_cmd:.3f} rps")


def main() -> None:
    rclpy.init()
    node = DriveTravelDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity_command(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


