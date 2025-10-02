#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControllerStatus


class LeftWheelDistance(Node):
    def __init__(self) -> None:
        super().__init__('left_wheel_distance')

        # Parameters matching diff_drive_controller/gpr nodes
        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('wheel_radius', 0.072)  # m
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('print_rate_hz', 2.0)

        self.left_ns = str(self.get_parameter('left_ns').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.print_rate_hz = float(self.get_parameter('print_rate_hz').value)

        topic = f'/{self.left_ns}/controller_status'
        self.sub = self.create_subscription(ControllerStatus, topic, self.status_cb, 20)

        self._pos_start = None  # motor turns at start
        self._pos_latest = None

        self.create_timer(1.0 / max(0.1, self.print_rate_hz), self.print_distance)
        self.get_logger().info(f'Listening on {topic} to compute left wheel distance')

    def status_cb(self, msg: ControllerStatus) -> None:
        pos = float(msg.pos_estimate)  # motor turns
        if self._pos_start is None:
            self._pos_start = pos
        self._pos_latest = pos

    def print_distance(self) -> None:
        if self._pos_start is None or self._pos_latest is None:
            return
        motor_turns_delta = self._pos_latest - self._pos_start
        motor_turns_delta = -motor_turns_delta if self.invert_left else motor_turns_delta
        wheel_turns_delta = motor_turns_delta / self.gear_ratio
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        distance_m_signed = wheel_turns_delta * wheel_circ
        distance_m_abs = abs(distance_m_signed)
        self.get_logger().info(
            f'Left wheel distance: {distance_m_abs:.4f} m (signed {distance_m_signed:.4f} m)')


def main() -> None:
    rclpy.init()
    node = LeftWheelDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


