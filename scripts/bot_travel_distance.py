#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControllerStatus


class BotTravelDistance(Node):
    def __init__(self) -> None:
        super().__init__('bot_travel_distance')

        # Parameters
        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('right_ns', 'right')
        self.declare_parameter('wheel_radius', 0.072)  # m
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        self.declare_parameter('print_rate_hz', 2.0)

        self.left_ns = str(self.get_parameter('left_ns').value)
        self.right_ns = str(self.get_parameter('right_ns').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.print_rate_hz = float(self.get_parameter('print_rate_hz').value)

        # Subscriptions
        self.create_subscription(ControllerStatus, f'/{self.left_ns}/controller_status', self.left_cb, 20)
        self.create_subscription(ControllerStatus, f'/{self.right_ns}/controller_status', self.right_cb, 20)

        # State (motor turns)
        self.left_start = None
        self.right_start = None
        self.left_latest = None
        self.right_latest = None

        self.create_timer(1.0 / max(0.1, self.print_rate_hz), self.print_distance)
        self.get_logger().info('Listening on left/right controller_status to compute robot distance')

    def left_cb(self, msg: ControllerStatus) -> None:
        pos = float(msg.pos_estimate)
        if self.left_start is None:
            self.left_start = pos
        self.left_latest = pos

    def right_cb(self, msg: ControllerStatus) -> None:
        pos = float(msg.pos_estimate)
        if self.right_start is None:
            self.right_start = pos
        self.right_latest = pos

    def print_distance(self) -> None:
        if self.left_latest is None and self.right_latest is None:
            return

        wheel_circ = 2.0 * math.pi * self.wheel_radius

        distances_signed = []
        if self.left_latest is not None and self.left_start is not None:
            d_motor = self.left_latest - self.left_start
            d_motor = -d_motor if self.invert_left else d_motor
            d_wheel = d_motor / self.gear_ratio
            distances_signed.append(d_wheel * wheel_circ)

        if self.right_latest is not None and self.right_start is not None:
            d_motor = self.right_latest - self.right_start
            d_motor = -d_motor if self.invert_right else d_motor
            d_wheel = d_motor / self.gear_ratio
            distances_signed.append(d_wheel * wheel_circ)

        if not distances_signed:
            return

        # Robot forward distance ~ average of wheel distances for straight motion
        bot_distance_signed = sum(distances_signed) / len(distances_signed)
        bot_distance_abs = abs(bot_distance_signed)

        # Also report each side if both available
        if len(distances_signed) == 2:
            self.get_logger().info(
                f'Robot distance: {bot_distance_abs:.4f} m (signed {bot_distance_signed:.4f} m) | '
                f'L={distances_signed[0]:.4f} m R={distances_signed[1]:.4f} m')
        else:
            self.get_logger().info(
                f'Robot distance: {bot_distance_abs:.4f} m (signed {bot_distance_signed:.4f} m)')


def main() -> None:
    rclpy.init()
    node = BotTravelDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


