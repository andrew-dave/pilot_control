#!/usr/bin/env python3
"""
motion_testing.py

Arms the drive motors once, then continuously publishes /cmd_vel based on user input.

Controls (stdin):
- Enter two numbers: "<linear_mps> <angular_radps>" to update velocities.
- Press 's' to instantly stop (publish 0,0 immediately).
- Press 'q' to quit (and optionally disarm).

Notes:
- Uses raw terminal mode to capture single key presses without Enter.
- Restores terminal settings on exit.
"""

import sys
import time
import argparse
import threading
import select
import termios
import tty
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.srv import AxisState


@contextmanager
def raw_stdin():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class MotionTester(Node):
    def __init__(self, linear: float, angular: float, rate_hz: float, arm_third: bool, disarm_on_exit: bool, arm_timeout_s: float):
        super().__init__('motion_tester')
        self.current_linear = float(linear)
        self.current_angular = float(angular)
        self.period = 1.0 / float(rate_hz if rate_hz > 0 else 10.0)
        self.disarm_on_exit = bool(disarm_on_exit)
        self.arm_timeout_s = float(arm_timeout_s)
        self.shutdown = False
        self.instant_stop_event = threading.Event()

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_axis_client = self.create_client(AxisState, '/left/request_axis_state')
        self.right_axis_client = self.create_client(AxisState, '/right/request_axis_state')
        self.gpr_axis_client = self.create_client(AxisState, '/gpr/request_axis_state') if arm_third else None

    def arm_axes_once(self) -> bool:
        req = AxisState.Request()
        req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL

        def call(client, name: str) -> bool:
            if not client.wait_for_service(timeout_sec=self.arm_timeout_s):
                self.get_logger().error(f'{name} AxisState service not available')
                return False
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.arm_timeout_s)
            if future.result() is None:
                self.get_logger().error(f'{name} arming call failed (timeout or error)')
                return False
            self.get_logger().info(f'{name} armed (CLOSED_LOOP)')
            return True

        ok = True
        ok &= call(self.left_axis_client, 'Left')
        ok &= call(self.right_axis_client, 'Right')
        if self.gpr_axis_client is not None:
            ok &= call(self.gpr_axis_client, 'GPR')
        return ok

    def disarm_axes(self):
        if not self.disarm_on_exit:
            return
        req = AxisState.Request()
        req.axis_requested_state = 1  # IDLE

        def safe_call(client, name: str):
            try:
                if client is None:
                    return
                if not client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn(f'{name} AxisState service not available for disarm')
                    return
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                self.get_logger().info(f'{name} disarm request sent (IDLE)')
            except Exception as e:
                self.get_logger().warn(f'Disarm call failed for {name}: {e}')

        safe_call(self.left_axis_client, 'Left')
        safe_call(self.right_axis_client, 'Right')
        if self.gpr_axis_client is not None:
            safe_call(self.gpr_axis_client, 'GPR')

    def publish_cmd(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def publish_zero(self):
        self.publish_cmd(0.0, 0.0)


def input_thread_fn(node: MotionTester):
    buf = ''
    node.get_logger().info("Input ready: type '<linear> <angular>' and Enter, or press 's' to STOP, 'q' to quit.")
    with raw_stdin():
        while not node.shutdown and rclpy.ok():
            # Non-blocking read from stdin
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue
            ch = sys.stdin.read(1)
            if not ch:
                continue
            if ch == 'q':
                node.shutdown = True
                break
            if ch == 's':
                node.instant_stop_event.set()
                continue
            if ch == '\n':
                line = buf.strip()
                buf = ''
                if not line:
                    continue
                try:
                    parts = line.split()
                    if len(parts) != 2:
                        node.get_logger().warn("Please enter two numbers: <linear> <angular>")
                        continue
                    lin = float(parts[0])
                    ang = float(parts[1])
                    node.current_linear = lin
                    node.current_angular = ang
                    node.get_logger().info(f"Updated velocities: linear={lin:.3f} m/s, angular={ang:.3f} rad/s")
                except Exception as e:
                    node.get_logger().warn(f"Parse error: {e}")
                continue
            else:
                buf += ch


def main():
    parser = argparse.ArgumentParser(description='Arm motors once and interactively publish cmd_vel')
    parser.add_argument('--linear', type=float, default=0.0, help='Initial linear velocity (m/s)')
    parser.add_argument('--angular', type=float, default=0.0, help='Initial angular velocity (rad/s)')
    parser.add_argument('--rate', type=float, default=20.0, help='Publish rate (Hz), default 20')
    parser.add_argument('--arm-third', action='store_true', help='Also arm GPR axis')
    parser.add_argument('--disarm', action='store_true', help='Disarm axes on exit')
    parser.add_argument('--arm-timeout', type=float, default=3.0, help='Service wait/arm timeout (s)')
    args = parser.parse_args()

    rclpy.init(args=None)
    node = MotionTester(
        linear=args.linear,
        angular=args.angular,
        rate_hz=args.rate,
        arm_third=args.arm_third,
        disarm_on_exit=args.disarm,
        arm_timeout_s=args.arm_timeout,
    )

    try:
        if not node.arm_axes_once():
            node.get_logger().error('Arming failed; exiting.')
            node.destroy_node()
            rclpy.shutdown()
            return

        # Start input thread
        th = threading.Thread(target=input_thread_fn, args=(node,), daemon=True)
        th.start()

        node.get_logger().info('Publishing started. Press q to quit.')
        while rclpy.ok() and not node.shutdown:
            if node.instant_stop_event.is_set():
                node.publish_zero()
                node.instant_stop_event.clear()
            else:
                node.publish_cmd(node.current_linear, node.current_angular)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(node.period)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        try:
            node.publish_zero()
            node.disarm_axes()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()


