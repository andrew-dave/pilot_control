#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class OdomGapMonitor(Node):
    def __init__(self):
        super().__init__('odom_gap_monitor')

        # Parameters
        self.declare_parameter('odometry_topic', '/Odometry')
        odom_topic = self.get_parameter('odometry_topic').value

        # State
        self._prev_arrival_ns = None

        # Subscriber (SensorDataQoS: best effort, keep_last=1)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(f'OdomGapMonitor listening on {odom_topic}')

    def _on_odom(self, msg: Odometry):
        # Current message header stamp in ns
        header_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        # Now (arrival) in ns
        now_ns = int(self.get_clock().now().nanoseconds)
        age_ms = (now_ns - header_ns) / 1e6

        if self._prev_arrival_ns is not None:
            arrival_gap_ms = (now_ns - self._prev_arrival_ns) / 1e6
            self.get_logger().info(f'age={age_ms:.1f} ms, arrival_gap={arrival_gap_ms:.1f} ms')
        else:
            self.get_logger().info(f'age={age_ms:.1f} ms')

        self._prev_arrival_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = OdomGapMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


