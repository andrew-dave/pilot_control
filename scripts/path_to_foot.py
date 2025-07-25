#!/usr/bin/env python3
"""Relay Fast-LIO2 /path into foot frame.
Publishes /path_foot with poses transformed to the static frame `foot`.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (needed for transforms)
from rclpy.duration import Duration

class PathRelay(Node):
    def __init__(self):
        super().__init__('path_to_foot')
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self)
        self.sub = self.create_subscription(Path, '/path', self.cb, 10)
        self.pub = self.create_publisher(Path, '/path_foot', 10)
        self.get_logger().info('path_to_foot relay running')

    def cb(self, msg: Path):
        out = Path()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'foot'
        for pose in msg.poses:
            try:
                transformed = self.buffer.transform(
                    pose, 'foot', timeout=Duration(seconds=0.1))
                out.poses.append(transformed)
            except Exception as exc:
                self.get_logger().warn(f'TF failed: {exc}', throttle_duration_sec=5.0)
                return
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(PathRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main() 