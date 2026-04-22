#!/usr/bin/env python3
"""
Test script for F2C to Pose Controller integration.

This script tests the new waypoint publishing functionality:
1. Simulates F2C GUI publishing waypoints to /f2c_waypoints
2. Checks that pose controller receives them
3. Tests navigation start signal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

class F2CIntegrationTester(Node):
    def __init__(self):
        super().__init__('f2c_integration_tester')

        # Publisher to simulate F2C GUI
        self.f2c_pub = self.create_publisher(
            Float64MultiArray,
            '/f2c_waypoints',
            10
        )

        self.get_logger().info('F2C Integration Tester ready')
        self.get_logger().info('Testing waypoint publishing...')

        # Test waypoints (simple square pattern)
        self.test_waypoints = [
            (0.0, 0.0, 0.0, 0.0),      # Start at origin
            (2.0, 0.0, 0.0, 0.0),      # Move +X
            (2.0, 2.0, 0.0, math.pi/2), # Move +Y, face north
            (0.0, 2.0, 0.0, math.pi),   # Move -X, face west
            (0.0, 0.0, 0.0, -math.pi/2) # Return to start, face south
        ]

        # Start test sequence
        self.start_test()

    def start_test(self):
        """Run the test sequence"""
        self.get_logger().info('=== F2C Integration Test Started ===')

        # Step 1: Publish waypoints (simulate "Publish Waypoints" button)
        self.get_logger().info('Step 1: Publishing waypoints...')
        msg = Float64MultiArray()
        for wp in self.test_waypoints:
            msg.data.extend(wp)
        self.f2c_pub.publish(msg)

        # Wait a bit for pose controller to process
        time.sleep(2.0)

        # Step 2: Send navigation start signal (simulate "Start Navigation" button)
        self.get_logger().info('Step 2: Sending navigation start signal...')
        start_msg = Float64MultiArray()
        start_msg.data = [0.0]  # Special signal value
        self.f2c_pub.publish(start_msg)

        self.get_logger().info('=== Test completed - check pose controller output ===')

def main(args=None):
    rclpy.init(args=args)
    node = F2CIntegrationTester()

    try:
        # Run for a short time then exit
        rclpy.spin_once(node, timeout_sec=5.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

