#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
import math
import time

class GPRPositionTester(Node):
    def __init__(self):
        super().__init__('gpr_position_tester')
        
        # Declare parameters (matching diff_drive_controller.cpp)
        self.declare_parameter('third_wheel_radius', 0.03)  # 60 mm diameter -> 0.03 m radius
        self.declare_parameter('third_gear_ratio', 1.0)
        self.declare_parameter('velocity_multiplier', 1.0)
        self.declare_parameter('invert_third', False)
        
        # Test parameter: simulated travel distance
        self.declare_parameter('test_travel_distance_m', 3.0)
        self.declare_parameter('auto_arm', True)
        
        # Get parameters
        self.third_wheel_radius = self.get_parameter('third_wheel_radius').value
        self.third_gear_ratio = self.get_parameter('third_gear_ratio').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        self.invert_third = self.get_parameter('invert_third').value
        self.test_travel_distance_m = self.get_parameter('test_travel_distance_m').value
        self.auto_arm = self.get_parameter('auto_arm').value
        
        # Publisher and service client
        self.control_pub = self.create_publisher(ControlMessage, '/gpr/control_message', 10)
        self.axis_client = self.create_client(AxisState, '/gpr/request_axis_state')
        
        # Calculate circumference
        self.third_circ = 2.0 * math.pi * self.third_wheel_radius
        
        self.get_logger().info('=== GPR Position Control Tester ===')
        self.get_logger().info(f'Third wheel radius: {self.third_wheel_radius:.4f} m')
        self.get_logger().info(f'Third wheel circumference: {self.third_circ:.4f} m')
        self.get_logger().info(f'Third gear ratio: {self.third_gear_ratio}')
        self.get_logger().info(f'Velocity multiplier: {self.velocity_multiplier}')
        self.get_logger().info(f'Invert third: {self.invert_third}')
        self.get_logger().info(f'Test travel distance: {self.test_travel_distance_m:.3f} m')
        
        # Arm motor if requested
        if self.auto_arm:
            self.get_logger().info('Waiting for GPR ODrive service...')
            time.sleep(2)
            self.arm_motor()
        
        # Timer to publish position commands at 20 Hz (matching diff_drive_controller)
        self.timer = self.create_timer(0.05, self.update_position)
        
        self.get_logger().info('')
        self.get_logger().info('=== Commands ===')
        self.get_logger().info('Use ros2 param set to change travel distance in real-time:')
        self.get_logger().info('  ros2 param set /gpr_position_tester test_travel_distance_m 1.0')
        self.get_logger().info('')
        self.get_logger().info('To arm motor manually:')
        self.get_logger().info('  ros2 service call /gpr/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"')
        self.get_logger().info('')
        self.get_logger().info('To disarm (IDLE):')
        self.get_logger().info('  ros2 service call /gpr/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"')
        self.get_logger().info('')
        
    def arm_motor(self):
        """Arm the GPR motor to CLOSED_LOOP state"""
        if not self.axis_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('GPR ODrive service not available!')
            return
        
        request = AxisState.Request()
        request.axis_requested_state = 8  # CLOSED_LOOP
        
        future = self.axis_client.call_async(request)
        future.add_done_callback(self.arm_response_callback)
        
    def arm_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'GPR motor armed - State: {response.axis_state}, Errors: {response.active_errors}')
        except Exception as e:
            self.get_logger().error(f'Failed to arm GPR motor: {e}')
    
    def update_position(self):
        """Update GPR motor position based on test travel distance"""
        # Get current parameter value (allows real-time updates)
        self.test_travel_distance_m = self.get_parameter('test_travel_distance_m').value
        
        # Calculate target position in motor turns
        # Formula from diff_drive_controller.cpp line 356:
        # turns_target = (total_travel_m_ / third_circ) * third_gear_ratio_ * velocity_multiplier_
        turns_target = (self.test_travel_distance_m / self.third_circ) * self.third_gear_ratio * self.velocity_multiplier
        
        # Apply inversion if configured
        pos_cmd_turns = -turns_target if self.invert_third else turns_target
        
        # Create and publish position command
        msg = ControlMessage()
        msg.control_mode = 3  # POSITION_CONTROL
        msg.input_mode = 1    # PASSTHROUGH (as requested by user)
        msg.input_pos = pos_cmd_turns
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        
        self.control_pub.publish(msg)
        
        # Log every 2 seconds (40 cycles at 20 Hz)
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        if self.log_counter >= 40:
            self.log_counter = 0
            self.get_logger().info(
                f'Travel: {self.test_travel_distance_m:.3f} m → '
                f'Position: {pos_cmd_turns:.3f} turns '
                f'({pos_cmd_turns * 360:.1f}°)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GPRPositionTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

