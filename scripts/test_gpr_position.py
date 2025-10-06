#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage, ControllerStatus
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
        
        # Stepped motion parameters to prevent velocity runout
        self.declare_parameter('step_distance_threshold_m', 0.5)  # Move in steps if change > this
        self.declare_parameter('step_size_m', 0.3)  # Size of each step
        
        # Get parameters
        self.third_wheel_radius = self.get_parameter('third_wheel_radius').value
        self.third_gear_ratio = self.get_parameter('third_gear_ratio').value
        self.velocity_multiplier = self.get_parameter('velocity_multiplier').value
        self.invert_third = self.get_parameter('invert_third').value
        self.test_travel_distance_m = self.get_parameter('test_travel_distance_m').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.step_threshold_m = self.get_parameter('step_distance_threshold_m').value
        self.step_size_m = self.get_parameter('step_size_m').value
        
        # Publisher, subscriber, and service client
        self.control_pub = self.create_publisher(ControlMessage, '/gpr/control_message', 10)
        self.status_sub = self.create_subscription(
            ControllerStatus, '/gpr/controller_status', self.status_callback, 10)
        self.axis_client = self.create_client(AxisState, '/gpr/request_axis_state')
        
        # Calculate circumference
        self.third_circ = 2.0 * math.pi * self.third_wheel_radius
        
        # State tracking for stepped motion
        self.current_position_turns = 0.0  # Current motor position in turns
        self.target_distance_m = 0.0  # Final target distance
        self.current_target_distance_m = 0.0  # Current step target
        self.position_reached = True  # True when at target
        
        self.get_logger().info('=== GPR Position Control Tester (Stepped Motion) ===')
        self.get_logger().info(f'Third wheel radius: {self.third_wheel_radius:.4f} m')
        self.get_logger().info(f'Third wheel circumference: {self.third_circ:.4f} m')
        self.get_logger().info(f'Third gear ratio: {self.third_gear_ratio}')
        self.get_logger().info(f'Velocity multiplier: {self.velocity_multiplier}')
        self.get_logger().info(f'Invert third: {self.invert_third}')
        self.get_logger().info(f'Test travel distance: {self.test_travel_distance_m:.3f} m')
        self.get_logger().info(f'Step threshold: {self.step_threshold_m:.3f} m')
        self.get_logger().info(f'Step size: {self.step_size_m:.3f} m')
        
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
    
    def status_callback(self, msg):
        """Callback to receive current motor position"""
        self.current_position_turns = msg.pos_estimate
    
    def distance_to_turns(self, distance_m):
        """Convert travel distance to motor turns"""
        turns = (distance_m / self.third_circ) * self.third_gear_ratio * self.velocity_multiplier
        return -turns if self.invert_third else turns
    
    def turns_to_distance(self, turns):
        """Convert motor turns back to travel distance"""
        actual_turns = -turns if self.invert_third else turns
        return (actual_turns / (self.third_gear_ratio * self.velocity_multiplier)) * self.third_circ
    
    def update_position(self):
        """Update GPR motor position based on test travel distance with stepped motion"""
        # Get current parameter value (allows real-time updates)
        new_target = self.get_parameter('test_travel_distance_m').value
        
        # Check if target has changed
        if abs(new_target - self.target_distance_m) > 0.001:
            # New target set
            self.target_distance_m = new_target
            
            # Get current position in distance
            current_distance_m = self.turns_to_distance(self.current_position_turns)
            distance_error = self.target_distance_m - current_distance_m
            
            # Check if we need stepped motion
            if abs(distance_error) > self.step_threshold_m:
                # Large movement - use stepped approach
                # Calculate first step target
                step_direction = 1.0 if distance_error > 0 else -1.0
                self.current_target_distance_m = current_distance_m + (self.step_size_m * step_direction)
                self.position_reached = False
                self.get_logger().info(
                    f'📍 New target: {self.target_distance_m:.3f} m | '
                    f'Current: {current_distance_m:.3f} m | '
                    f'Error: {distance_error:.3f} m | '
                    f'→ Using stepped motion (step size: {self.step_size_m:.3f} m)'
                )
            else:
                # Small movement - go directly
                self.current_target_distance_m = self.target_distance_m
                self.position_reached = False
                self.get_logger().info(
                    f'📍 New target: {self.target_distance_m:.3f} m | '
                    f'Error: {distance_error:.3f} m | '
                    f'→ Direct motion (below threshold)'
                )
        
        # Update stepping logic if we're in stepped motion
        if not self.position_reached:
            current_distance_m = self.turns_to_distance(self.current_position_turns)
            remaining_to_final = self.target_distance_m - current_distance_m
            remaining_to_step = self.current_target_distance_m - current_distance_m
            
            # Check if we've reached current step target (within 0.02m = ~0.1 turns)
            if abs(remaining_to_step) < 0.02:
                # Reached step target, calculate next step
                if abs(remaining_to_final) < 0.02:
                    # Reached final target
                    self.position_reached = True
                    self.current_target_distance_m = self.target_distance_m
                    self.get_logger().info(f'✅ Target reached: {self.target_distance_m:.3f} m')
                elif abs(remaining_to_final) > self.step_threshold_m:
                    # Still need more steps
                    step_direction = 1.0 if remaining_to_final > 0 else -1.0
                    self.current_target_distance_m = current_distance_m + (self.step_size_m * step_direction)
                    self.get_logger().info(
                        f'⏭️  Step complete | Next step target: {self.current_target_distance_m:.3f} m | '
                        f'Remaining: {remaining_to_final:.3f} m'
                    )
                else:
                    # Last step - go directly to target
                    self.current_target_distance_m = self.target_distance_m
                    self.get_logger().info(
                        f'⏭️  Final step | Target: {self.target_distance_m:.3f} m | '
                        f'Remaining: {remaining_to_final:.3f} m'
                    )
        
        # Convert current target to turns and publish
        pos_cmd_turns = self.distance_to_turns(self.current_target_distance_m)
        
        # Create and publish position command
        msg = ControlMessage()
        msg.control_mode = 3  # POSITION_CONTROL
        msg.input_mode = 1    # PASSTHROUGH
        msg.input_pos = pos_cmd_turns
        msg.input_vel = 0.0
        msg.input_torque = 0.0
        
        self.control_pub.publish(msg)
        
        # Log status every 2 seconds (40 cycles at 20 Hz)
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        if self.log_counter >= 40:
            self.log_counter = 0
            current_distance_m = self.turns_to_distance(self.current_position_turns)
            self.get_logger().info(
                f'Current: {current_distance_m:.3f} m ({self.current_position_turns:.2f} turns) | '
                f'Step target: {self.current_target_distance_m:.3f} m | '
                f'Final target: {self.target_distance_m:.3f} m'
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

