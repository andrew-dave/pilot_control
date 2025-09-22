#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import time

class GPRSerialBridge(Node):
    def __init__(self):
        super().__init__('gpr_serial_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameters
        self.serial_port: str = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate: int = int(self.get_parameter('baud_rate').get_parameter_value().integer_value)
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # State
        self.scan_active = False
        

        # Create services
        self.line_start_service = self.create_service(
            Trigger, 
            'gpr_line_start', 
            self.line_start_callback
        )
        
        self.line_stop_service = self.create_service(
            Trigger, 
            'gpr_line_stop', 
            self.line_stop_callback
        )
        
        self.get_logger().info(f'GPR Serial Bridge started on {self.serial_port}')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /gpr_line_start - Send "L" to Arduino')
        self.get_logger().info('  /gpr_line_stop  - Send "K" to Arduino')
    
    def connect_serial(self):
        """Connect to Arduino serial port"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Successfully connected to Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_conn = None
    
    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial_conn is None:
            self.get_logger().error('Serial connection not available')
            return False
        
        try:
            self.serial_conn.write(command.encode())
            self.get_logger().info(f'Sent command: "{command}" to Arduino')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False
    
    def line_start_callback(self, request, response):
        """Service callback for line start (send 'L')"""
        if self.send_command('L'):
            response.success = True
            response.message = 'Line start command sent to Arduino'
            self.scan_active = True
        else:
            response.success = False
            response.message = 'Failed to send line start command'
        return response
    
    def line_stop_callback(self, request, response):
        """Service callback for line stop (send 'K')"""
        if self.send_command('K'):
            response.success = True
            response.message = 'Line stop command sent to Arduino'
            self.scan_active = False
        else:
            response.success = False
            response.message = 'Failed to send line stop command'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GPRSerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 