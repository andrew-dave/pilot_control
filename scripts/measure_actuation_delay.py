#!/usr/bin/env python3
"""
Actuation Delay Measurement Tool
================================

This script measures the net actuation time delay in the robot's drive system,
including:
- Pure transport delay (dead time): Time from command sent to first response
- First-order time constant: How quickly velocity approaches setpoint
- Effective ramp rate: Actual velocity rate of change (if using VEL_RAMP mode)

Usage:
    ros2 run pilot_control measure_actuation_delay.py

    # With custom CAN interface
    ros2 run pilot_control measure_actuation_delay.py --ros-args -p can_interface:=can0

    # Test only one wheel
    ros2 run pilot_control measure_actuation_delay.py --ros-args -p test_wheel:=left

The script will:
1. Launch ODrive CAN nodes for left and right wheels
2. Arm motors into CLOSED_LOOP_CONTROL
3. Send step velocity commands to wheels
4. Record actual velocity from encoder feedback
5. Analyze response to extract delay parameters
6. Disarm motors and save results

⚠️  SAFETY: Robot should be safely supported with wheels free to spin!
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState

import numpy as np
import time
import math
import os
import subprocess
import signal
from datetime import datetime
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class StepResponseData:
    """Data from a single step response test."""
    wheel: str
    step_magnitude: float  # rev/s
    direction: str  # "up" or "down"
    timestamps: List[float] = field(default_factory=list)  # seconds from step
    velocities: List[float] = field(default_factory=list)  # rev/s
    command_time: float = 0.0
    
    
@dataclass
class DelayAnalysisResult:
    """Results from analyzing step response."""
    transport_delay: float  # seconds - time until response starts
    time_constant: float    # seconds - time to reach 63.2% of step
    settling_time: float    # seconds - time to reach 95% of step
    effective_ramp_rate: float  # rev/s² - average rate during ramp
    steady_state_error: float   # rev/s - difference from commanded
    

class ActuationDelayMeasurement(Node):
    """
    ROS2 node for measuring actuation delay.
    Launches ODrive nodes and handles motor arming/disarming automatically.
    """
    
    def __init__(self):
        super().__init__('actuation_delay_measurement')
        
        # Parameters
        self.declare_parameter('step_velocity', 0.5)       # rev/s step magnitude
        self.declare_parameter('hold_time', 1.5)           # seconds to hold each step
        self.declare_parameter('settle_time', 0.5)         # seconds to wait before step
        self.declare_parameter('num_trials', 3)            # number of trials per direction
        self.declare_parameter('test_wheel', 'both')       # 'left', 'right', or 'both'
        self.declare_parameter('input_mode', 2)            # 1=PASSTHROUGH, 2=VEL_RAMP
        self.declare_parameter('output_dir', '/tmp')       # where to save results
        self.declare_parameter('can_interface', 'can0')    # CAN interface name
        self.declare_parameter('left_node_id', 2)          # ODrive CAN node ID for left
        self.declare_parameter('right_node_id', 1)         # ODrive CAN node ID for right
        self.declare_parameter('launch_odrive_nodes', True)  # whether to launch ODrive nodes
        
        self.step_velocity = float(self.get_parameter('step_velocity').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.num_trials = int(self.get_parameter('num_trials').value)
        self.test_wheel = str(self.get_parameter('test_wheel').value)
        self.input_mode = int(self.get_parameter('input_mode').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.can_interface = str(self.get_parameter('can_interface').value)
        self.left_node_id = int(self.get_parameter('left_node_id').value)
        self.right_node_id = int(self.get_parameter('right_node_id').value)
        self.launch_odrive_nodes = bool(self.get_parameter('launch_odrive_nodes').value)
        
        # State
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_vel_time = 0.0
        self.right_vel_time = 0.0
        self.left_axis_state = 0
        self.right_axis_state = 0
        self.recording = False
        self.current_data: Optional[StepResponseData] = None
        
        # Results storage
        self.all_results: List[Tuple[StepResponseData, DelayAnalysisResult]] = []
        
        # Subprocess handles for ODrive nodes
        self.odrive_processes: List[subprocess.Popen] = []
        
        # Publishers
        self.left_pub = self.create_publisher(ControlMessage, '/left/control_message', 10)
        self.right_pub = self.create_publisher(ControlMessage, '/right/control_message', 10)
        
        # Service clients for arming/disarming
        self.left_axis_client = self.create_client(AxisState, '/left/request_axis_state')
        self.right_axis_client = self.create_client(AxisState, '/right/request_axis_state')
        
        # Subscribers
        self.create_subscription(
            ControllerStatus,
            '/left/controller_status',
            self.left_status_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            ControllerStatus,
            '/right/controller_status',
            self.right_status_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info("="*60)
        self.get_logger().info("  Actuation Delay Measurement Tool")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Step velocity: {self.step_velocity} rev/s")
        self.get_logger().info(f"Hold time: {self.hold_time} s")
        self.get_logger().info(f"Trials per direction: {self.num_trials}")
        self.get_logger().info(f"Test wheel(s): {self.test_wheel}")
        self.get_logger().info(f"Input mode: {'VEL_RAMP' if self.input_mode == 2 else 'PASSTHROUGH'}")
        self.get_logger().info(f"CAN interface: {self.can_interface}")
        self.get_logger().info(f"Left ODrive ID: {self.left_node_id}, Right ODrive ID: {self.right_node_id}")
        self.get_logger().info("="*60)
        
    def left_status_callback(self, msg: ControllerStatus):
        """Callback for left wheel status."""
        self.left_vel = float(msg.vel_estimate)
        self.left_vel_time = time.time()
        self.left_axis_state = int(msg.axis_state)
        
        if self.recording and self.current_data and self.current_data.wheel in ['left', 'both']:
            # Store absolute timestamp - will be converted to relative later
            self.current_data.timestamps.append(time.time())
            self.current_data.velocities.append(self.left_vel)
    
    def right_status_callback(self, msg: ControllerStatus):
        """Callback for right wheel status."""
        self.right_vel = float(msg.vel_estimate)
        self.right_vel_time = time.time()
        self.right_axis_state = int(msg.axis_state)
        
        if self.recording and self.current_data and self.current_data.wheel in ['right', 'both']:
            # Store absolute timestamp - will be converted to relative later
            self.current_data.timestamps.append(time.time())
            self.current_data.velocities.append(self.right_vel)
    
    def _start_odrive_nodes(self) -> bool:
        """Launch ODrive CAN nodes for left and right wheels."""
        if not self.launch_odrive_nodes:
            self.get_logger().info("Skipping ODrive node launch (launch_odrive_nodes=False)")
            return True
        
        self.get_logger().info("Launching ODrive CAN nodes...")
        
        try:
            # Launch left ODrive node
            left_cmd = [
                'ros2', 'run', 'odrive_can', 'odrive_can_node',
                '--ros-args',
                '-r', '__ns:=/left',
                '-r', '__node:=odrive_can_left',
                '-p', f'node_id:={self.left_node_id}',
                '-p', f'interface:={self.can_interface}'
            ]
            self.get_logger().info(f"  Starting left ODrive (ID={self.left_node_id})...")
            left_proc = subprocess.Popen(
                left_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            self.odrive_processes.append(left_proc)
            
            # Launch right ODrive node
            right_cmd = [
                'ros2', 'run', 'odrive_can', 'odrive_can_node',
                '--ros-args',
                '-r', '__ns:=/right',
                '-r', '__node:=odrive_can_right',
                '-p', f'node_id:={self.right_node_id}',
                '-p', f'interface:={self.can_interface}'
            ]
            self.get_logger().info(f"  Starting right ODrive (ID={self.right_node_id})...")
            right_proc = subprocess.Popen(
                right_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.odrive_processes.append(right_proc)
            
            self.get_logger().info("  ODrive nodes launched successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to launch ODrive nodes: {e}")
            return False
    
    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """Wait for ODrive axis state services to become available."""
        self.get_logger().info("Waiting for ODrive services...")
        
        start = time.time()
        while time.time() - start < timeout:
            # Use non-blocking wait - main thread handles spinning
            left_ready = self.left_axis_client.service_is_ready()
            right_ready = self.right_axis_client.service_is_ready()
            
            if left_ready and right_ready:
                self.get_logger().info("  ODrive services are ready!")
                return True
            
            time.sleep(0.2)
        
        self.get_logger().error("Timeout waiting for ODrive services")
        return False
    
    def wait_for_feedback(self, timeout: float = 5.0) -> bool:
        """Wait for velocity feedback from both wheels."""
        self.get_logger().info("Waiting for encoder feedback...")
        
        start = time.time()
        while time.time() - start < timeout:
            # Main thread handles spinning, we just check the timestamps
            now = time.time()
            left_recent = (now - self.left_vel_time) < 1.0 if self.left_vel_time > 0 else False
            right_recent = (now - self.right_vel_time) < 1.0 if self.right_vel_time > 0 else False
            
            if left_recent and right_recent:
                self.get_logger().info(f"  Receiving feedback: L={self.left_vel:.3f}, R={self.right_vel:.3f} rev/s")
                return True
            
            time.sleep(0.1)
        
        self.get_logger().error("Timeout waiting for encoder feedback")
        return False
    
    def arm_motors(self) -> bool:
        """Arm both motors into CLOSED_LOOP_CONTROL state."""
        self.get_logger().info("Arming motors (CLOSED_LOOP_CONTROL)...")
        
        try:
            # Request CLOSED_LOOP_CONTROL (state 8) for both axes
            req_left = AxisState.Request()
            req_left.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            
            req_right = AxisState.Request()
            req_right.axis_requested_state = 8  # CLOSED_LOOP_CONTROL
            
            # Send requests asynchronously
            future_left = self.left_axis_client.call_async(req_left)
            future_right = self.right_axis_client.call_async(req_right)
            
            # Wait for futures to complete (main thread handles spinning)
            timeout = 5.0
            start = time.time()
            while time.time() - start < timeout:
                if future_left.done() and future_right.done():
                    break
                time.sleep(0.1)
            
            # Check if successful
            time.sleep(0.5)  # Give time for state to update
            
            # Verify motors are armed (axis_state == 8)
            if self.left_axis_state == 8 and self.right_axis_state == 8:
                self.get_logger().info("  Motors armed successfully!")
                return True
            else:
                self.get_logger().warn(f"  Motor states: L={self.left_axis_state}, R={self.right_axis_state}")
                self.get_logger().warn("  Expected state 8 (CLOSED_LOOP_CONTROL)")
                # Try anyway - state feedback might be delayed
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to arm motors: {e}")
            return False
    
    def disarm_motors(self):
        """Disarm both motors into IDLE state."""
        self.get_logger().info("Disarming motors (IDLE)...")
        
        try:
            req_idle = AxisState.Request()
            req_idle.axis_requested_state = 1  # IDLE
            
            # Send requests asynchronously
            future_left = self.left_axis_client.call_async(req_idle)
            future_right = self.right_axis_client.call_async(req_idle)
            
            # Wait for completion
            timeout = 2.0
            start = time.time()
            while time.time() - start < timeout:
                if future_left.done() and future_right.done():
                    break
                time.sleep(0.1)
            
            self.get_logger().info("  Motors disarmed")
            
        except Exception as e:
            self.get_logger().warn(f"Error disarming motors: {e}")
    
    def _shutdown_odrive_nodes(self):
        """Shutdown the ODrive node subprocesses."""
        self.get_logger().info("Shutting down ODrive nodes...")
        
        for proc in self.odrive_processes:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f"Error shutting down ODrive node: {e}")
                try:
                    proc.kill()
                except Exception:
                    pass
        
        self.odrive_processes.clear()
        self.get_logger().info("  ODrive nodes shut down")
    
    def setup(self) -> bool:
        """Initialize ODrive nodes and arm motors."""
        # Launch ODrive nodes
        if self.launch_odrive_nodes:
            if not self._start_odrive_nodes():
                return False
            time.sleep(2.0)  # Wait for nodes to initialize
        
        # Wait for services
        if not self.wait_for_services():
            return False
        
        # Wait for feedback
        if not self.wait_for_feedback():
            return False
        
        # Arm motors
        if not self.arm_motors():
            return False
        
        time.sleep(0.5)  # Allow motors to settle
        return True
    
    def cleanup(self):
        """Disarm motors and shutdown ODrive nodes."""
        # Stop motors
        self.send_velocity('both', 0.0)
        time.sleep(0.2)
        
        # Disarm
        self.disarm_motors()
        time.sleep(0.5)
        
        # Shutdown ODrive nodes
        if self.launch_odrive_nodes:
            self._shutdown_odrive_nodes()
    
    def send_velocity(self, wheel: str, velocity: float):
        """Send velocity command to specified wheel(s)."""
        msg = ControlMessage()
        msg.control_mode = 2  # VELOCITY_CONTROL
        msg.input_mode = self.input_mode
        msg.input_vel = float(velocity)
        msg.input_torque = 0.0
        msg.input_pos = 0.0
        
        if wheel in ['left', 'both']:
            self.left_pub.publish(msg)
        if wheel in ['right', 'both']:
            # Right wheel might need inversion - adjust if needed
            self.right_pub.publish(msg)
    
    def run_step_test(self, wheel: str, start_vel: float, end_vel: float) -> StepResponseData:
        """
        Run a single step response test.
        
        Args:
            wheel: 'left', 'right', or 'both'
            start_vel: Initial velocity (rev/s)
            end_vel: Target velocity (rev/s)
            
        Returns:
            StepResponseData with recorded response
        """
        direction = "up" if end_vel > start_vel else "down"
        step_mag = abs(end_vel - start_vel)
        
        self.get_logger().info(f"  Step test: {wheel} wheel, {start_vel:.2f} → {end_vel:.2f} rev/s")
        
        # Initialize at start velocity
        self.send_velocity(wheel, start_vel)
        time.sleep(self.settle_time)
        
        # Prepare recording
        self.current_data = StepResponseData(
            wheel=wheel,
            step_magnitude=step_mag,
            direction=direction,
        )
        
        # Record baseline data (absolute timestamps stored by callbacks)
        self.recording = True
        time.sleep(0.15)  # Record 150ms of baseline
        
        # Apply step command and record the actual step time
        step_time = time.time()
        self.send_velocity(wheel, end_vel)
        self.current_data.command_time = step_time
        
        # Record response
        time.sleep(self.hold_time)
        
        self.recording = False
        
        # Convert absolute timestamps to relative (step happens at t=0)
        # Baseline data will have negative timestamps, response data positive
        self.current_data.timestamps = [
            t - step_time for t in self.current_data.timestamps
        ]
        
        return self.current_data
    
    def analyze_response(self, data: StepResponseData) -> Optional[DelayAnalysisResult]:
        """
        Analyze step response data to extract delay parameters.
        
        Args:
            data: StepResponseData from a step test
            
        Returns:
            DelayAnalysisResult with extracted parameters
        """
        if len(data.timestamps) < 10:
            self.get_logger().warn(f"Not enough data points for analysis: {len(data.timestamps)}")
            return None
        
        t = np.array(data.timestamps)
        v = np.array(data.velocities)
        
        # Debug: print data statistics
        n_baseline = np.sum(t < 0)
        n_response = np.sum(t >= 0)
        sample_rate = len(t) / (t[-1] - t[0]) if (t[-1] - t[0]) > 0 else 0
        self.get_logger().debug(
            f"    Data: {len(t)} samples, {n_baseline} baseline, {n_response} response, "
            f"~{sample_rate:.1f} Hz, t=[{t[0]*1000:.1f}, {t[-1]*1000:.1f}] ms"
        )
        
        # Find baseline (average of first few samples before step)
        # Step happens at t=0, so samples with t < 0 are baseline
        baseline_mask = t < 0
        if np.sum(baseline_mask) > 0:
            v_baseline = np.mean(v[baseline_mask])
        else:
            # Use first few samples
            v_baseline = np.mean(v[:5]) if len(v) > 5 else v[0]
        
        # Find steady state (average of last portion)
        v_steady = np.mean(v[-20:]) if len(v) > 20 else np.mean(v[-5:])
        
        # Step size (actual achieved)
        step_size = v_steady - v_baseline
        
        self.get_logger().debug(
            f"    Baseline: {v_baseline:.4f}, Steady: {v_steady:.4f}, Step: {step_size:.4f} rev/s"
        )
        
        if abs(step_size) < 0.01:
            self.get_logger().warn(f"Step size too small for analysis: {step_size:.4f}")
            return None
        
        # Find transport delay: time until velocity starts changing
        # Define "starts changing" as exceeding 5% of step
        threshold_start = v_baseline + 0.05 * step_size
        
        # Only look at data after t=0 (after step command)
        post_step_mask = t >= 0
        t_post = t[post_step_mask]
        v_post = v[post_step_mask]
        
        if len(t_post) < 5:
            self.get_logger().warn("Not enough post-step data")
            return None
        
        # Find first crossing of 5% threshold
        if step_size > 0:
            crossing_indices = np.where(v_post > threshold_start)[0]
        else:
            crossing_indices = np.where(v_post < threshold_start)[0]
        
        if len(crossing_indices) > 0:
            transport_delay = t_post[crossing_indices[0]]
        else:
            transport_delay = 0.0
        
        # Find time constant: time to reach 63.2% of step
        threshold_63 = v_baseline + 0.632 * step_size
        if step_size > 0:
            crossing_63 = np.where(v_post > threshold_63)[0]
        else:
            crossing_63 = np.where(v_post < threshold_63)[0]
        
        if len(crossing_63) > 0:
            time_constant = t_post[crossing_63[0]] - transport_delay
        else:
            time_constant = self.hold_time  # Didn't reach 63.2%
        
        # Find settling time: time to reach 95% of step
        threshold_95 = v_baseline + 0.95 * step_size
        if step_size > 0:
            crossing_95 = np.where(v_post > threshold_95)[0]
        else:
            crossing_95 = np.where(v_post < threshold_95)[0]
        
        if len(crossing_95) > 0:
            settling_time = t_post[crossing_95[0]]
        else:
            settling_time = self.hold_time  # Didn't reach 95%
        
        # Compute effective ramp rate (average velocity change rate during rise)
        # Look at data between 10% and 90% of step
        threshold_10 = v_baseline + 0.10 * step_size
        threshold_90 = v_baseline + 0.90 * step_size
        
        if step_size > 0:
            ramp_mask = (v_post > threshold_10) & (v_post < threshold_90)
        else:
            ramp_mask = (v_post < threshold_10) & (v_post > threshold_90)
        
        if np.sum(ramp_mask) >= 2:
            t_ramp = t_post[ramp_mask]
            v_ramp = v_post[ramp_mask]
            # Linear fit to get slope
            if len(t_ramp) > 1:
                slope = (v_ramp[-1] - v_ramp[0]) / (t_ramp[-1] - t_ramp[0])
                effective_ramp_rate = abs(slope)
            else:
                effective_ramp_rate = abs(step_size) / max(settling_time, 0.001)
        else:
            effective_ramp_rate = abs(step_size) / max(settling_time, 0.001)
        
        # Steady state error
        commanded_vel = v_baseline + data.step_magnitude * (1 if data.direction == "up" else -1)
        steady_state_error = v_steady - commanded_vel
        
        return DelayAnalysisResult(
            transport_delay=max(0, transport_delay),
            time_constant=max(0, time_constant),
            settling_time=max(0, settling_time),
            effective_ramp_rate=effective_ramp_rate,
            steady_state_error=steady_state_error
        )
    
    def save_results(self, data: StepResponseData, result: DelayAnalysisResult, trial: int):
        """Save raw data to CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.output_dir}/delay_test_{data.wheel}_{data.direction}_{trial}_{timestamp}.csv"
        
        with open(filename, 'w') as f:
            f.write("# Actuation Delay Measurement\n")
            f.write(f"# Wheel: {data.wheel}\n")
            f.write(f"# Direction: {data.direction}\n")
            f.write(f"# Step magnitude: {data.step_magnitude} rev/s\n")
            f.write(f"# Input mode: {'VEL_RAMP' if self.input_mode == 2 else 'PASSTHROUGH'}\n")
            f.write(f"# Transport delay: {result.transport_delay*1000:.2f} ms\n")
            f.write(f"# Time constant: {result.time_constant*1000:.2f} ms\n")
            f.write(f"# Settling time: {result.settling_time*1000:.2f} ms\n")
            f.write(f"# Effective ramp rate: {result.effective_ramp_rate:.2f} rev/s²\n")
            f.write("time_s,velocity_revps\n")
            for t, v in zip(data.timestamps, data.velocities):
                f.write(f"{t:.6f},{v:.6f}\n")
        
        self.get_logger().info(f"    Saved: {filename}")
    
    def run_all_tests(self):
        """Run complete test sequence."""
        self.get_logger().info("\nStarting test sequence...")
        self.get_logger().info("Make sure wheels are free to spin!\n")
        
        time.sleep(2.0)  # Give time to read warning
        
        wheels_to_test = ['left', 'right'] if self.test_wheel == 'both' else [self.test_wheel]
        
        for wheel in wheels_to_test:
            self.get_logger().info(f"\n{'='*40}")
            self.get_logger().info(f"Testing {wheel.upper()} wheel")
            self.get_logger().info(f"{'='*40}")
            
            for trial in range(self.num_trials):
                self.get_logger().info(f"\nTrial {trial + 1}/{self.num_trials}")
                
                # Step UP: 0 → step_velocity
                data_up = self.run_step_test(wheel, 0.0, self.step_velocity)
                result_up = self.analyze_response(data_up)
                if result_up:
                    self.all_results.append((data_up, result_up))
                    self.save_results(data_up, result_up, trial)
                    self.get_logger().info(f"    ↑ Delay: {result_up.transport_delay*1000:.1f}ms, "
                                          f"τ: {result_up.time_constant*1000:.1f}ms, "
                                          f"Ramp: {result_up.effective_ramp_rate:.1f} rev/s²")
                
                time.sleep(0.5)
                
                # Step DOWN: step_velocity → 0
                data_down = self.run_step_test(wheel, self.step_velocity, 0.0)
                result_down = self.analyze_response(data_down)
                if result_down:
                    self.all_results.append((data_down, result_down))
                    self.save_results(data_down, result_down, trial)
                    self.get_logger().info(f"    ↓ Delay: {result_down.transport_delay*1000:.1f}ms, "
                                          f"τ: {result_down.time_constant*1000:.1f}ms, "
                                          f"Ramp: {result_down.effective_ramp_rate:.1f} rev/s²")
                
                time.sleep(0.5)
        
        # Stop motors
        self.send_velocity('both', 0.0)
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print summary of all test results."""
        if not self.all_results:
            self.get_logger().warn("No valid results to summarize")
            return
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  SUMMARY")
        self.get_logger().info("="*60)
        
        # Aggregate results
        delays = [r.transport_delay for _, r in self.all_results]
        time_constants = [r.time_constant for _, r in self.all_results]
        settling_times = [r.settling_time for _, r in self.all_results]
        ramp_rates = [r.effective_ramp_rate for _, r in self.all_results]
        
        self.get_logger().info(f"\nTransport Delay (dead time):")
        self.get_logger().info(f"  Mean:   {np.mean(delays)*1000:.2f} ms")
        self.get_logger().info(f"  Std:    {np.std(delays)*1000:.2f} ms")
        self.get_logger().info(f"  Min:    {np.min(delays)*1000:.2f} ms")
        self.get_logger().info(f"  Max:    {np.max(delays)*1000:.2f} ms")
        
        self.get_logger().info(f"\nTime Constant (τ):")
        self.get_logger().info(f"  Mean:   {np.mean(time_constants)*1000:.2f} ms")
        self.get_logger().info(f"  Std:    {np.std(time_constants)*1000:.2f} ms")
        
        self.get_logger().info(f"\nSettling Time (95%):")
        self.get_logger().info(f"  Mean:   {np.mean(settling_times)*1000:.2f} ms")
        self.get_logger().info(f"  Std:    {np.std(settling_times)*1000:.2f} ms")
        
        self.get_logger().info(f"\nEffective Ramp Rate:")
        self.get_logger().info(f"  Mean:   {np.mean(ramp_rates):.2f} rev/s²")
        self.get_logger().info(f"  Std:    {np.std(ramp_rates):.2f} rev/s²")
        
        # Recommendations for MPC parameters
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  RECOMMENDED MPC PARAMETERS")
        self.get_logger().info("="*60)
        
        recommended_delay = np.mean(delays) + np.std(delays)  # Conservative estimate
        recommended_ramp = np.mean(ramp_rates) * 0.9  # Slightly conservative
        
        self.get_logger().info(f"\nFor WheelRampCompensator:")
        self.get_logger().info(f"  wheel_delay_time: {recommended_delay:.4f}  # seconds")
        self.get_logger().info(f"  wheel_ramp_rate:  {recommended_ramp:.1f}   # rev/s²")
        
        self.get_logger().info("\nNote: These are based on measured response.")
        self.get_logger().info("If using VEL_RAMP mode, wheel_ramp_rate should match")
        self.get_logger().info("your ODrive's vel_ramp_rate configuration.")
        self.get_logger().info("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    node = ActuationDelayMeasurement()
    
    # Run tests in a separate thread to allow callbacks
    import threading
    test_complete = threading.Event()
    test_success = [False]  # Use list to allow modification in nested function
    
    def run_tests():
        import traceback
        try:
            # Setup: launch ODrive nodes and arm motors
            node.get_logger().info("\n" + "="*60)
            node.get_logger().info("  SETUP")
            node.get_logger().info("="*60)
            
            if not node.setup():
                node.get_logger().error("Setup failed! Check ODrive connections.")
                test_complete.set()
                return
            
            # Run the actual tests
            node.run_all_tests()
            test_success[0] = True
            
        except Exception as e:
            node.get_logger().error(f"Test error: {e}")
            node.get_logger().error(traceback.format_exc())
        finally:
            test_complete.set()
    
    test_thread = threading.Thread(target=run_tests, daemon=True)
    test_thread.start()
    
    try:
        # Spin until tests complete or user interrupts
        while not test_complete.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Keep spinning briefly to finish any pending operations
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info("\nInterrupted by user")
    finally:
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("  CLEANUP")
        node.get_logger().info("="*60)
        
        # Cleanup: disarm motors and shutdown ODrive nodes
        node.cleanup()
        
        node.destroy_node()
        rclpy.shutdown()
        
        if test_success[0]:
            print("\n✅ Test completed successfully!")
        else:
            print("\n❌ Test failed or was interrupted")


if __name__ == '__main__':
    main()
