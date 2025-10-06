# GPR Position Control Test Script

## Overview
This script tests the GPR motor position control using POSITION_CONTROL mode with PASSTHROUGH input mode. It simulates the behavior of the `update_gpr_motor()` function from `diff_drive_controller.cpp` but allows you to manually set the travel distance parameter for testing.

## Build and Install
```bash
cd ~/pilot_ws
colcon build --packages-select pilot_control
source install/setup.bash
```

## Usage

### Method 1: Quick Test with Default Parameters
```bash
ros2 run pilot_control test_gpr_position.py
```

### Method 2: Set Initial Travel Distance
```bash
# Test with 1 meter travel distance
ros2 run pilot_control test_gpr_position.py --ros-args -p test_travel_distance_m:=1.0
```

### Method 3: Adjust Distance in Real-Time
While the script is running, you can change the travel distance dynamically:

```bash
# Move GPR to position corresponding to 0.5m travel
ros2 param set /gpr_position_tester test_travel_distance_m 0.5

# Move GPR to position corresponding to 1.0m travel
ros2 param set /gpr_position_tester test_travel_distance_m 1.0

# Move GPR to position corresponding to 2.5m travel
ros2 param set /gpr_position_tester test_travel_distance_m 2.5

# Return to zero position
ros2 param set /gpr_position_tester test_travel_distance_m 0.0
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `third_wheel_radius` | 0.03 | Radius of GPR virtual wheel (m) |
| `third_gear_ratio` | 1.0 | GPR gear ratio |
| `velocity_multiplier` | 1.0 | Velocity multiplier |
| `invert_third` | False | Invert motor direction |
| `test_travel_distance_m` | 0.0 | Simulated travel distance (m) |
| `auto_arm` | True | Automatically arm motor on startup |

## Position Calculation

The script uses the same formula as `diff_drive_controller.cpp`:

```
turns_target = (travel_distance / wheel_circumference) * gear_ratio * velocity_multiplier
```

Where:
- `wheel_circumference = 2 * π * wheel_radius`
- Default: `2 * π * 0.03 = 0.188 m`

### Example Calculations
With default parameters:
- **1 meter travel** = 1 / 0.188 = **5.31 turns** (1912°)
- **0.5 meter travel** = 0.5 / 0.188 = **2.65 turns** (956°)
- **2 meters travel** = 2 / 0.188 = **10.61 turns** (3820°)

## Manual Motor Control

### Arm Motor (CLOSED_LOOP)
```bash
ros2 service call /gpr/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
```

### Disarm Motor (IDLE)
```bash
ros2 service call /gpr/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
```

## Testing Workflow

1. **Start the CAN interface** (if not already running):
   ```bash
   sudo ip link set can0 up type can bitrate 250000
   ```

2. **Start ODrive CAN node for GPR** (if not already running):
   ```bash
   ros2 run odrive_can odrive_can_node --ros-args \
     -r __ns:=/gpr \
     -p node_id:=2 \
     -p interface:=can0
   ```

3. **Run the test script**:
   ```bash
   ros2 run pilot_control test_gpr_position.py
   ```

4. **Test different distances**:
   ```bash
   # Small movement (10cm)
   ros2 param set /gpr_position_tester test_travel_distance_m 0.1
   
   # Medium movement (50cm)
   ros2 param set /gpr_position_tester test_travel_distance_m 0.5
   
   # Large movement (2m)
   ros2 param set /gpr_position_tester test_travel_distance_m 2.0
   
   # Return to zero
   ros2 param set /gpr_position_tester test_travel_distance_m 0.0
   ```

5. **Monitor the output** - The script logs position commands every 2 seconds:
   ```
   [gpr_position_tester]: Travel: 1.000 m → Position: 5.305 turns (1909.8°)
   ```

## Troubleshooting

### Motor doesn't move
- Check if ODrive node is running: `ros2 node list | grep gpr`
- Check if motor is armed: `ros2 topic echo /gpr/controller_status`
- Verify CAN interface: `ip link show can0`
- Check for errors: `candump can0`

### Motor moves opposite direction
Set the invert parameter:
```bash
ros2 param set /gpr_position_tester invert_third True
```

### Position is incorrect
Check the wheel radius parameter matches your physical setup:
```bash
ros2 param set /gpr_position_tester third_wheel_radius 0.03
```

## Comparison with diff_drive_controller

This test script replicates the GPR position control logic from `diff_drive_controller.cpp` (lines 318-370), specifically:

- Uses **POSITION_CONTROL** mode (mode 3)
- Uses **PASSTHROUGH** input mode (mode 1) - as requested for testing
  - Note: The main controller uses POSE_FILTER (mode 5) for smoother motion
- Calculates position from travel distance using the same formula
- Publishes at 20 Hz (same as `gpr_timer_`)
- Applies inversion logic consistently

## Notes

- The script publishes commands continuously at 20 Hz to maintain position
- You can stop the script with Ctrl+C - the motor will hold last commanded position
- To change input mode to POSE_FILTER (smoother), modify line 103: `msg.input_mode = 5`

