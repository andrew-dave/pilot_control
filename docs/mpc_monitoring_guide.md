# MPC Autonomous Controller - Monitoring Guide

## Critical Parameters to Monitor

### 1. **MPC Optimization Performance**

#### Solve Time
- **Topic**: Monitor via logs or add diagnostic publisher
- **Target**: < 5ms per solve
- **Maximum**: < 50ms (hard limit)
- **Action**: If consistently > 50ms, reduce horizon or simplify problem

#### Solver Status
- **Check**: MPC solve return status
- **Expected**: `'solved'`
- **Action**: If status != 'solved', check constraints and reference trajectory

### 2. **Slip Estimation**

#### Slip Ratios
- **Topic**: `/mpc_control/slip_ratios` (Float64MultiArray)
- **Format**: `[left_slip, right_slip]`
- **Expected Range**: [-1.0, 1.0]
- **Normal Operation**: 
  - On flat ground: ~0.0 to 0.1 (small slip)
  - On loose terrain: 0.1 to 0.3 (moderate slip)
  - Extreme conditions: > 0.3 (high slip)
- **Action**: 
  - If consistently > 0.5: Check encoder calibration
  - If negative: Check wheel direction/inversion

#### Slip Estimation Window
- **Parameter**: `slip_estimation_window` (default: 1.0s)
- **Monitor**: Ensure enough data points in history
- **Action**: Increase if noisy, decrease if too slow to respond

### 3. **Error States**

#### Position Errors (Body Frame)
- **xe**: Forward error (m)
- **ye**: Lateral error (m)
- **θe**: Yaw error (rad)
- **Expected**: Should decrease over time
- **Action**: 
  - If xe, ye not decreasing: Check cost weights (Q_xe, Q_ye)
  - If oscillating: Reduce control frequency or adjust MPC parameters

### 4. **Control Outputs**

#### Wheel Velocities
- **Topics**: 
  - `/left/control_message` (ControlMessage)
  - `/right/control_message` (ControlMessage)
- **Units**: Motor rev/s
- **Expected**: Within limits based on max_linear_velocity
- **Action**: Check if saturating at limits

#### Command Twist (Diagnostics)
- **Topic**: `/mpc_control/cmd_twist` (Twist)
- **Contains**: Linear and angular velocity commands
- **Use**: Compare with actual odometry to verify tracking

### 5. **Reference Trajectory**

#### Waypoint Generation
- **Check**: Number of waypoints = mpc_horizon
- **Verify**: Waypoints are properly spaced
- **Action**: If waypoints not generated, check path initialization

#### Waypoint Spacing
- **Expected**: `max_linear_velocity * mpc_dt` (m)
- **Action**: Adjust if waypoints too close/far

### 6. **System Health**

#### Odometry Updates
- **Topic**: `/Odometry_tilt_corrected_diff`
- **Check**: Update frequency matches expected rate
- **Action**: If missing updates, check Fast-LIO and tilt corrector

#### Encoder Updates
- **Topics**: 
  - `/left/controller_status`
  - `/right/controller_status`
- **Check**: Both encoders updating at similar rates
- **Action**: If one missing, check ODrive node

#### Control Loop Frequency
- **Parameter**: `control_frequency` (default: 10.0 Hz)
- **Monitor**: Actual loop execution time
- **Action**: If slower than expected, check system load

## Recommended Monitoring Setup

### 1. **ROS 2 Topic Monitoring**

```bash
# Monitor slip ratios
ros2 topic echo /mpc_control/slip_ratios

# Monitor command twist
ros2 topic echo /mpc_control/cmd_twist

# Monitor control messages
ros2 topic echo /left/control_message
ros2 topic echo /right/control_message

# Monitor odometry
ros2 topic echo /Odometry_tilt_corrected_diff

# Monitor encoder status
ros2 topic echo /left/controller_status
ros2 topic echo /right/controller_status
```

### 2. **Topic Frequency Monitoring**

```bash
# Check control loop frequency
ros2 topic hz /mpc_control/cmd_twist

# Check odometry frequency
ros2 topic hz /Odometry_tilt_corrected_diff

# Check encoder frequencies
ros2 topic hz /left/controller_status
ros2 topic hz /right/controller_status
```

### 3. **Parameter Monitoring**

```bash
# List all MPC parameters
ros2 param list /mpc_autonomous_controller

# Get specific parameter
ros2 param get /mpc_autonomous_controller mpc_horizon
ros2 param get /mpc_autonomous_controller mpc_Q_xe
```

### 4. **Diagnostic Publishers (To Add)**

Consider adding these diagnostic topics:

1. **MPC Solve Time**: `Float64` - Solve time in milliseconds
2. **Error States**: `Float64MultiArray` - [xe, ye, θe]
3. **Reference Trajectory**: `PoseArray` - Visualize waypoints
4. **Solver Status**: `String` - OSQP solver status

## Tuning Guidelines

### If Robot Overshoots Target
- **Increase**: `mpc_Q_xe`, `mpc_Q_ye` (position error weights)
- **Decrease**: `max_linear_velocity`
- **Check**: Waypoint generation (should stop at target)

### If Robot Oscillates
- **Decrease**: `control_frequency`
- **Increase**: `mpc_dt` (larger time step)
- **Check**: Slip estimation (may be too noisy)

### If MPC Solve Too Slow
- **Decrease**: `mpc_horizon` (fewer prediction steps)
- **Check**: System CPU load
- **Verify**: OSQP solver settings

### If Slip Estimation Unreliable
- **Increase**: `slip_estimation_window` (longer averaging)
- **Check**: Encoder calibration
- **Verify**: Odometry quality

## Safety Checks

1. **Emergency Stop**: Monitor for NaN or Inf values in control outputs
2. **Constraint Violations**: Check if MPC solution violates bounds
3. **Communication Loss**: Monitor topic update rates
4. **Solver Failures**: Log and handle MPC solve failures gracefully

## Logging Recommendations

Log the following at each control step:
- MPC solve time
- Current error state [xe, ye, θe]
- Slip ratios [λ_L, λ_R]
- Control outputs [ωL, ωR]
- Solver status
- Reference trajectory (first waypoint)

