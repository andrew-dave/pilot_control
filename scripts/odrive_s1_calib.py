#!/usr/bin/env python3
"""
ODrive S1 Configuration & Calibration Script
============================================
Hardware setup:
- ODrive S1 controller
- BLDC motor with integrated Hall effect sensors (for commutation)
- Absolute encoder on gear-reduced output shaft (for position feedback)

Configuration strategy:
- Hall sensors → commutation feedback (coarse, built into motor)
- Absolute encoder → load/position feedback (precise, on output shaft)
- Gear ratio accounts for reduction between motor shaft and load

Usage:
  python3 odrive_s1_calib.py
  # or with specific serial number:
  python3 odrive_s1_calib.py 336A35623536
"""

import sys
import time
import math
import odrive
import odrive.enums as od_enums
from odrive.enums import *
from odrive.utils import dump_errors

# Try to import disconnect exception
try:
    from odrive.libodrive import DeviceLostException
except Exception:
    class DeviceLostException(Exception): 
        pass

# ============================================================================
# USER CONFIGURATION PARAMETERS
# ============================================================================

# --- Power Supply Protection ---
BUS_OVERVOLTAGE = 30     # V - Set based on your power supply (e.g., 48V → 56V)
BUS_UNDERVOLTAGE = 10.5     # V - Minimum voltage before shutdown
#DC_MAX_POSITIVE_CURRENT = 40.0  # A - Maximum battery/PSU charge current
#DC_MAX_NEGATIVE_CURRENT = -5.0  # A - Regen braking current limit

# --- Motor Specifications ---
# TODO: Measure/lookup from motor datasheet
MOTOR_TYPE = MotorType.HIGH_CURRENT  # or MotorType.GIMBAL for low-current motors
POLE_PAIRS = 2             # Count magnets, divide by 2 (e.g., 14 magnets = 7 pole pairs)
MOTOR_KV = 350             # RPM/V (from datasheet)
#MOTOR_RESISTANCE = 0.5      # Ohms (measure with multimeter or let ODrive calibrate)
#MOTOR_INDUCTANCE = 0.0005   # Henries (typically 0.0001 to 0.001 for BLDC)

# Calculate torque constant from Kv: Kt (N·m/A) ≈ 8.3 / Kv
TORQUE_CONSTANT = 8.3 / MOTOR_KV if MOTOR_KV > 0 else 0.05  # N·m/A

# --- Current Limits ---
CALIBRATION_CURRENT = 3.0   # A - Current for motor R/L calibration (safe moderate value)
CURRENT_LIMIT = 6.0        # A - Continuous current limit (check motor rating!)
#CURRENT_SOFT_MAX = 20.0     # A - Soft limit before torque limiting
#CURRENT_HARD_MAX = 30.0     # A - Absolute maximum (brief peaks only)

# --- Voltage Limits ---
#VOLTAGE_LIMIT = 24.0        # V - Motor voltage limit (for speed control)
CALIBRATION_VOLTAGE = 2.0   # V - Voltage limit during calibration

# --- Current Controller Bandwidth ---
#CURRENT_CONTROL_BANDWIDTH = 1000.0  # Hz - Higher = more responsive, lower = smoother

# --- Hall Sensor Configuration ---
# Hall sensors are typically 6-state (60° electrical spacing)
HALL_POLARITY = 0           # 0 or 6 (try 0 first, switch if direction is wrong)
HALL_POLARITY_CALIBRATED = False  # Set to True after first successful calibration

# --- Absolute Encoder Configuration ---
# AMT212B-OD: CUI 12-bit absolute encoder on gear-reduced output shaft
ENCODER_TYPE = "SPI"        # SPI interface
ENCODER_CPR = 16384          # 12-bit = 4096 counts per revolution
ENCODER_MODE = ENCODER_MODE_SPI_ABS_CUI  # CUI AMT212B-OD uses CUI protocol
                                          
# --- Gear Ratio Configuration ---
# IMPORTANT: This is the mechanical reduction between encoder and motor
# Encoder (AMT212B-OD) is on GEAR-REDUCED OUTPUT shaft
# TODO: Measure your actual gear ratio (e.g., motor turns X times for 1 encoder turn)
GEAR_RATIO = 10.0           # Motor shaft rotations per encoder rotation (UPDATE THIS!)

# --- Velocity Controller ---
VEL_LIMIT = 10.0            # turn/s - Maximum velocity (motor shaft speed)
VEL_LIMIT_TOLERANCE = 1.2   # Tolerance factor for velocity limit
VEL_RAMP_RATE = 10.0        # turn/s² - Acceleration/deceleration rate
VEL_GAIN = 0.16             # Velocity controller proportional gain (will auto-tune)
VEL_INTEGRATOR_GAIN = 0.32  # Velocity controller integral gain (will auto-tune)

# --- Position Controller ---
POS_GAIN = 20.0             # Position controller proportional gain (start conservative)

# --- Trap Trajectory ---
TRAP_TRAJ_ACCEL_LIMIT = 10.0  # turn/s² - Trapezoidal trajectory acceleration

# --- Torque Limits ---
TORQUE_SOFT_MIN = float('-inf')  # N·m - No lower torque limit
TORQUE_SOFT_MAX = float('inf')   # N·m - No upper torque limit

# --- Control Mode Defaults ---
DEFAULT_CONTROL_MODE = ControlMode.VELOCITY_CONTROL  # or POSITION_CONTROL
DEFAULT_INPUT_MODE = InputMode.VEL_RAMP              # VEL_RAMP, PASSTHROUGH, TRAP_TRAJ

# --- Startup Behavior ---
STARTUP_MOTOR_CALIBRATION = False   # Auto-calibrate motor on boot (after first setup)
STARTUP_ENCODER_OFFSET_CAL = False  # Auto-calibrate encoder offset on boot
STARTUP_CLOSED_LOOP = False         # Auto-enter closed loop on boot

# --- CAN Configuration (Optional) ---
CAN_NODE_ID = 1             # CAN node ID (0-63, set to unique value per ODrive)
CAN_HEARTBEAT_MS = 100      # Heartbeat message rate (ms)
CAN_FEEDBACK_MS = 10        # Feedback message rate (ms)
CAN_BAUD_RATE = 250000      # CAN bitrate (250000, 500000, 1000000)
ENABLE_WATCHDOG = False     # Disable watchdog
ENABLE_UART_A = False       # Disable UART A

# --- Encoder Bandwidth ---
ENCODER_BANDWIDTH = 100     # Hz - Encoder filter bandwidth

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def wait_for_state(axis, target_state, timeout=60.0):
    """Wait for axis to reach target state"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if axis.current_state == target_state:
            return True
        time.sleep(0.1)
    return False

def wait_for_idle(axis, timeout=90.0):
    """Wait for axis to return to IDLE state"""
    return wait_for_state(axis, AxisState.IDLE, timeout)

def safe_save_and_reboot(odrv, serial_number=None, pre_delay=0.5, post_delay=5.0):
    """Robust save and reboot that handles USB disconnection"""
    print("💾 Saving configuration...")
    try:
        odrv.save_configuration()
        time.sleep(pre_delay)
        print("🔄 Rebooting ODrive (USB may disconnect briefly)...")
        try:
            odrv.reboot()
        except Exception:
            pass  # Expected on some systems
    except DeviceLostException:
        pass  # Expected during reboot
    except Exception as e:
        print(f"⚠️  Save/reboot exception: {e}")
    
    time.sleep(post_delay)
    print("🔍 Reconnecting...")
    if serial_number:
        return odrive.find_any(serial_number=serial_number)
    else:
        return odrive.find_any()

def configure_power_limits(odrv):
    """Configure bus voltage and current limits"""
    print("\n--- Configuring Power Limits ---")
    odrv.config.dc_bus_overvoltage_trip_level = BUS_OVERVOLTAGE
    odrv.config.dc_bus_undervoltage_trip_level = BUS_UNDERVOLTAGE
    #odrv.config.dc_max_positive_current = DC_MAX_POSITIVE_CURRENT
    #odrv.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
    print(f"✓ Bus limits: {BUS_UNDERVOLTAGE}V - {BUS_OVERVOLTAGE}V")
    #print(f"✓ Current limits: {DC_MAX_NEGATIVE_CURRENT}A to +{DC_MAX_POSITIVE_CURRENT}A")

def configure_motor(axis):
    """Configure motor parameters"""
    print("\n--- Configuring Motor ---")
    
    # Motor type and pole pairs
    axis.motor.config.motor_type = MOTOR_TYPE
    axis.motor.config.pole_pairs = POLE_PAIRS
    
    # Torque constant
    axis.motor.config.torque_constant = TORQUE_CONSTANT
    
    # Current limits
    axis.motor.config.current_lim = CURRENT_LIMIT
    axis.motor.config.current_lim_margin = CURRENT_LIMIT * 1.25
    axis.motor.config.calibration_current = CALIBRATION_CURRENT
    
    # Voltage limits
    #if hasattr(axis.motor.config, 'voltage_limit'):
    #    axis.motor.config.voltage_limit = VOLTAGE_LIMIT
    if hasattr(axis.motor.config, 'resistance_calib_max_voltage'):
        axis.motor.config.resistance_calib_max_voltage = CALIBRATION_VOLTAGE
    
    # Current control bandwidth
    #if hasattr(axis.motor.config, 'current_control_bandwidth'):
    #    axis.motor.config.current_control_bandwidth = CURRENT_CONTROL_BANDWIDTH
    
    # Resistance and inductance (let ODrive measure these)
    # These will be calibrated automatically
    
    print(f"✓ Motor type: {MOTOR_TYPE}")
    print(f"✓ Pole pairs: {POLE_PAIRS}")
    print(f"✓ Torque constant: {TORQUE_CONSTANT:.6f} N·m/A")
    print(f"✓ Current limits: soft={CURRENT_SOFT_MAX}A, hard={CURRENT_HARD_MAX}A")
    print(f"✓ Calibration current: {CALIBRATION_CURRENT}A, lockin: {LOCKIN_CURRENT}A")
    print(f"✓ Current control BW: {CURRENT_CONTROL_BANDWIDTH} Hz")
    print(f"✓ Motor thermistor: {MOTOR_THERMISTOR_ENABLED}")

def configure_hall_sensor(axis):
    """Configure hall effect sensor for commutation"""
    print("\n--- Configuring Hall Sensors (Commutation) ---")
    
    # Find hall sensor object
    hall = None
    if hasattr(axis, 'hall_encoder'):
        hall = axis.hall_encoder
    elif hasattr(axis.config, 'hall_encoder'):
        hall = axis.config.hall_encoder
    
    if hall is None:
        print("⚠️  Warning: Hall encoder object not found on this firmware")
        return
    
    # Configure hall sensor
    hall.config.enabled = True
    
    # Polarity (direction)
    if hasattr(hall.config, 'hall_polarity'):
        hall.config.hall_polarity = HALL_POLARITY
    
    # Set as commutation encoder
    axis.config.commutation_encoder = EncoderId.HALL_ENCODER0
    
    print(f"✓ Hall sensor enabled (polarity: {HALL_POLARITY})")
    print(f"✓ Set as commutation encoder")

def configure_absolute_encoder(odrv, axis):
    """Configure AMT212B-OD absolute SPI encoder for position feedback"""
    print("\n--- Configuring Absolute Encoder (AMT212B-OD on Output Shaft) ---")
    
    # Find the encoder object (S1 uses 'spi_encoder0' or 'axis0.encoder')
    encoder = None
    encoder_id = None
    
    # Try different encoder object names (S1-specific)
    candidates = [
        ('spi_encoder0', EncoderId.SPI_ENCODER0),
        ('axis0.encoder', None),  # Sometimes encoder is under axis
    ]
    
    for obj_path, enc_id in candidates:
        try:
            if '.' in obj_path:
                parts = obj_path.split('.')
                obj = getattr(odrv, parts[0])
                obj = getattr(obj, parts[1])
            else:
                obj = getattr(odrv, obj_path)
            
            if obj is not None:
                encoder = obj
                encoder_id = enc_id
                print(f"✓ Found encoder: {obj_path}")
                break
        except AttributeError:
            continue
    
    if encoder is None:
        print("⚠️  Warning: Could not find encoder object")
        return
    
    # Configure encoder mode (AMT212B-OD is CUI SPI absolute)
    if hasattr(encoder.config, 'mode'):
        encoder.config.mode = ENCODER_MODE
        print(f"✓ Encoder mode: {ENCODER_MODE}")
    
    # Configure CPR (AMT212B-OD is 12-bit = 4096)
    if hasattr(encoder.config, 'cpr'):
        encoder.config.cpr = ENCODER_CPR
        print(f"✓ Encoder CPR: {ENCODER_CPR} (12-bit)")
    
    # Configure bandwidth
    if hasattr(encoder.config, 'bandwidth'):
        encoder.config.bandwidth = 1000
    
    # Disable index pulse (absolute encoders don't need it)
    if hasattr(encoder.config, 'use_index'):
        encoder.config.use_index = False
    
    # Configure gear ratio (IMPORTANT: encoder on OUTPUT shaft)
    if hasattr(encoder.config, 'gear_ratio'):
        encoder.config.gear_ratio = GEAR_RATIO
        print(f"✓ Gear ratio: {GEAR_RATIO}:1 (encoder on OUTPUT shaft)")
    else:
        print(f"⚠️  Warning: gear_ratio not available in encoder config")
        print(f"   📝 NOTE: You MUST set gear ratio = {GEAR_RATIO} manually!")
    
    # Set as load encoder (position feedback) - SPI_ENCODER0 for S1
    axis.config.load_encoder = EncoderId.SPI_ENCODER0
    print(f"✓ Set as load encoder (position feedback): SPI_ENCODER0")

def configure_controller(axis):
    """Configure velocity and position controllers"""
    print("\n--- Configuring Controller ---")
    
    # Control and input modes
    axis.controller.config.control_mode = DEFAULT_CONTROL_MODE
    axis.controller.config.input_mode = DEFAULT_INPUT_MODE
    
    # Velocity limits
    axis.controller.config.vel_limit = VEL_LIMIT
    axis.controller.config.vel_limit_tolerance = VEL_LIMIT_TOLERANCE
    
    # Velocity ramp
    if hasattr(axis.controller.config, 'vel_ramp_rate'):
        axis.controller.config.vel_ramp_rate = VEL_RAMP_RATE
    
    # Trap trajectory
    if hasattr(axis, 'trap_traj') and hasattr(axis.trap_traj, 'config'):
        axis.trap_traj.config.accel_limit = TRAP_TRAJ_ACCEL_LIMIT
    
    # Velocity gains
    if hasattr(axis.controller.config, 'vel_gain'):
        axis.controller.config.vel_gain = VEL_GAIN
    if hasattr(axis.controller.config, 'vel_integrator_gain'):
        axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    
    # Position gain
    if hasattr(axis.controller.config, 'pos_gain'):
        axis.controller.config.pos_gain = POS_GAIN
    
    # Torque limits
    if hasattr(axis.config, 'torque_soft_min'):
        axis.config.torque_soft_min = TORQUE_SOFT_MIN
    if hasattr(axis.config, 'torque_soft_max'):
        axis.config.torque_soft_max = TORQUE_SOFT_MAX
    
    # Circular setpoints (optional, for continuous rotation)
    if hasattr(axis.controller.config, 'circular_setpoints'):
        axis.controller.config.circular_setpoints = False
    
    # Watchdog
    if hasattr(axis.config, 'enable_watchdog'):
        axis.config.enable_watchdog = ENABLE_WATCHDOG
    
    # Encoder bandwidth
    if hasattr(axis.config, 'encoder_bandwidth'):
        axis.config.encoder_bandwidth = ENCODER_BANDWIDTH
    
    print(f"✓ Control mode: {DEFAULT_CONTROL_MODE}")
    print(f"✓ Input mode: {DEFAULT_INPUT_MODE}")
    print(f"✓ Velocity limit: {VEL_LIMIT} turn/s (tolerance: {VEL_LIMIT_TOLERANCE})")
    print(f"✓ Velocity ramp: {VEL_RAMP_RATE} turn/s²")
    print(f"✓ Velocity gains: P={VEL_GAIN}, I={VEL_INTEGRATOR_GAIN}")
    print(f"✓ Position gain: P={POS_GAIN}")
    print(f"✓ Trap traj accel: {TRAP_TRAJ_ACCEL_LIMIT} turn/s²")
    print(f"✓ Watchdog: {ENABLE_WATCHDOG}, Encoder BW: {ENCODER_BANDWIDTH} Hz")

def configure_startup_behavior(axis):
    """Configure what happens when ODrive boots"""
    print("\n--- Configuring Startup Behavior ---")
    
    axis.config.startup_motor_calibration = STARTUP_MOTOR_CALIBRATION
    axis.config.startup_encoder_offset_calibration = STARTUP_ENCODER_OFFSET_CAL
    axis.config.startup_closed_loop_control = STARTUP_CLOSED_LOOP
    
    print(f"✓ Startup motor calibration: {STARTUP_MOTOR_CALIBRATION}")
    print(f"✓ Startup encoder calibration: {STARTUP_ENCODER_OFFSET_CAL}")
    print(f"✓ Startup closed loop: {STARTUP_CLOSED_LOOP}")

def configure_can(odrv):
    """Configure CAN bus (optional)"""
    print("\n--- Configuring CAN Bus ---")
    
    if not hasattr(odrv, 'can'):
        print("⚠️  CAN not available on this ODrive")
        return
    
    try:
        # Protocol (Simple CAN) - set first
        if hasattr(odrv.can.config, 'protocol'):
            try:
                odrv.can.config.protocol = od_enums.Protocol.SIMPLE
            except:
                pass
        
        # Baud rate
        if hasattr(odrv.can.config, 'baud_rate'):
            odrv.can.config.baud_rate = CAN_BAUD_RATE
        
        # Node ID (axis-level for S1)
        if hasattr(odrv, 'axis0') and hasattr(odrv.axis0.config, 'can'):
            if hasattr(odrv.axis0.config.can, 'node_id'):
                odrv.axis0.config.can.node_id = CAN_NODE_ID
        
        # Heartbeat and feedback rates (axis-level for S1)
        if hasattr(odrv, 'axis0') and hasattr(odrv.axis0.config, 'can'):
            can_cfg = odrv.axis0.config.can
            if hasattr(can_cfg, 'heartbeat_msg_rate_ms'):
                can_cfg.heartbeat_msg_rate_ms = CAN_HEARTBEAT_MS
            
            # Set all feedback message rates
            feedback_attrs = [
                'encoder_msg_rate_ms', 'iq_msg_rate_ms', 
                'torques_msg_rate_ms', 'error_msg_rate_ms',
                'temperature_msg_rate_ms', 'bus_voltage_msg_rate_ms'
            ]
            for attr in feedback_attrs:
                if hasattr(can_cfg, attr):
                    setattr(can_cfg, attr, CAN_FEEDBACK_MS)
        
        print(f"✓ CAN node ID: {CAN_NODE_ID}")
        print(f"✓ CAN baud rate: {CAN_BAUD_RATE}")
        print(f"✓ Heartbeat: {CAN_HEARTBEAT_MS}ms, Feedback: {CAN_FEEDBACK_MS}ms")
    
    except Exception as e:
        print(f"⚠️  CAN configuration partial or failed: {e}")

def run_motor_calibration(axis):
    """Run motor resistance and inductance calibration"""
    print("\n--- Running Motor Calibration ---")
    print("⚠️  Ensure motor shaft is FREE to spin!")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    print("🔄 Running motor calibration (R and L measurement)...")
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    
    if not wait_for_idle(axis, timeout=30):
        print("❌ Motor calibration timed out or failed!")
        dump_errors(axis, True)
        return False
    
    # Check if calibration succeeded
    if axis.motor.is_calibrated:
        print(f"✅ Motor calibration successful!")
        print(f"   Resistance: {axis.motor.config.phase_resistance:.4f} Ω")
        print(f"   Inductance: {axis.motor.config.phase_inductance*1e6:.2f} µH")
        
        # Mark as pre-calibrated to skip on future boots
        axis.motor.config.pre_calibrated = True
        return True
    else:
        print("❌ Motor calibration failed!")
        dump_errors(axis, True)
        return False

def run_hall_calibration(axis):
    """Run hall sensor polarity calibration"""
    print("\n--- Running Hall Sensor Calibration ---")
    
    if HALL_POLARITY_CALIBRATED:
        print("ℹ️  Hall polarity already calibrated (skipping)")
        return True
    
    print("⚠️  Ensure motor shaft is FREE to spin!")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    print("🔄 Running hall polarity calibration...")
    axis.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
    
    if not wait_for_idle(axis, timeout=30):
        print("❌ Hall calibration timed out or failed!")
        dump_errors(axis, True)
        return False
    
    print(f"✅ Hall sensor calibration successful!")
    if hasattr(axis.hall_encoder.config, 'hall_polarity'):
        detected_polarity = axis.hall_encoder.config.hall_polarity
        print(f"   Detected polarity: {detected_polarity}")
        print(f"   💡 Update HALL_POLARITY = {detected_polarity} in script for future runs")
    
    return True

def run_encoder_offset_calibration(axis):
    """Run encoder offset calibration"""
    print("\n--- Running Encoder Offset Calibration ---")
    print("⚠️  Ensure motor shaft is FREE to spin!")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    print("🔄 Running encoder offset calibration...")
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_idle(axis, timeout=60):
        print("❌ Encoder calibration timed out or failed!")
        dump_errors(axis, True)
        return False
    
    # Find encoder object to check calibration
    encoder = None
    if hasattr(axis, 'encoder'):
        encoder = axis.encoder
    elif hasattr(axis, 'spi_encoder0'):
        encoder = axis.spi_encoder0
    
    if encoder and hasattr(encoder, 'is_ready'):
        if encoder.is_ready:
            print(f"✅ Encoder offset calibration successful!")
            if hasattr(encoder.config, 'pre_calibrated'):
                encoder.config.pre_calibrated = True
            return True
    
    print("❌ Encoder calibration failed!")
    dump_errors(axis, True)
    return False

def test_velocity_control(axis):
    """Quick test of velocity control"""
    print("\n--- Testing Velocity Control ---")
    print("⚠️  Motor will spin at low speed for 3 seconds")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    try:
        print("🔄 Entering closed loop control...")
        axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.5)
        
        print("→ Spinning at 1.0 turn/s...")
        axis.controller.input_vel = 1.0
        time.sleep(3.0)
        
        print("→ Stopping...")
        axis.controller.input_vel = 0.0
        time.sleep(1.0)
        
        print("→ Returning to idle...")
        axis.requested_state = AxisState.IDLE
        
        print("✅ Velocity test complete!")
        return True
    
    except Exception as e:
        print(f"❌ Velocity test failed: {e}")
        axis.requested_state = AxisState.IDLE
        dump_errors(axis, True)
        return False

# ============================================================================
# MAIN CALIBRATION SEQUENCE
# ============================================================================

def main():
    print("=" * 70)
    print("ODrive S1 Configuration & Calibration")
    print("=" * 70)
    
    # Get serial number from command line if provided
    serial_number = sys.argv[1] if len(sys.argv) > 1 else None
    
    # Connect to ODrive
    print("\n🔍 Searching for ODrive...")
    if serial_number:
        odrv = odrive.find_any(serial_number=serial_number)
    else:
        odrv = odrive.find_any()
    
    print(f"✅ Connected to ODrive S1")
    print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"   Serial: {hex(odrv.serial_number)}")
    print(f"   Bus voltage: {odrv.vbus_voltage:.2f} V")
    
    # Clear any existing errors
    odrv.clear_errors()
    
    axis = odrv.axis0
    
    # Configuration sequence
    configure_power_limits(odrv)
    configure_motor(axis)
    configure_hall_sensor(axis)
    configure_absolute_encoder(odrv, axis)
    configure_controller(axis)
    configure_startup_behavior(axis)
    configure_can(odrv)
    
    # Save configuration and reboot
    odrv = safe_save_and_reboot(odrv, serial_number)
    axis = odrv.axis0
    
    # Clear errors after reboot
    odrv.clear_errors()
    
    # Calibration sequence
    print("\n" + "=" * 70)
    print("CALIBRATION SEQUENCE")
    print("=" * 70)
    
    # 1. Motor calibration (R and L)
    if not run_motor_calibration(axis):
        print("\n❌ Calibration failed at motor stage")
        sys.exit(1)
    
    # 2. Hall sensor calibration
    if not run_hall_calibration(axis):
        print("\n⚠️  Hall calibration failed, but continuing...")
    
    # 3. Encoder offset calibration
    if not run_encoder_offset_calibration(axis):
        print("\n❌ Calibration failed at encoder stage")
        sys.exit(1)
    
    # Save calibration results
    print("\n💾 Saving calibration results...")
    odrv = safe_save_and_reboot(odrv, serial_number)
    axis = odrv.axis0
    
    # Test velocity control
    test_velocity_control(axis)
    
    # Final status
    print("\n" + "=" * 70)
    print("CALIBRATION COMPLETE!")
    print("=" * 70)
    print("\n📊 Final Status:")
    dump_errors(odrv, False)
    
    print("\n✅ ODrive is ready for operation!")
    print("\n💡 Next steps:")
    print("   1. Test manually: axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL")
    print("   2. Set velocity: axis0.controller.input_vel = 2.0")
    print("   3. Set position: axis0.controller.input_pos = 10.0")
    print("   4. Use via CAN bus with your ROS nodes")
    print("\n📝 Important parameters to note:")
    print(f"   - Resistance: {axis.motor.config.phase_resistance:.4f} Ω")
    print(f"   - Inductance: {axis.motor.config.phase_inductance*1e6:.2f} µH")
    if hasattr(axis.hall_encoder.config, 'hall_polarity'):
        print(f"   - Hall polarity: {axis.hall_encoder.config.hall_polarity}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

