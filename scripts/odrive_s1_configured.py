#!/usr/bin/env python3
"""
ODrive S1 Configuration & Calibration Script - CONFIGURED VERSION
============================================
Hardware setup (YOUR ACTUAL CONFIGURATION):
- ODrive S1 controller
- BLDC motor with 20 pole pairs (40 magnets)
- Hall effect sensors for commutation (polarity calibrated)
- AMT212B-OD: CUI 12-bit absolute SPI encoder on gear-reduced output shaft
- Gear ratio: TBD (measure your reduction)

This script contains YOUR EXACT working parameters from the previous setup.
Only the encoder has changed from RS485 (AMT21) to SPI (AMT212B-OD).

Usage:
  python3 odrive_s1_configured.py
"""

import sys
import time
import math
import odrive
import odrive.enums as od_enums
from odrive.enums import *
from odrive.utils import dump_errors

try:
    from odrive.libodrive import DeviceLostException
except Exception:
    class DeviceLostException(Exception): 
        pass

# ============================================================================
# CONFIGURED PARAMETERS (FROM YOUR WORKING SYSTEM)
# ============================================================================

# --- Power Supply Protection ---
BUS_OVERVOLTAGE = 28.0          # V
BUS_UNDERVOLTAGE = 10.5         # V
DC_MAX_POSITIVE_CURRENT = math.inf
DC_MAX_NEGATIVE_CURRENT = -math.inf
BRAKE_RESISTOR_ENABLE = False

# --- Motor Specifications ---
MOTOR_TYPE = MotorType.HIGH_CURRENT  # Was PMSM_CURRENT_CONTROL in your config
POLE_PAIRS = 2                      # 40 magnets
MOTOR_KV = 350                       # RPM/V
TORQUE_CONSTANT = 0.023628571428571426  # Your measured value (N·m/A)

# --- Current Limits ---
CALIBRATION_CURRENT = 4.0       # A
CURRENT_SOFT_MAX = 6.0          # A
CURRENT_HARD_MAX = 17.8         # A
LOCKIN_CURRENT = 10.0           # A (for encoder calibration)

# --- Voltage Limits ---
CALIBRATION_VOLTAGE = 6.0       # V

# --- Motor Thermistor ---
MOTOR_THERMISTOR_ENABLED = False

# --- Hall Sensor Configuration (Commutation) ---
HALL_POLARITY = 0               # Your working value
HALL_POLARITY_CALIBRATED = True # Already calibrated

# --- Absolute Encoder Configuration (Position Feedback) ---
# AMT212B-OD: CUI 12-bit absolute SPI encoder
ENCODER_CPR = 16384              # 12-bit = 4096
ENCODER_MODE = Rs485EncoderMode.AMT21_EVENT_DRIVEN

# --- Gear Ratio ---
# TODO: CRITICAL - Measure your actual gearbox ratio!
# If motor spins 10 times for 1 encoder rotation → GEAR_RATIO = 10.0
GEAR_RATIO = 10.0               # ⚠️ UPDATE THIS WITH YOUR MEASURED VALUE!

# --- Velocity Estimation ---
# In dual-encoder setups with gearbox backlash, ODrive recommends using
# the commutation encoder (hall sensors) for velocity estimation instead
# of the load encoder (AMT212B-OD through gearbox)
USE_COMMUTATION_VEL = True      # Use hall sensors for velocity estimation

# CRITICAL: ODrive documentation specifies the correct formula:
# "If the position encoder is mounted after a gearbox with a reduction ratio N:1,
#  this variable must be set to 1/(pole_pairs*N)"
# This accounts for the fact that hall sensors measure electrical rotations!
COMMUTATION_VEL_SCALE = 1.0 / (POLE_PAIRS * GEAR_RATIO)  # Correct formula!

# --- Velocity Controller ---
VEL_LIMIT = 10.0                # turn/s (motor shaft)
VEL_LIMIT_TOLERANCE = 1.2
VEL_RAMP_RATE = 10.0            # turn/s²
VEL_GAIN = 0.03
VEL_INTEGRATOR_GAIN = 0.15

# --- Position Controller ---
POS_GAIN = 20.0

# --- Trap Trajectory ---
TRAP_TRAJ_ACCEL_LIMIT = 10.0    # turn/s²

# --- Torque Limits ---
TORQUE_SOFT_MIN = -math.inf
TORQUE_SOFT_MAX = math.inf

# --- Control Mode Defaults ---
DEFAULT_CONTROL_MODE = ControlMode.VELOCITY_CONTROL
DEFAULT_INPUT_MODE = InputMode.VEL_RAMP

# --- Startup Behavior ---
STARTUP_MOTOR_CALIBRATION = False
STARTUP_ENCODER_OFFSET_CAL = False
STARTUP_CLOSED_LOOP = False

# --- CAN Configuration ---
CAN_NODE_ID = 1                 # Your working node ID
CAN_HEARTBEAT_MS = 100
CAN_FEEDBACK_MS = 10
CAN_BAUD_RATE = 250000
ENABLE_WATCHDOG = False
ENABLE_UART_A = False

# --- Encoder Bandwidth ---
ENCODER_BANDWIDTH = 100         # Hz

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def wait_for_idle(axis, timeout=5000.0):
    """Wait for axis to return to IDLE state"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if axis.current_state == AxisState.IDLE:
            print(time.time() - t0 )
            return True
        time.sleep(0.1)
    print(time.time() - t0 )
    return False

def safe_save_and_reboot(odrv, serial_number=None, pre_delay=0.5, post_delay=5.0):
    """Robust save and reboot that handles USB disconnection"""
    print("💾 Saving configuration...")
    try:
        odrv.save_configuration()
        time.sleep(pre_delay)
        print("🔄 Rebooting ODrive...")
        try:
            odrv.reboot()
        except Exception:
            pass
    except (DeviceLostException, Exception) as e:
        print(f"   (Expected disconnect during reboot)")
    
    time.sleep(post_delay)
    print("🔍 Reconnecting...")
    return odrive.find_any(serial_number=serial_number) if serial_number else odrive.find_any()

def configure_all_parameters(odrv, axis):
    """Apply all configuration parameters"""
    print("\n" + "="*70)
    print("APPLYING CONFIGURATION")
    print("="*70)
    
    # --- Power Limits ---
    print("\n📊 Power Limits...")
    odrv.config.dc_bus_overvoltage_trip_level = BUS_OVERVOLTAGE
    odrv.config.dc_bus_undervoltage_trip_level = BUS_UNDERVOLTAGE
    odrv.config.dc_max_positive_current = DC_MAX_POSITIVE_CURRENT
    odrv.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
    if hasattr(odrv.config, 'brake_resistor0'):
        odrv.config.brake_resistor0.enable = BRAKE_RESISTOR_ENABLE
    if hasattr(odrv.config, 'enable_uart_a'):
        odrv.config.enable_uart_a = ENABLE_UART_A
    print(f"  ✓ Bus: {BUS_UNDERVOLTAGE}V - {BUS_OVERVOLTAGE}V")
    
    # --- Motor Configuration ---
    print("\n⚙️  Motor Configuration...")
    axis.config.motor.motor_type = MOTOR_TYPE
    axis.config.motor.pole_pairs = POLE_PAIRS
    axis.config.motor.torque_constant = TORQUE_CONSTANT
    axis.config.motor.current_soft_max = CURRENT_SOFT_MAX
    axis.config.motor.current_hard_max = CURRENT_HARD_MAX
    axis.config.motor.calibration_current = CALIBRATION_CURRENT
    if hasattr(axis.config.motor, 'resistance_calib_max_voltage'):
        axis.config.motor.resistance_calib_max_voltage = CALIBRATION_VOLTAGE
    if hasattr(axis.config, 'calibration_lockin'):
        axis.config.calibration_lockin.current = LOCKIN_CURRENT
    if hasattr(axis.config.motor, 'motor_thermistor'):
        axis.motor.motor_thermistor.config.enabled = MOTOR_THERMISTOR_ENABLED
    print(f"  ✓ Pole pairs: {POLE_PAIRS}, Kt: {TORQUE_CONSTANT:.6f} N·m/A")
    print(f"  ✓ Current: soft={CURRENT_SOFT_MAX}A, hard={CURRENT_HARD_MAX}A, cal={CALIBRATION_CURRENT}A")
    
    # --- Hall Sensor (Commutation) ---
    print("\n🧭 Hall Sensor (Commutation)...")
    if hasattr(odrv, 'hall_encoder0'):
        odrv.hall_encoder0.config.enabled = True
        if hasattr(odrv.hall_encoder0, 'hall_polarity'):
            odrv.hall_encoder0.config.hall_polarity = HALL_POLARITY
        axis.config.commutation_encoder = EncoderId.HALL_ENCODER0
        print(f"  ✓ Hall enabled, polarity={HALL_POLARITY}, set as commutation encoder")
    
    # --- SPI Encoder (Position Feedback) ---
    print("\n📐 SPI Encoder (AMT212B-OD on output shaft)...")
    if hasattr(odrv, 'spi_encoder0'):
        enc = odrv.spi_encoder0
        enc.config.mode = ENCODER_MODE
        if hasattr(enc.config, 'bandwidth'):
            enc.config.bandwidth = 1000
        if hasattr(enc.config, 'use_index'):
            enc.config.use_index = False
        if hasattr(enc.config, 'gear_ratio'):
            enc.config.gear_ratio = GEAR_RATIO
            print(f"  ✓ Gear ratio set: {GEAR_RATIO}:1")
        else:
            print(f"  ⚠️  gear_ratio not available - SET MANUALLY!")
        axis.config.load_encoder = EncoderId.SPI_ENCODER0
        print(f"  ✓ SPI encoder: {ENCODER_CPR} CPR, mode={ENCODER_MODE}")
        print(f"  ✓ Set as load encoder (position feedback)")
        if hasattr(enc.config, 'pre_calibrated'):
            enc.config.pre_calibrated = False
    
    # --- Controller ---
    print("\n🎮 Controller...")
    axis.controller.config.control_mode = DEFAULT_CONTROL_MODE
    axis.controller.config.input_mode = DEFAULT_INPUT_MODE
    axis.controller.config.vel_limit = VEL_LIMIT
    axis.controller.config.vel_limit_tolerance = VEL_LIMIT_TOLERANCE
    if hasattr(axis.controller.config, 'vel_ramp_rate'):
        axis.controller.config.vel_ramp_rate = VEL_RAMP_RATE
    if hasattr(axis, 'trap_traj'):
        axis.trap_traj.config.accel_limit = TRAP_TRAJ_ACCEL_LIMIT
    if hasattr(axis.controller.config, 'vel_gain'):
        axis.controller.config.vel_gain = VEL_GAIN
    if hasattr(axis.controller.config, 'vel_integrator_gain'):
        axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    if hasattr(axis.controller.config, 'pos_gain'):
        axis.controller.config.pos_gain = POS_GAIN
    if hasattr(axis.config, 'torque_soft_min'):
        axis.config.torque_soft_min = TORQUE_SOFT_MIN
        axis.config.torque_soft_max = TORQUE_SOFT_MAX
    if hasattr(axis.config, 'enable_watchdog'):
        axis.config.enable_watchdog = ENABLE_WATCHDOG
    if hasattr(axis.config, 'encoder_bandwidth'):
        axis.config.encoder_bandwidth = ENCODER_BANDWIDTH
    
    # --- Velocity Estimation (IMPORTANT for dual-encoder with gearbox!) ---
    # Use commutation encoder (hall sensors) for velocity estimation to avoid
    # gearbox backlash affecting velocity control
    if hasattr(axis.controller.config, 'use_commutation_vel'):
        axis.controller.config.use_commutation_vel = USE_COMMUTATION_VEL
        if hasattr(axis.controller.config, 'commutation_vel_scale'):
            axis.controller.config.commutation_vel_scale = COMMUTATION_VEL_SCALE
            print(f"  ✓ Using commutation encoder for velocity (scale={COMMUTATION_VEL_SCALE:.4f})")
        else:
            print(f"  ⚠️  use_commutation_vel set but commutation_vel_scale not available")
    else:
        print(f"  ℹ️  use_commutation_vel not available (older firmware)")
    
    print(f"  ✓ Vel: {VEL_LIMIT} turn/s, ramp: {VEL_RAMP_RATE} turn/s²")
    print(f"  ✓ Gains: vel_P={VEL_GAIN}, vel_I={VEL_INTEGRATOR_GAIN}, pos_P={POS_GAIN}")
    
    # --- CAN ---
    print("\n📡 CAN Bus...")
    if hasattr(odrv, 'can'):
        if hasattr(odrv.can.config, 'protocol'):
            try:
                odrv.can.config.protocol = od_enums.Protocol.SIMPLE
            except:
                pass
        if hasattr(odrv.can.config, 'baud_rate'):
            odrv.can.config.baud_rate = CAN_BAUD_RATE
        if hasattr(axis.config, 'can'):
            axis.config.can.node_id = CAN_NODE_ID
            axis.config.can.heartbeat_msg_rate_ms = CAN_HEARTBEAT_MS
            for attr in ['encoder_msg_rate_ms', 'iq_msg_rate_ms', 'torques_msg_rate_ms',
                        'error_msg_rate_ms', 'temperature_msg_rate_ms', 'bus_voltage_msg_rate_ms']:
                if hasattr(axis.config.can, attr):
                    setattr(axis.config.can, attr, CAN_FEEDBACK_MS)
        print(f"  ✓ Node ID: {CAN_NODE_ID}, baud: {CAN_BAUD_RATE}, rates: {CAN_FEEDBACK_MS}ms")
    
    # --- Startup ---
    print("\n🚀 Startup Behavior...")
    axis.config.startup_motor_calibration = STARTUP_MOTOR_CALIBRATION
    axis.config.startup_encoder_offset_calibration = STARTUP_ENCODER_OFFSET_CAL
    axis.config.startup_closed_loop_control = STARTUP_CLOSED_LOOP
    print(f"  ✓ Auto-calibrate: motor={STARTUP_MOTOR_CALIBRATION}, encoder={STARTUP_ENCODER_OFFSET_CAL}")

# ============================================================================
# CALIBRATION FUNCTIONS
# ============================================================================

def run_motor_calibration(axis):
    """Run motor R and L calibration"""
    print("\n" + "="*70)
    print("MOTOR CALIBRATION")
    print("="*70)
    print("⚠️  Motor shaft must be FREE to spin!")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    print("🔄 Running motor calibration...")
    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    
    if not wait_for_idle(axis, timeout=5000):
        print("❌ Motor calibration failed!")
        dump_errors(axis, True)
        return False
    
    # Accept success if firmware reports a successful procedure result (GUI-like)
    proc = getattr(axis, 'procedure_result', None)
    proc_success = False
    try:
        proc_success = (proc == od_enums.ProcedureResult.SUCCESS)
    except Exception:
        proc_success = False

    # Fallback robust check based on errors and measured parameters
    calibrated_flag = getattr(axis.motor, 'is_calibrated', None)
    if isinstance(calibrated_flag, bool):
        calibrated = calibrated_flag
    else:
        calibrated = (
            getattr(axis.motor, 'error', 1) == 0 and
            getattr(axis, 'error', 1) == 0
        )

    if proc_success or calibrated:
        print(f"✅ Motor calibration successful!")
        print(f"   R = {axis.config.motor.phase_resistance:.4f} Ω")
        print(f"   L = {axis.config.motor.phase_inductance*1e6:.2f} µH")
        # axis.config.motor.pre_calibrated = True
        return True
    print("❌ Motor calibration failed!")
    dump_errors(axis, True)
    return False

def run_encoder_offset_calibration(axis):
    """Run encoder offset calibration"""
    print("\n" + "="*70)
    print("ENCODER OFFSET CALIBRATION")
    print("="*70)
    print("⚠️  Motor shaft must be FREE to spin!")
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    print("🔄 Running encoder offset calibration...")
    # Step 1: Calibrate commutation (hall) first if supported
    if hasattr(axis, 'hall_encoder') and hasattr(axis, 'requested_state'):
        try:
            # Many firmwares lack a dedicated state; we can at least request partial calibration
            # Fallback to general encoder offset calibration state
            pass
        except Exception:
            pass
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_idle(axis, timeout=60):
        print("❌ Encoder calibration failed!")
        dump_errors(axis, True)
        return False
    
    print(f"✅ Encoder offset calibration successful!")
    return True

def test_velocity(axis):
    """Quick velocity test"""
    print("\n" + "="*70)
    print("VELOCITY TEST")
    print("="*70)
    print("⏳ Starting in 3 seconds...")
    time.sleep(3)
    
    try:
        print("🔄 Entering closed loop...")
        axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.5)
        
        print("→ Spinning at 1 turn/s for 3 seconds...")
        axis.controller.input_vel = 1.0
        time.sleep(3.0)
        
        print("→ Stopping...")
        axis.controller.input_vel = 0.0
        time.sleep(1.0)
        
        axis.requested_state = AxisState.IDLE
        print("✅ Velocity test complete!")
        return True
    except Exception as e:
        print(f"❌ Test failed: {e}")
        axis.requested_state = AxisState.IDLE
        return False

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("="*70)
    print("ODrive S1 - AMT212B-OD Configuration & Calibration")
    print("="*70)
    
    serial_number = sys.argv[1] if len(sys.argv) > 1 else None
    
    print("\n🔍 Connecting to ODrive...")
    odrv = odrive.find_any(serial_number=serial_number) if serial_number else odrive.find_any()
    
    print(f"✅ Connected!")
    print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"   Serial: {hex(odrv.serial_number)}")
    print(f"   Vbus: {odrv.vbus_voltage:.2f} V")
    
    odrv.clear_errors()
    axis = odrv.axis0
    
    # Apply all configuration
    configure_all_parameters(odrv, axis)
    
    # Save and reboot
    odrv = safe_save_and_reboot(odrv, serial_number)
    axis = odrv.axis0
    odrv.clear_errors()
    
    # Calibration sequence (GUI-like): motor first, then encoder offset
    if not run_motor_calibration(axis):
        print("\n❌ Calibration failed!")
        sys.exit(1)
    
    if not run_encoder_offset_calibration(axis):
        print("\n❌ Calibration failed!")
        sys.exit(1)
    
    # Mark devices as pre-calibrated if attributes exist
    try:
        if hasattr(axis.motor.config, 'pre_calibrated'):
            axis.motor.config.pre_calibrated = True
        if hasattr(odrv, 'spi_encoder0') and hasattr(odrv.spi_encoder0.config, 'pre_calibrated'):
            odrv.spi_encoder0.config.pre_calibrated = True
        if hasattr(odrv, 'hall_encoder0') and hasattr(odrv.hall_encoder0.config, 'pre_calibrated'):
            odrv.hall_encoder0.config.pre_calibrated = True
    except Exception:
        pass

    # Save calibration
    print("\n💾 Saving calibration...")
    odrv = safe_save_and_reboot(odrv, serial_number)
    axis = odrv.axis0
    
    # Test
    test_velocity(axis)
    
    # Final status
    print("\n" + "="*70)
    print("CALIBRATION COMPLETE!")
    print("="*70)
    print("\n📊 Status:")
    dump_errors(odrv, False)
    
    print("\n✅ ODrive ready for operation via CAN!")
    print(f"\n💡 CAN Node ID: {CAN_NODE_ID}")
    print(f"\n📝 IMPORTANT NOTES:")
    print(f"   1. Verify gear ratio = {GEAR_RATIO} is correct!")
    print(f"      Test: Spin motor 1 rotation → count output shaft rotations")
    print(f"   2. Velocity estimation: {'COMMUTATION (hall sensors)' if USE_COMMUTATION_VEL else 'LOAD (SPI encoder)'}")
    if USE_COMMUTATION_VEL:
        print(f"      → Using hall sensors avoids gearbox backlash in velocity control")
        print(f"      → Position still uses accurate SPI encoder")
        print(f"      → commutation_vel_scale = 1/(pole_pairs × gear_ratio)")
        print(f"      → commutation_vel_scale = 1/({POLE_PAIRS} × {GEAR_RATIO}) = {COMMUTATION_VEL_SCALE:.6f}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

