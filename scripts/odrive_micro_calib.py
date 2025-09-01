#!/usr/bin/env python3
"""
ODrive Micro + iPower GM2804 (12N/14P → 7 pp) @ 12 V
- Sets motor/encoder/controller params
- Full calibration
- Harmonic calibration (FW >= 0.6.11)
- Robust save() that handles the expected USB drop during flash write
Usage:
  python3 odrive_micro_calib.py
  # or with serial (hex without 0x):
  python3 odrive_micro_calib.py 335737803432
"""
import sys, time, math
import odrive
from odrive.enums import *
from odrive.utils import dump_errors

# Try to import the specific disconnect exception; fall back to generic
try:
    from odrive.libodrive import DeviceLostException
except Exception:
    class DeviceLostException(Exception): pass

# ---------- User constants ----------
BUS_OV = 16.0            # V  (regen headroom on 12 V bus)
BUS_UV = 9.2             # V
POLE_PAIRS = 7           # GM2804 = 12N/14P => 7
KT = 0.056               # N·m/A (Kv~170 RPM/V)
I_CAL = 0.5              # A   (R/L calibration)
V_CAL_MAX = 4.5          # V   (~0.5 A * 9 Ω)
I_LIM = 1.2              # A   (gentle)
BW_CURRENT = 300.0       # Hz  (softer current loop for small gimbals)

VEL_LIMIT = 12.0         # turn/s
VEL_RAMP  = 1.0          # turn/s^2
VEL_GAIN  = 0.02
VEL_IGAIN = 0.0

LOCKIN_I = 0.65          # A
TORQUE_SOFT_MIN = -0.1
TORQUE_SOFT_MAX  = 0.1

# Harmonic calibration (FW >= 0.6.11)
HC_VEL    = 5.0          # turn/s
HC_TURNS  = 4.0          # mech turns
HC_SETTLE = 0.2          # s

# Optional: run these at boot once you’re happy
STARTUP_MOTOR_CAL = True
STARTUP_ENC_CAL   = True
STARTUP_CLOSED_LOOP = False

def wait_idle(axis, timeout=90.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if axis.current_state == AxisState.IDLE:
            return True
        time.sleep(0.1)
    return False

def safe_save_and_reboot(odrv, serial=None, pre=0.4, post=5.0):
    try:
        odrv.save_configuration()
        time.sleep(pre)
        try:
            odrv.reboot()
        except Exception:
            pass
    except DeviceLostException:
        pass  # expected on some builds
    except Exception as e:
        print("save_configuration raised:", repr(e))
    time.sleep(post)
    return odrive.find_any(serial_number=serial) if serial else odrive.find_any()

def pick(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

def set_cfg_if_has(cfg, pairs):
    for k, v in pairs:
        if cfg and hasattr(cfg, k):
            setattr(cfg, k, v)

def configure_encoder_objects_and_map(odrv):
    """
    Configure SPI Absolute (AMS) 14-bit on whichever encoder object is exposed,
    then map Axis0 load+commutation encoders to Onboard Encoder 0.
    """
    # 1) Configure axis0.encoder if present
    enc = pick(odrv.axis0, "encoder")
    if enc and hasattr(enc, "config"):
        try:
            enc.config.mode = ENCODER_MODE_SPI_ABS_AMS
        except Exception:
            try: enc.config.mode = ENCODER_MODE_SPI_ABS
            except Exception: pass
        set_cfg_if_has(enc.config, [("cpr", 16384), ("bandwidth", 1000), ("use_index", False)])
        print("Configured axis0.encoder (SPI ABS, 14-bit) — OK")

    # 2) Configure the onboard encoder object too (Micro exposes this)
    onboard = pick(odrv, "onboard_encoder0", "encoder0")
    if onboard and hasattr(onboard, "config"):
        try:
            onboard.config.mode = ENCODER_MODE_SPI_ABS_AMS
        except Exception:
            try: onboard.config.mode = ENCODER_MODE_SPI_ABS
            except Exception: pass
        set_cfg_if_has(onboard.config, [("cpr", 16384), ("bandwidth", 1000), ("use_index", False)])
        print("Configured onboard encoder (SPI ABS, 14-bit) — OK")

    # 3) Map Axis0 to use the onboard encoder for both roles
    if hasattr(odrv.axis0, "config") and hasattr(odrv.axis0.config, "load_encoder"):
        odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
    if hasattr(odrv.axis0, "config") and hasattr(odrv.axis0.config, "commutation_encoder"):
        odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
    print("Axis0 mapped: load & commutation → ONBOARD_ENCODER0")

def main():
    serial = sys.argv[1] if len(sys.argv) > 1 else None

    print("Connecting to ODrive...")
    odrv = odrive.find_any(serial_number=serial) if serial else odrive.find_any()
    print(f"Connected: FW {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}  SN {hex(odrv.serial_number)}")
    print(f"Vbus: {odrv.vbus_voltage:.2f} V")

    # -------- Bus protection --------
    odrv.config.dc_bus_overvoltage_trip_level = BUS_OV
    odrv.config.dc_bus_undervoltage_trip_level = BUS_UV
    odrv.config.dc_max_positive_current = float('inf')
    odrv.config.dc_max_negative_current = -float('inf')

    ax = odrv.axis0

    # -------- Motor config (handle both layouts) --------
    motor_if  = pick(ax, "motor")
    motor_cfg = pick(motor_if, "config") if motor_if else None
    axis_cfg  = pick(ax, "config")
    axis_motor_cfg = pick(axis_cfg, "motor") if axis_cfg else None  # fallback on some builds

    def set_motor_param(name, value):
        ok = False
        if motor_cfg and hasattr(motor_cfg, name):
            setattr(motor_cfg, name, value); ok = True
        if axis_motor_cfg and hasattr(axis_motor_cfg, name):
            setattr(axis_motor_cfg, name, value); ok = True
        return ok

    if not set_motor_param("motor_type", MotorType.HIGH_CURRENT):
        raise RuntimeError("Motor config not accessible (update 'odrive' Python pkg and reflash matching FW).")

    set_motor_param("pole_pairs", POLE_PAIRS)
    set_motor_param("torque_constant", KT)
    if not set_motor_param("current_lim", I_LIM):
        set_motor_param("current_soft_max", I_LIM)
        set_motor_param("current_hard_max", I_LIM * 1.7)
    set_motor_param("calibration_current", I_CAL)
    set_motor_param("resistance_calib_max_voltage", V_CAL_MAX)
    if motor_cfg and hasattr(motor_cfg, "current_control_bandwidth"):
        motor_cfg.current_control_bandwidth = BW_CURRENT

    # Lock-in & soft torque window
    if axis_cfg and hasattr(axis_cfg, "calibration_lockin"):
        axis_cfg.calibration_lockin.current = LOCKIN_I
    if axis_cfg and hasattr(axis_cfg, "torque_soft_min"):
        axis_cfg.torque_soft_min = TORQUE_SOFT_MIN
    if axis_cfg and hasattr(axis_cfg, "torque_soft_max"):
        axis_cfg.torque_soft_max = TORQUE_SOFT_MAX

    # -------- Encoder(s) + mapping --------
    configure_encoder_objects_and_map(odrv)

    # -------- Controller (velocity + ramp) --------
    ctrl = pick(ax, "controller")
    ctrl_cfg = pick(ctrl, "config") if ctrl else None
    if ctrl_cfg:
        ctrl_cfg.control_mode = ControlMode.VELOCITY_CONTROL
        ctrl_cfg.input_mode   = InputMode.VEL_RAMP
        ctrl_cfg.vel_limit    = VEL_LIMIT
        ctrl_cfg.vel_limit_tolerance = 1.25
        ctrl_cfg.vel_ramp_rate = VEL_RAMP
        if hasattr(ctrl_cfg, "vel_gain"):            ctrl_cfg.vel_gain = VEL_GAIN
        if hasattr(ctrl_cfg, "vel_integrator_gain"): ctrl_cfg.vel_integrator_gain = VEL_IGAIN

    # -------- Optional startup behavior --------
    if axis_cfg:
        set_cfg_if_has(axis_cfg, [
            ("startup_motor_calibration", STARTUP_MOTOR_CAL),
            ("startup_encoder_offset_calibration", STARTUP_ENC_CAL),
            ("startup_closed_loop_control", STARTUP_CLOSED_LOOP),
        ])

    # -------- Save & reboot (robust) --------
    print("Saving configuration & rebooting (USB may drop briefly)...")
    odrv = safe_save_and_reboot(odrv, serial=serial)

    # -------- Full calibration --------
    print("Running FULL_CALIBRATION_SEQUENCE (shaft unloaded)...")
    odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    if not wait_idle(odrv.axis0, 90):
        print("Calibration timed out. Errors:")
        dump_errors(odrv, True); sys.exit(1)
    print("Calibration done. Error snapshot:")
    dump_errors(odrv, False)

    # -------- Harmonic compensation (if FW exposes it) --------
    hc_cfg = None
    if hasattr(odrv.axis0, "config") and hasattr(odrv.axis0.config, "harmonic_compensation"):
        hc_cfg = odrv.axis0.config.harmonic_compensation
        set_cfg_if_has(hc_cfg, [
            ("calib_vel", HC_VEL),
            ("calib_turns", HC_TURNS),
            ("calib_settling_delay", HC_SETTLE),
        ])
    has_hc_state = hasattr(AxisState, "HARMONIC_CALIBRATION")

    if hc_cfg and has_hc_state:
        print("Running Harmonic Calibration (shaft unloaded)...")
        odrv.axis0.requested_state = AxisState.HARMONIC_CALIBRATION
        if not wait_idle(odrv.axis0, 60):
            print("Harmonic calibration timed out. Errors:")
            dump_errors(odrv, True)
        else:
            coefs = [getattr(hc_cfg, k) if hasattr(hc_cfg, k) else None
                     for k in ("cosx_coef", "sinx_coef", "cos2x_coef", "sin2x_coef")]
            print("Harmonic coefficients:", coefs)
            print("Saving coefficients...")
            odrv = safe_save_and_reboot(odrv, serial=serial)
    else:
        print("Harmonic compensation not available on this FW/API; skipping.")

    # -------- Quick velocity test --------
    print("Quick velocity test at 1.0 turn/s (2 s)...")
    try:
        odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        odrv.axis0.controller.input_vel = 1.0
        time.sleep(2.0)
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis0.requested_state = AxisState.IDLE
    except Exception as e:
        print("Skipped test:", e)

    print("Final error snapshot:")
    dump_errors(odrv, False)
    print("Done.")

if __name__ == "__main__":
    main()
