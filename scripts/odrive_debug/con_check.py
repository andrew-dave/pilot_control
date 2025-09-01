import time, odrive
from odrive.enums import *

def wait_idle(axis, t=60.0):
    t0=time.time()
    while time.time()-t0 < t:
        if axis.current_state == AxisState.IDLE:
            return True
        time.sleep(0.05)
    return False

def resolve_encoder(odrv):
    # (object name on device, EncoderId enum member)
    cands = [
        ("onboard_encoder0", "ONBOARD_ENCODER0"),  # ODrive Micro onboard AMS
        ("spi_encoder0",     "SPI_ENCODER0"),      # external SPI encoder
        ("encoder0",         "ENCODER0"),          # older layouts
    ]
    for obj_name, enum_name in cands:
        obj = getattr(odrv, obj_name, None)
        if obj is not None and hasattr(EncoderId, enum_name):
            return obj, getattr(EncoderId, enum_name)
    raise RuntimeError("No supported encoder object found on this FW/API")

print("Connecting…")
odrv = odrive.find_any()
ax = odrv.axis0

# 1) Clear anything stale
odrv.clear_errors()

# 2) Configure encoder object (assume AMS SPI 14-bit if fields exist) and map both roles
enc_obj, enc_id = resolve_encoder(odrv)
cfg = getattr(enc_obj, "config", None)
if cfg:
    if hasattr(cfg, "mode"):
        try: cfg.mode = ENCODER_MODE_SPI_ABS_AMS
        except Exception:
            try: cfg.mode = ENCODER_MODE_SPI_ABS
            except Exception: pass
    if hasattr(cfg, "cpr"):        cfg.cpr = 16384
    if hasattr(cfg, "bandwidth"):  cfg.bandwidth = 1000
    if hasattr(cfg, "use_index"):  cfg.use_index = False

axcfg = getattr(ax, "config", None)
if axcfg:
    if hasattr(axcfg, "load_encoder"):         axcfg.load_encoder = enc_id
    if hasattr(axcfg, "commutation_encoder"):  axcfg.commutation_encoder = enc_id

# 3) Make sure we’re not requiring a home/absolute reference
ctrl = getattr(ax, "controller", None)
ctrk = getattr(ctrl, "config", None)
if hasattr(ctrl, "absolute_setpoints"): ctrl.absolute_setpoints = False
if ctrk and hasattr(ctrk, "circular_setpoints"): ctrk.circular_setpoints = False

# Optional: set a reasonable vel limit wherever it exists
if ctrk and hasattr(ctrk, "vel_limit"): ctrk.vel_limit = 10.0
if axcfg and hasattr(axcfg, "vel_limit"): axcfg.vel_limit = 10.0

# 4) Save & reboot (USB may drop briefly)
print("Saving & rebooting…")
try:
    odrv.save_configuration()
    time.sleep(0.3)
    odrv.reboot()
except Exception:
    pass
time.sleep(4.0)
odrv = odrive.find_any()
ax = odrv.axis0

# 5) Run encoder offset calibration (shaft free)
print("Offset calibration…")
ax.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
if not wait_idle(ax, 45):
    from odrive.utils import dump_errors
    print("Calibration timed out / failed:")
    dump_errors(odrv, True)
    raise SystemExit

# 6) Enter closed loop and quick spin
print("Closed loop…")
ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
if hasattr(ax, "controller"):
    ax.controller.input_vel = 0.5
    time.sleep(1.5)
    ax.controller.input_vel = 0.0
ax.requested_state = AxisState.IDLE

from odrive.utils import dump_errors
print("Final error snapshot:")
dump_errors(odrv, False)
print("Done.")

