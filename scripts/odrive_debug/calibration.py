"""
ODrive Micro Calibration Script
================================
For ODrive Micro with ONBOARD magnetic encoder (built into PCB).

Key points:
- Motor R/L calibration: Can be pre-calibrated (values don't change between power cycles)
- Encoder offset calibration: MUST run at every power cycle for absolute encoders
  to establish the electrical phase offset. If skipped, MISSING_ESTIMATE errors occur.

After running this script, the startup sequence will be:
1. Power on → Motor calibration skipped (pre-calibrated)
2. Encoder offset calibration runs automatically (quick beep/vibration)
3. Motor enters IDLE, ready for CLOSED_LOOP_CONTROL
"""

import time, odrive
from odrive.enums import *

print("Connecting to ODrive Micro...")
odrv = odrive.find_any()
print(f"Connected: FW {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
print(f"Serial: {hex(odrv.serial_number)}")
print(f"Vbus: {odrv.vbus_voltage:.2f} V")

# Run full calibration (shaft free to spin)
print("\nRunning FULL_CALIBRATION_SEQUENCE (keep shaft free to spin)...")
odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
while odrv.axis0.current_state != AxisState.IDLE: 
    time.sleep(0.1)

# Check for errors
if odrv.axis0.active_errors != 0:
    print(f"ERROR: Calibration failed with errors: {odrv.axis0.active_errors}")
    from odrive.utils import dump_errors
    dump_errors(odrv, True)
    exit(1)

print("Calibration successful!")

# Mark MOTOR as pre-calibrated (motor R/L values don't change)
odrv.axis0.motor.config.pre_calibrated = True
print("✓ Motor marked as pre-calibrated")

# For ODrive Micro ONBOARD encoder (absolute magnetic encoder):
# Do NOT set encoder pre_calibrated = True
# The encoder offset calibration MUST run at each power cycle to establish
# the electrical phase offset. Without it, you get MISSING_ESTIMATE errors.
#
# The startup_encoder_offset_calibration=True in odrive_micro_calib.py
# ensures encoder offset calibration runs automatically at each power-up.
print("✓ Encoder NOT marked as pre-calibrated (offset cal will run at each startup)")

odrv.save_configuration()
print("\nConfiguration saved. Rebooting...")
time.sleep(0.3)
try: 
    odrv.reboot()
except: 
    pass

print("Done! On next power-up:")
print("  1. Motor calibration will be SKIPPED (pre-calibrated)")
print("  2. Encoder offset calibration will RUN (required for absolute encoder)")
print("  3. Motor enters IDLE, ready for CLOSED_LOOP")

