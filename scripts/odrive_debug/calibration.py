import time, odrive
from odrive.enums import *

odrv = odrive.find_any()

# Run full calibration (shaft free to spin)
odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
while odrv.axis0.current_state != AxisState.IDLE: time.sleep(0.1)

# Mark results as pre-calibrated
odrv.axis0.motor.config.pre_calibrated = True

# Find the actual encoder object your FW exposes
for name in ("onboard_encoder0","spi_encoder0","encoder0"):
    if hasattr(odrv, name):
        getattr(odrv, name).config.pre_calibrated = True
        break

odrv.save_configuration()
time.sleep(0.3)
try: odrv.reboot()
except: pass

