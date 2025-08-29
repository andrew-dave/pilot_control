#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *

# Try to import the specific disconnect exception (varies by install)
try:
    from odrive.libodrive import DeviceLostException
except Exception:
    class DeviceLostException(Exception): pass

BAUD = 250000   # set to 0 to use autobaud on FW 0.6.11+
NODE_ID = 2     # your third device, with S1s at 0 and 1

def reconnect(serial=None):
    return odrive.find_any(serial_number=serial) if serial else odrive.find_any()

def safe_save_and_reboot(odrv, serial=None, pre=0.3, post=4.0):
    try:
        odrv.save_configuration()
        time.sleep(pre)
        try:
            odrv.reboot()
        except Exception:
            pass
    except DeviceLostException:
        # expected on some builds; the USB link drops during flash/reboot
        pass
    except Exception as e:
        print("save_configuration raised:", repr(e))
    time.sleep(post)
    return reconnect(serial)

def main():
    # --- connect ---
    odrv = reconnect()
    print(f"Connected: FW {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}  "
          f"SN {hex(odrv.serial_number)}  Vbus {odrv.vbus_voltage:.2f} V")

    # --- CAN interface (device-level) ---
    # Prefer Protocol.SIMPLE; fall back if enum name differs
    try:
        odrv.can.config.protocol = Protocol.SIMPLE
    except Exception:
        odrv.can.config.protocol = 1  # SIMPLE

    # Explicit baud for deterministic startup (or 0 for autobaud)
    try:
        odrv.can.config.baud_rate = BAUD
    except AttributeError:
        # Older API name fallback
        odrv.can.config.bitrate = BAUD

    # --- Axis-level CAN settings ---
    # Path is axis0.config.can.* on FW 0.6.x
    can_cfg = odrv.axis0.config.can
    can_cfg.node_id = NODE_ID
    # Cyclic messages (tune to taste / bus load)
    can_cfg.heartbeat_msg_rate_ms = 100   # 10 Hz heartbeat
    can_cfg.encoder_msg_rate_ms  = 10     # 100 Hz encoder feedback
    # Optional: set others to 0 to reduce traffic
    if hasattr(can_cfg, "iq_msg_rate_ms"):
        can_cfg.iq_msg_rate_ms = 0
    if hasattr(can_cfg, "vel_msg_rate_ms"):
        can_cfg.vel_msg_rate_ms = 0

    # Persist (handles the disconnect) and reconnect
    print("Saving & rebooting… (USB may drop briefly)")
    odrv = safe_save_and_reboot(odrv)

    # --- verify after reboot ---
    print("Reconnected.")
    print("Protocol:", odrv.can.config.protocol)
    try:
        print("Baud:", odrv.can.config.baud_rate)
    except AttributeError:
        print("Baud:", odrv.can.config.bitrate)
    print("Node ID:", odrv.axis0.config.can.node_id)
    print("HB / ENC rates (ms):",
          odrv.axis0.config.can.heartbeat_msg_rate_ms,
          odrv.axis0.config.can.encoder_msg_rate_ms)

if __name__ == "__main__":
    main()
