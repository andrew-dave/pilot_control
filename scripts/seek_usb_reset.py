#!/usr/bin/env python3
"""seek_usb_reset.py — programmatic Seek thermal USB reset.

Triggered by the OCU's `udc_supervisor` when the Seek SDK appears
wedged (no SEEKCAMERA_MANAGER_EVENT_CONNECT within 5 s, OR runtime
drop-rate climbs while recording).  Equivalent to physically
unplugging and replugging the Seek USB cable, but executed in
software via the per-device sysfs `authorized` toggle.

Why `authorized` and not driver `unbind`/`bind`?
    `authorized` is per-device (writable on a single sysfs path that
    udev can chmod), whereas `unbind`/`bind` are driver-global files
    owned by root with no clean per-device permission grant path.
    Toggling `authorized` 1->0->1 disconnects the kernel driver,
    drops the USB device, then reauthorizes it — same end effect as
    physical replug from the userspace POV.

Permission requirements:
    Writable `authorized` for the `roofus` group.  Granted by the
    udev rule shipped at `udev/99-bdr-seek-thermal.rules` (one-time
    operator install via `scripts/install_seek_udev_rule.sh`).
    Falls back to `sudo -n` if the udev rule isn't installed; surfaces
    a clear error if neither path works.

Exit codes:
    0  — Reset succeeded; Seek device re-enumerated within timeout.
    1  — No Seek device found before reset (already unplugged?).
    2  — Permission denied writing `authorized` (udev rule + sudo
         both unavailable; operator action required).
    3  — Re-enumeration timed out (device went away but didn't
         come back; likely a hardware fault).
    4  — Unknown error (uncaught exception; stderr has details).
"""

from __future__ import annotations

import argparse
import errno
import glob
import os
import shutil
import subprocess
import sys
import time
from typing import Optional


SEEK_VENDOR_ID = "289d"
SYS_USB_DEVICES = "/sys/bus/usb/devices"

# Tunable timings.  Picked for the LattePanda Sigma's USB controller
# behavior — re-enumeration usually completes within ~600 ms but we
# pad to 3 s for cold boots / busy buses.
DEAUTH_HOLD_S = 0.5
ENUMERATE_TIMEOUT_S = 3.0
ENUMERATE_POLL_INTERVAL_S = 0.1


def _read_attr(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="ascii") as f:
            return f.read().strip()
    except OSError:
        return None


def _find_seek_devices() -> list[str]:
    """Return list of /sys/bus/usb/devices/<port> paths matching Seek VID.

    Excludes interfaces (paths containing ':').  We want the *device*
    not the interface — `authorized` lives on the device.
    """
    matches: list[str] = []
    for vendor_path in sorted(glob.glob(f"{SYS_USB_DEVICES}/*/idVendor")):
        if ":" in os.path.basename(os.path.dirname(vendor_path)):
            continue  # interface path, skip
        vid = _read_attr(vendor_path)
        if vid and vid.lower() == SEEK_VENDOR_ID:
            matches.append(os.path.dirname(vendor_path))
    return matches


def _write_authorized(device_path: str, value: str) -> None:
    """Write `value` to <device_path>/authorized.  Tries plain write
    first, then falls back to `sudo -n tee` on PermissionError so the
    script works even if the udev rule wasn't installed (paying a
    sudo password-cache hit instead of failing outright).

    Raises PermissionError if both paths fail.
    """
    target = os.path.join(device_path, "authorized")
    try:
        with open(target, "w", encoding="ascii") as f:
            f.write(value)
        return
    except PermissionError:
        pass

    # Fallback: sudo -n tee.  -n means non-interactive; if no NOPASSWD
    # rule exists this fails immediately rather than blocking on a
    # tty prompt that udc_supervisor will never see.
    sudo = shutil.which("sudo")
    tee = shutil.which("tee") or "/usr/bin/tee"
    if not sudo:
        raise PermissionError(
            f"plain write to {target} denied and `sudo` not found on PATH"
        )
    proc = subprocess.run(
        [sudo, "-n", tee, target],
        input=value.encode("ascii"),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        timeout=5,
        check=False,
    )
    if proc.returncode != 0:
        # Don't surface sudo's raw stderr verbatim — it always contains
        # the substring "password" (e.g. "sudo: a password is required"),
        # which can trip crude downstream regex checks that flag the
        # entire robot launch as failed.  The OCU's
        # AppShellWindow::onRobotLaunchOutput pattern-match used to do
        # exactly this and froze the launch diagnostics on a benign
        # warning.  The actionable hint (run install_seek_udev_rule.sh)
        # is already printed by the caller in scripts/seek_usb_reset.py
        # `_run_reset_cycle` right after this exception is caught, so
        # dropping the raw stderr here loses nothing the operator needs.
        raise PermissionError(
            f"sudo -n tee {target} denied (rc={proc.returncode}); "
            f"NOPASSWD rule for /sys path is missing"
        )


def _wait_for_reenumeration(timeout_s: float) -> bool:
    """Poll until at least one Seek device shows up in sysfs again."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _find_seek_devices():
            return True
        time.sleep(ENUMERATE_POLL_INTERVAL_S)
    return False


def reset_seek(reason: str = "unspecified") -> int:
    """Top-level reset routine.  Returns process exit code."""
    devices = _find_seek_devices()
    if not devices:
        print(
            f"[seek_usb_reset] no Seek device (VID {SEEK_VENDOR_ID}) "
            f"found under {SYS_USB_DEVICES}; cannot reset (reason={reason})",
            file=sys.stderr,
        )
        return 1

    if len(devices) > 1:
        # Multi-Seek setups aren't expected on Roofus, but log so we
        # don't silently reset only the first one.
        print(
            f"[seek_usb_reset] WARNING: {len(devices)} Seek devices found; "
            "resetting all of them",
            file=sys.stderr,
        )

    print(
        f"[seek_usb_reset] resetting {len(devices)} Seek device(s) "
        f"(reason={reason}): {', '.join(os.path.basename(d) for d in devices)}",
        file=sys.stderr,
    )

    for dev in devices:
        try:
            _write_authorized(dev, "0")
        except PermissionError as exc:
            print(
                f"[seek_usb_reset] PERMISSION DENIED on {dev}/authorized: {exc}\n"
                "  -> install udev rule via "
                "`ros2 run pilot_control install_seek_udev_rule.sh` "
                "(one-time operator setup)",
                file=sys.stderr,
            )
            return 2
        except OSError as exc:
            print(
                f"[seek_usb_reset] OSError deauthorizing {dev}: {exc}",
                file=sys.stderr,
            )
            return 4

    time.sleep(DEAUTH_HOLD_S)

    for dev in devices:
        # Reauthorize: write "1" back.  If the dev path is gone (the
        # kernel may have torn it down on deauth), skip — _wait_for_re-
        # enumeration below will catch the new device path.
        try:
            _write_authorized(dev, "1")
        except FileNotFoundError:
            continue
        except PermissionError as exc:
            print(
                f"[seek_usb_reset] PERMISSION DENIED on {dev}/authorized "
                f"(reauthorize): {exc}",
                file=sys.stderr,
            )
            return 2
        except OSError as exc:
            if exc.errno in (errno.ENODEV, errno.ENOENT):
                continue
            print(
                f"[seek_usb_reset] OSError reauthorizing {dev}: {exc}",
                file=sys.stderr,
            )
            return 4

    if not _wait_for_reenumeration(ENUMERATE_TIMEOUT_S):
        print(
            f"[seek_usb_reset] re-enumeration timed out after "
            f"{ENUMERATE_TIMEOUT_S:.1f}s; Seek device did NOT come back. "
            "Likely a hardware fault — operator may need to physically "
            "replug the cable.",
            file=sys.stderr,
        )
        return 3

    print(
        "[seek_usb_reset] success: Seek device re-enumerated",
        file=sys.stderr,
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reason",
        default="unspecified",
        help="Free-form string written to the log so operators can correlate "
        "reset events with the supervisor decision that triggered them.",
    )
    args = parser.parse_args()
    try:
        return reset_seek(reason=args.reason)
    except Exception as exc:  # noqa: BLE001 — top-level guard
        print(f"[seek_usb_reset] uncaught exception: {exc!r}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    sys.exit(main())
