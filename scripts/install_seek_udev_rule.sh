#!/usr/bin/env bash
# install_seek_udev_rule.sh — one-time operator setup.
#
# Copies the Seek Thermal udev rule into /etc/udev/rules.d/, reloads
# udevd, and re-fires the `add` event so the rule applies to the
# already-plugged Seek device without requiring an unplug/replug.
#
# Run once on each robot, after installing/updating the pilot_control
# package.  Requires sudo (writing to /etc/udev/ + reloading udevd).
#
# Idempotent: re-running it just overwrites the rule with whatever
# the current package ships and re-triggers, so you can safely call
# it from a deploy script.
#
# Verification (the script prints this for the operator):
#   stat -c '%G %a' /sys/bus/usb/devices/<port>/authorized
# should show `roofus 664` for the Seek device.

set -euo pipefail

RULE_NAME="99-bdr-seek-thermal.rules"
DEST="/etc/udev/rules.d/${RULE_NAME}"

# Locate the source rule.  Search ROS share dir first (installed
# package), then alongside this script (running directly from src).
candidates=(
    "$(ros2 pkg prefix pilot_control 2>/dev/null)/share/pilot_control/udev/${RULE_NAME}"
    "$(dirname "$(readlink -f "$0")")/../udev/${RULE_NAME}"
    "$(dirname "$(readlink -f "$0")")/../share/pilot_control/udev/${RULE_NAME}"
)
SRC=""
for c in "${candidates[@]}"; do
    if [ -f "$c" ]; then
        SRC="$c"
        break
    fi
done
if [ -z "$SRC" ]; then
    echo "ERROR: could not locate ${RULE_NAME} in any expected location:" >&2
    printf '  %s\n' "${candidates[@]}" >&2
    exit 1
fi

echo "Installing ${RULE_NAME}: ${SRC} -> ${DEST}"
sudo install -m 0644 -o root -g root "$SRC" "$DEST"

echo "Reloading udev rules..."
sudo udevadm control --reload-rules

echo "Triggering re-evaluation for any already-plugged Seek devices..."
# We trigger BY DEVICE PATH for every plugged Seek (matched by VID
# 289d) instead of using `--attr-match=idVendor=289d`.  Field-tested
# May 2026 on Ubuntu 22.04 / udev 249: the `--attr-match` filter
# silently matches nothing on this udevadm version even though the
# attribute is clearly present in sysfs (`udevadm test` confirms the
# rule WOULD fire).  Path-targeted triggers always work because they
# bypass the filter engine entirely.
triggered=0
for f in /sys/bus/usb/devices/*/idVendor; do
    [ -f "$f" ] || continue
    vid=$(cat "$f" 2>/dev/null)
    if [ "${vid,,}" = "289d" ]; then
        dev_dir=$(dirname "$f")
        echo "  -> triggering ${dev_dir}"
        sudo udevadm trigger --action=add "$dev_dir" || true
        triggered=$((triggered + 1))
    fi
done
if [ "$triggered" -eq 0 ]; then
    echo "  (no Seek device currently plugged in; rule will apply automatically on next plug)"
fi
# Brief settle so the verify block below sees the post-RUN state.
sleep 0.5

echo
echo "Done.  Verify the per-device authorized file is now group-writable:"
echo
for f in /sys/bus/usb/devices/*/idVendor; do
    [ -f "$f" ] || continue
    vid=$(cat "$f" 2>/dev/null)
    if [ "${vid,,}" = "289d" ]; then
        dev_dir=$(dirname "$f")
        auth="${dev_dir}/authorized"
        if [ -f "$auth" ]; then
            stat -c "  %n  ->  group=%G  mode=%a" "$auth"
        fi
    fi
done
echo
echo "Expected: group=roofus  mode=664"
echo "If group/mode is wrong, the rule installed but didn't apply -- try:"
echo "  sudo udevadm trigger --action=add --subsystem-match=usb"
