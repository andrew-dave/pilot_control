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
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=289d || true

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
