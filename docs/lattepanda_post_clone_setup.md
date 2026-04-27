# LattePanda Post-Clone Bring-Up Guide

This guide assumes you already restored a known-good LattePanda image onto a
new machine and now need to make that clone unique and field-ready.

It does not repeat the full fresh-install workflow from
`lattepanda_robot_production_setup.md`. It only covers the steps that should
happen after the cloned image first boots.

## Scope

This guide covers:

- making the cloned robot unique on the network
- regenerating identity files that must not be shared
- removing or reprovisioning secrets that must not be cloned
- checking the cloned workspace and robot-local config
- verifying `/R_DATA`, Livox, RF, CAN, and launch readiness

Assumptions:

- Ubuntu, ROS 2, and the workspace already exist in the cloned image
- the target workspace is `~/pilot_ws`
- the robot-local config file is `~/pilot_config/robot.yaml`

## Before First Network Connection

If possible, do the first boot with the new robot disconnected from the shared
robot network.

Why:

- the clone may still have the original hostname
- the clone may still have the original `machine-id`
- the clone may still have the original SSH host keys
- NetworkManager profiles may autoconnect immediately

Best practice:

1. Boot the cloned LattePanda on local HDMI and keyboard first.
2. Complete the uniqueness steps below.
3. Only then reconnect it to the normal robot network.

## 1. Set The New Hostname

```bash
sudo hostnamectl set-hostname <new-robot-hostname>
hostnamectl
```

If you also keep a static hostname entry in `/etc/hosts`, verify it still makes
sense for this robot.

## 2. Regenerate `machine-id`

The cloned robot must not keep the source robot's machine identity.

```bash
sudo rm -f /etc/machine-id /var/lib/dbus/machine-id
sudo systemd-machine-id-setup
sudo ln -s /etc/machine-id /var/lib/dbus/machine-id
cat /etc/machine-id
```

Expected:

- `/etc/machine-id` exists
- it contains a new unique value

## 3. Regenerate SSH Host Keys

The cloned robot must not reuse the source robot's SSH host identity.

```bash
sudo rm -f /etc/ssh/ssh_host_*
sudo ssh-keygen -A
sudo systemctl restart ssh
sudo ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

Expected:

- new host keys exist under `/etc/ssh/`
- the new robot presents a different SSH fingerprint than the source robot

Note:

- laptops that previously connected to the source robot may need their
  `known_hosts` entry updated for the new hostname or IP

## 4. Ensure User Git SSH Keys Are Unique Or Absent

Do not keep cloned user SSH private keys if the new robot should have its own
git identity.

If on-robot git access is required, generate a fresh keypair:

```bash
rm -f ~/.ssh/id_ed25519 ~/.ssh/id_ed25519.pub
ssh-keygen -t ed25519 -C "<robot-specific-email-or-label>"
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
cat ~/.ssh/id_ed25519.pub
```

If the robot never needs to pull or push code directly, it is fine to leave the
user git SSH keys absent.

## 5. Remove And Reprovision Robot Auth Secrets

If the cloned image includes Coverage Planner or other robot auth state, remove
the old robot-specific identity and secret material.

Remove the cloned values:

```bash
sudo rm -f /etc/pilot_robot_id
sudo rm -f /etc/pilot_auth_secret.key
sudo rm -f /etc/pilot_access_pin.hash
sudo rm -rf /var/lib/pilot_control/*
```

Then provision fresh values for this robot if that workflow is used in your
deployment:

- a new robot ID
- a new auth secret
- a new PIN hash

If your workflow does not use Coverage Planner auth, those files can remain
absent.

## 6. Verify The Cloned Workspace

Check that the workspace exists where the cloned image expects it:

```bash
ls ~/pilot_ws
ls ~/pilot_ws/src
```

Check the workspace state:

```bash
cd ~/pilot_ws
git status --short
```

If the clone was meant to preserve a known-good deployment state, verify the
expected commit or tag before changing anything else.

If the clone was restored under a different Linux username or a different home
directory than the source image used, do a clean rebuild before field use.

## 7. Review The Machine-Local Robot Config

The robot-specific values are intended to live outside the repo in:

- `~/pilot_config/robot.yaml`

Create or refresh it:

```bash
mkdir -p ~/pilot_config
cp -n ~/pilot_ws/src/pilot_control/config/robot_config.example.yaml ~/pilot_config/robot.yaml
nano ~/pilot_config/robot.yaml
```

Review at minimum:

- `robot.wheel_radius`
- `robot.wheel_base`
- `can.interface`
- `can.bitrate`
- `can.left_node_id`
- `can.right_node_id`
- `can.gpr_node_id`
- `cameras.left_device`
- `cameras.right_device`
- `stream.host`
- `stream.port`
- `gps.device`
- `arduino.serial_port` (normally keep `/dev/arduino`; use `/dev/ttyACM*` only
  as a temporary fallback while fixing udev)
- `preflight.rf_target_ip`

If you use a non-default path:

```bash
export PILOT_ROBOT_CONFIG=/full/path/to/robot.yaml
```

## 8. Verify `/R_DATA`

Check that the data SSD is mounted at `/R_DATA` and writable:

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS
df -h /R_DATA
stat -c '%n %F %U:%G %a' /R_DATA
```

If the cloned robot boots into emergency mode before the new SSD is installed,
the most common cause is a stale `/R_DATA` UUID from the source machine in
`/etc/fstab`.

Recover like this:

```bash
journalctl -xb -p err..alert
lsblk -f
cat /etc/fstab
```

Then temporarily comment out the stale `/R_DATA` line in `/etc/fstab`, boot the
robot normally, and finish the SSD setup below.

If the new robot does not have a data SSD yet, or the source UUID is no longer
valid, provision the new SSD with a fresh filesystem and a fresh UUID instead of
reusing the old `fstab` entry.

Identify the target disk first:

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS,MODEL
```

If the target partition does not exist yet, create it:

```bash
sudo parted /dev/<new-disk> -- mklabel gpt
sudo parted /dev/<new-disk> -- mkpart primary ext4 1MiB 100%
```

If the partition exists in `lsblk` but:

- `sudo blkid /dev/<new-disk-partition>` prints nothing
- `lsblk -f` shows a blank `FSTYPE`

that usually means the partition exists but no filesystem has been created yet.
Format it:

```bash
sudo mkfs.ext4 -L R_DATA /dev/<new-disk-partition>
sudo blkid /dev/<new-disk-partition>
```

Add the new UUID to `/etc/fstab`. During clone bring-up, `nofail` is
recommended so the robot still boots cleanly even if the SSD is temporarily not
detected:

```fstab
UUID=<new-r-data-uuid> /R_DATA ext4 defaults,nofail 0 2
```

Test the mount before rebooting:

```bash
sudo mkdir -p /R_DATA
sudo mount -a
df -h /R_DATA
stat -c '%n %F %U:%G %a' /R_DATA
sudo chown "$USER:$USER" /R_DATA
```

Create the expected subdirectories if needed:

```bash
mkdir -p /R_DATA/tilt_calibration
mkdir -p /R_DATA/startup_check
mkdir -p /R_DATA/unified_scans
mkdir -p /R_DATA/gpr_scans
mkdir -p /R_DATA/raw_maps
```

If the cloned OS does not yet have the correct data mount, fix the SSD mount
first and then return here.

## 9. Verify Livox, RF, And Network Profiles

Review the active NetworkManager profiles:

```bash
nmcli connection show
nmcli connection show --active
nmcli device status
```

For Livox, verify the expected host and sensor IP plan still matches the new
robot:

- host IP typically `192.168.1.50/24`
- MID360 IP typically `192.168.1.127`

Review:

- `~/pilot_ws/src/livox_ros_driver2/config/MID360_config.json`

If using the floating `livox-flex` profile approach, verify:

- only one Ethernet interface owns the Livox host IP
- `connection.autoconnect yes` is enabled on `livox-flex`
- generic wired profiles are not stealing the link

For RF or laptop communication, verify the target values in:

- `~/pilot_config/robot.yaml`

## 10. Verify Udev And Device Paths

Make sure the cloned robot still sees the expected stable device paths:

```bash
ls -l /dev/arduino
ls -l /dev/gps
ls -l /dev/v4l/by-id
lsusb | rg "8036|Arduino|Leonardo"
```

The most important checks are:

- `/dev/arduino`
- `/dev/gps`
- the two expected RGB camera by-id paths

On new robots, `/dev/arduino` should point to the LattePanda's onboard
Leonardo-compatible controller, usually a `/dev/ttyACM*` device behind the
stable symlink.

If `/dev/arduino` is missing after restoring the clone:

- reinstall and reload `src/pilot_control/config/99-arduino.rules`
- inspect the live ACM device with
  `udevadm info -a -n /dev/ttyACM0 | rg "idVendor|idProduct"`
- temporarily set `arduino.serial_port` in `~/pilot_config/robot.yaml` to the
  live `/dev/ttyACM*` path until the udev rule matches that robot

If the camera IDs differ on the new robot, update `~/pilot_config/robot.yaml`
before launching.

## 11. Verify Runtime Environment

Source the shell and check the expected environment:

```bash
source ~/.bashrc
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DOMAIN_ID"
echo "$CYCLONEDDS_URI"
```

Expected current-LP values:

- `rmw_cyclonedds_cpp`
- `0`
- `file:///home/$USER/cyclone_loopback.xml`

If this clone is meant to match the current production LP closely, also verify
CPU isolation:

```bash
cat /proc/cmdline
```

Expected kernel args:

- `isolcpus=0-1`
- `nohz_full=0-1`
- `rcu_nocbs=0-1`

## 12. Rebuild Only If Needed

If the clone preserved the same username, workspace path, and package state, a
rebuild may not be necessary.

Rebuild if any of these changed:

- Linux username
- workspace location
- source revisions
- package dependencies
- external config integration or launch code

Typical rebuild command:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select \
  odrive_can livox_ros_driver2 fast_lio pilot_control
```

## 13. Run Validation

Run the same validation sequence used on a fresh deployment:

```bash
source ~/.bashrc
ldconfig -p | grep -E 'livox_lidar_sdk|seekcamera|jxl'
ip -details link show can0
ros2 run pilot_control startup_preflight
ros2 run pilot_control tilt_calibration
```

If preflight passes, launch the robot stack:

```bash
ros2 launch pilot_control robot_complete.launch.py
```

Indoor mode if needed:

```bash
ros2 launch pilot_control robot_complete.launch.py scan_mode:=indoor
```

## Final Post-Clone Checklist

The cloned robot is ready only when all items below are true:

- hostname is unique
- `machine-id` is unique
- SSH host keys are unique
- user git SSH keys are unique or intentionally absent
- cloned robot auth secrets have been removed or reprovisioned
- `~/pilot_config/robot.yaml` matches the physical robot
- `/R_DATA` is mounted and writable
- Livox networking matches the target robot
- the onboard Leonardo resolves to `/dev/arduino` and `/dev/gps` is correct
- camera by-id paths are correct
- `startup_preflight` passes
- tilt calibration is present or recreated
- `robot_complete.launch.py` starts cleanly

## Quick Commands

```bash
hostnamectl
cat /etc/machine-id
sudo ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
nmcli connection show --active
df -h /R_DATA
cat ~/pilot_config/robot.yaml
ros2 run pilot_control startup_preflight
ros2 launch pilot_control robot_complete.launch.py
```
