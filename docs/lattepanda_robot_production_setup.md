# LattePanda Robot Production Setup Guide

This guide provisions a fresh LattePanda to run the robot-side stack in
`src/pilot_control/launch/robot_complete.launch.py`.

It is written as a production go-to guide, not a development scratchpad. Follow
it in order on a clean Ubuntu install.

If this robot was restored from a previously captured image instead of a clean
install, do the post-clone uniqueness and storage steps in
`lattepanda_post_clone_setup.md` before reconnecting it to the normal network.

## Scope

This guide covers:

- Ubuntu installation and baseline OS setup
- git setup and repository access
- ROS 2 Humble installation
- native SDKs and non-ROS binaries
- workspace layout and build
- udev rules, Linux permissions, and data directories
- Livox, RF, GPS, camera, Arduino, and CAN/ODrive setup
- production boot automation with systemd
- calibration, validation, and first launch
- optional Coverage Planner robot auth provisioning

This guide assumes the LattePanda is the robot computer and the target launch is
`robot_complete.launch.py`.

## What The Launch Expects

The current launch and supporting code expect the following robot hardware and
runtime layout:

- Ubuntu 22.04
- ROS 2 Humble
- CycloneDDS (`rmw_cyclonedds_cpp`)
- Livox MID360
- 2x e-con See3CAM 24CUG USB RGB cameras
- 1x Seek Thermal USB camera
- 1x u-blox ZED-F9P GNSS receiver on `/dev/gps`
- 1x LattePanda onboard Arduino Leonardo-compatible controller on `/dev/arduino`
- 3x ODrives on `can0` with node IDs `0`, `1`, and `2`
- writable robot data root at `/R_DATA`
- `zenohd` plus the `ros2dds` plugin for robot-to-laptop comms

Robot-specific assumptions are currently hardcoded in:

- `src/pilot_control/launch/robot_complete.launch.py`
- `src/pilot_control/config/fastlio_mid360.yaml`
- `src/livox_ros_driver2/config/MID360_config.json`
- `src/pilot_control/config/99-arduino.rules`
- `src/pilot_control/config/99-ublox-gps.rules`

## Before You Start

Collect these values before touching the robot:

- robot Linux username, for example `<robot-user>`
- robot hostname, for example `<robot-hostname>`
- git server hostname, for example `github.com` or your internal GitLab
- approved git branch, tag, or commit SHA for each repo
- Livox host IP for the LattePanda NIC, if different from `192.168.1.50/24`
- Livox MID360 IP, if different from `192.168.1.127`
- RF or laptop IP for streaming and Zenoh reachability
- whether the robot will run `scan_mode:=outdoor` or `scan_mode:=indoor`
- whether Coverage Planner GUI auth is required

## 1. Install Ubuntu And Baseline OS Packages

Install `Ubuntu 22.04 LTS` on the LattePanda.

After first boot, log in as the robot user and apply the baseline OS setup:

```bash
sudo apt update
sudo apt full-upgrade -y

sudo apt install -y \
  openssh-server curl wget gnupg lsb-release ca-certificates \
  software-properties-common jq vim tmux htop tree unzip net-tools ripgrep \
  usbutils pciutils iproute2 iputils-ping

sudo systemctl enable --now ssh
sudo hostnamectl set-hostname <robot-hostname>
sudo timedatectl set-timezone <Region/City>
sudo timedatectl set-ntp true
```

Recommended for production robots:

- disable automatic suspend and hibernate
- set a fixed hostname that matches your asset inventory
- verify the system clock is correct before collecting any data

To disable sleep on a field robot:

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

## 2. Install And Configure Git Access

Install git and optional helper packages:

```bash
sudo apt install -y git git-lfs
git lfs install
```

Set your git identity:

```bash
git config --global user.name "<Your Name>"
git config --global user.email "<your-email@example.com>"
```

Generate an SSH key for repository access:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
ssh-keygen -t ed25519 -C "<your-email@example.com>"
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
```

Print the public key and add it to your git hosting account:

```bash
cat ~/.ssh/id_ed25519.pub
```

Test SSH access:

```bash
ssh -T git@<git-host>
```

Production note:

- pin the robot to approved commits or tags
- do not deploy from floating feature branches unless that is part of your release process
- record the exact commit SHAs in the robot deployment log

## 3. Create The Workspace And Clone The Required Sources

Choose one of the source layouts below.

### Option A: Single Internal Monorepo

If your team stores this exact workspace in a single repo, do this instead of
manually cloning individual repos into `~/pilot_ws/src`:

```bash
cd ~
git clone --recurse-submodules <workspace-repo-url> pilot_ws
cd ~/pilot_ws
git checkout <approved-tag-or-commit>
git submodule update --init --recursive
```

If this monorepo intentionally omits packages such as `serial` or
`pointcloud_to_grid`, that is expected. Use the monorepo-specific build command
in Step 6 rather than copying the multi-repo package list verbatim.

### Option B: Multi-Repo Layout

If your team stores the workspace as separate repos, clone them into
`~/pilot_ws/src` so the final tree contains:

```bash
mkdir -p ~/pilot_ws/src
cd ~/pilot_ws/src
```

- `pilot_control`
- `FAST_LIO`
- `livox_ros_driver2`
- `ros_odrive`
- `serial`
- `Livox-SDK2`

Example:

```bash
cd ~/pilot_ws/src

git clone <pilot_control-repo-url> pilot_control
git clone <fast_lio-repo-url> FAST_LIO
git clone <livox_ros_driver2-repo-url> livox_ros_driver2
git clone <ros_odrive-repo-url> ros_odrive
git clone <serial-repo-url> serial
git clone <livox_sdk2-repo-url> Livox-SDK2

cd pilot_control && git checkout <approved-tag-or-commit> && cd ..
cd FAST_LIO && git checkout <approved-tag-or-commit> && cd ..
cd livox_ros_driver2 && git checkout <approved-tag-or-commit> && cd ..
cd ros_odrive && git checkout <approved-tag-or-commit> && cd ..
cd serial && git checkout <approved-tag-or-commit> && cd ..
cd Livox-SDK2 && git checkout <approved-tag-or-commit> && cd ..
```

If your source tree already exists by other means, verify the final layout before
continuing:

```text
~/pilot_ws/
  src/
    pilot_control/
    FAST_LIO/
    livox_ros_driver2/
    ros_odrive/
    serial/
    Livox-SDK2/
```

## 4. Install ROS 2 Humble And System Dependencies

On a fresh Ubuntu image, first do the official ROS 2 Humble apt bootstrap.

Set a UTF-8 locale:

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Enable `universe` and install the official ROS apt source package:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
```

Add the Zenoh signing key and repository in its own source-list file:

```bash
sudo install -d -m 0755 /etc/apt/keyrings
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null

sudo apt update
```

Important:

- keep the Zenoh repo in `/etc/apt/sources.list.d/zenoh.list`
- do not place the Zenoh repo directly in `/etc/apt/sources.list`, because
  tools such as `add-apt-repository` can rewrite that file in unsafe ways for
  third-party repos

Then install ROS 2 Humble and the non-ROS packages used by the current
robot-side stack:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cyclonedds \
  ros-humble-pcl-conversions \
  ros-humble-rosbag2-storage-mcap \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential cmake pkg-config python3-dev \
  python3-numpy python3-opencv python3-serial python3-scipy \
  python3-yaml \
  can-utils \
  libeigen3-dev libpcl-dev libopencv-dev libapr1-dev libbrotli-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-vaapi \
  zenohd \
  zenoh-plugin-ros2dds
```

Initialize `rosdep` if this is the first ROS setup on the machine:

```bash
sudo rosdep init
rosdep update
```

Add the core shell setup:

```bash
grep -q "/opt/ros/humble/setup.bash" ~/.bashrc || cat <<'EOF' >> ~/.bashrc
source /opt/ros/humble/setup.bash
EOF

source ~/.bashrc
```

Notes:

- If `/opt/ros/humble/setup.bash` already exists, the ROS repository bootstrap
  may already be present, but it is still safe to verify it.
- `libjxl-dev` is not reliably available from the default Ubuntu 22.04 repos
  used on these robots
- install `libbrotli-dev` here, then build and install `libjxl` from source in
  Step 5.4
- `gpr_scan_controller.py` records bags with `ros2 bag ... --storage mcap`, so
  `ros-humble-rosbag2-storage-mcap` is an intentional runtime dependency.
- `robot_complete.launch.py` starts the Accel MPC controller; `python3-scipy`
  is installed above and `osqp` is installed in Step 5.2.1.
- The robot-side video sender paths depend on runtime GStreamer elements such as
  `x264enc`, `h264parse`, and `rtph264pay`; the low-latency Intel hardware path
  also uses `vaapih264enc`, so those plugin packages are intentional runtime
  requirements.

## 5. Install Native SDKs And External Binaries

Some robot-side dependencies are not covered by normal ROS package manifests and
must be installed manually.

### 5.1 Livox SDK2

`livox_ros_driver2` expects the shared library from Livox SDK2 to be installed
into `/usr/local/lib`.

Build and install it from `~/pilot_ws/src/Livox-SDK2`:

```bash
cd ~/pilot_ws/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

Verify:

```bash
ldconfig -p | rg livox_lidar_sdk
```

### 5.2 Seek Thermal SDK

`pilot_control` links against `seekcamera`, so the Seek Thermal vendor SDK must
already be installed on the robot.

Required outcomes:

- `libseekcamera.so` is installed in a standard library path such as `/usr/lib`
  or `/lib`
- Seek headers are available under a standard include path such as
  `/usr/include/seekcamera`

If your private repo carries the vendor `.deb` inside `vendor/`, install it
from there. Example:

```bash
cd ~/pilot_ws
sudo apt install ./vendor/seekthermal-sdk-dev-<version>_amd64.deb
```

Verify:

```bash
dpkg -s seekthermal-sdk-dev
ls /usr/include/seekcamera
ls /lib/libseekcamera* /usr/lib/libseekcamera* 2>/dev/null
```

If those paths are missing, install the official Seek Thermal SDK package for
Ubuntu before building the workspace.

### 5.2.1 Python Runtime Extras For Accel MPC

`robot_complete.launch.py` starts
`src/pilot_control/scripts/mpc_accel_autonomous_controller.py`. That controller
requires both `scipy.sparse` and `osqp` in the same Python environment that ROS
2 uses.

Install and verify:

```bash
python3 -m pip install --user osqp
python3 -c "import osqp; from scipy import sparse; print('Accel MPC Python deps OK')"
```

Notes:

- `python3-scipy` is already included in the apt install list from Step 4.
- If you intentionally launch ROS nodes from a virtual environment, install both
  `osqp` and `scipy` into that environment too.

### 5.3 Zenoh Router And ROS 2 DDS Plugin

`robot_complete.launch.py` starts `zenohd` and loads the `ros2dds` plugin via
`src/pilot_control/config/zenoh/zenohd_robot.json5`, so both must be installed
on the robot.

Install the Eclipse Zenoh packages:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key \
  | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg

echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" \
  | sudo tee /etc/apt/sources.list.d/zenoh.list >/dev/null

sudo apt update
sudo apt install -y zenoh zenoh-plugin-ros2dds
```

Verify:

```bash
zenohd --version
```

### 5.4 JPEG XL

`libjxl` is recommended for the unified data collector. The code can fall back
to PNG if JPEG XL is unavailable, but production deployments should keep JPEG
XL enabled.

On Ubuntu 22.04, `libjxl-dev` is often unavailable in the default repos. The
current `pilot_control` prefers a system-wide `libjxl` install under
`/usr/local`, so build and install it from source:

```bash
sudo apt install -y build-essential cmake pkg-config libbrotli-dev git

cd ~
git clone https://github.com/libjxl/libjxl.git
cd libjxl
git submodule update --init --recursive

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ..
cmake --build . -- -j"$(nproc)"
sudo cmake --install .
sudo ldconfig
```

Verify:

```bash
ls /usr/local/include/jxl/encode.h
ldconfig -p | rg "libjxl|brotli"
```

Notes:

- do not run `./deps.sh` if you cloned the repo normally and already ran
  `git submodule update --init --recursive`
- `./deps.sh` is only needed when building from a source archive or other tree
  that does not already contain the required submodules
- the current `pilot_control` prefers `/usr/local/lib` first and only falls
  back to `~/libjxl`-style source builds if the system install is missing

## 6. Build The Workspace

Install ROS dependencies from the local source tree:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

With the current `pilot_control/package.xml`, this should resolve cleanly.
If `rosdep` still complains that it cannot resolve `opencv4`, your checkout is
older than the current setup guide. Update `src/pilot_control/package.xml` so
it depends on `libopencv-dev` and `python3-opencv`, or temporarily rerun
`rosdep` with `--skip-keys opencv4` after installing those packages manually.

If you are using the monorepo layout where `serial` is intentionally absent,
build the required robot-side packages with:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select \
  odrive_can livox_ros_driver2 fast_lio pilot_control
```

If you are using the multi-repo layout and `serial` is present in
`~/pilot_ws/src`, build with:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select \
  serial odrive_can livox_ros_driver2 fast_lio pilot_control
```

If `livox_ros_driver2` fails with a CMake error mentioning
`LIVOX_INTERFACES_INCLUDE_DIRECTORIES`, your checkout has the older Humble build
logic. Update `src/livox_ros_driver2/CMakeLists.txt` to the current revision.
As a temporary workaround only, rebuild with:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select livox_ros_driver2 \
  --cmake-args -DHUMBLE_ROS=humble
```

Add the workspace overlay to your shell:

```bash
echo 'source ~/pilot_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 7. Set The Production Runtime Environment

The current LattePanda uses CycloneDDS with default ROS domain `0`, and it keeps
CycloneDDS on loopback only. Cross-network robot-to-laptop communication is
handled by Zenoh rather than raw DDS multicast.

Create the local CycloneDDS config file:

```bash
cat <<'EOF' > ~/cyclone_loopback.xml
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>true</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="lo" priority="200" multicast="true"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
```

Append these exports to `~/.bashrc`:

```bash
cat <<'EOF' >> ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///home/$USER/cyclone_loopback.xml
source /opt/ros/humble/setup.bash
source ~/pilot_ws/install/setup.bash
EOF

source ~/.bashrc
```

Verify:

```bash
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DOMAIN_ID"
echo "$CYCLONEDDS_URI"
```

Expected values:

- `rmw_cyclonedds_cpp`
- `0`

If you intentionally want raw DDS traffic over Wi-Fi or RF instead of loopback +
Zenoh, do not blindly reuse the current robot's `CYCLONEDDS_URI`. Review that
networking design first.

## 7.1 Match The Current CPU Isolation And Affinity Setup

The current LattePanda does two separate things for timing-sensitive nodes:

1. it reserves CPU cores `0-1` at the kernel level
2. it pins specific nodes to those cores in `robot_complete.launch.py`

Current launch behavior:

- `livox_ros_driver2` runs with `taskset -c 1`
- `fast_lio` runs with `taskset -c 0`

To match the current robot, add these kernel arguments in
`/etc/default/grub`:

```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=0-1 nohz_full=0-1 rcu_nocbs=0-1"
```

Then apply and reboot:

```bash
sudo update-grub
sudo reboot
```

After reboot, verify:

```bash
cat /proc/cmdline
```

Expected entries:

- `isolcpus=0-1`
- `nohz_full=0-1`
- `rcu_nocbs=0-1`

Notes:

- the launch file already pins the LiDAR driver and Fast-LIO using `taskset`
- this is not just a convenience tweak; it is part of the current LP's runtime
  behavior
- the comments in `robot_complete.launch.py` also mention possible future
  pinning for `unified_data_collector` and `gpr_scan_controller`, but those are
  not actively pinned in the current launch

## 8. Create `/R_DATA` And Set Permissions

The launch writes robot data under `/R_DATA`, including tilt calibration, scan
data, GNSS logs, and preflight reports.

If the data SSD already exists, verify it:

```bash
lsblk -f
df -h /R_DATA
stat -c '%n %F %U:%G %a' /R_DATA
```

If this robot was restored from a cloned image and drops into emergency mode on
boot, the most common cause is a stale `/R_DATA` UUID from the source machine in
`/etc/fstab`.

In that case:

1. log in through the emergency shell
2. inspect `lsblk -f` and `cat /etc/fstab`
3. temporarily comment out the stale `/R_DATA` line in `/etc/fstab`
4. continue booting normally
5. install and provision the new SSD, then replace the commented line with the
   new UUID

If you are provisioning a new data SSD or replacement partition, use a fresh
filesystem and a fresh UUID instead of reusing the source robot's `fstab`
entry.

Identify the new disk first:

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS,MODEL
```

If the target partition does not exist yet, create it:

```bash
sudo parted /dev/<new-disk> -- mklabel gpt
sudo parted /dev/<new-disk> -- mkpart primary ext4 1MiB 100%
```

If the partition exists in `lsblk` but `blkid /dev/<new-disk-partition>`
prints nothing, that usually means the partition exists but no filesystem has
been created yet. Format it:

```bash
sudo mkfs.ext4 -L R_DATA /dev/<new-disk-partition>
sudo blkid /dev/<new-disk-partition>
```

Add the new UUID to `/etc/fstab`. During clone bring-up, `nofail` is
recommended so a missing or not-yet-installed data SSD does not force the robot
back into emergency mode:

```fstab
UUID=<new-r-data-uuid> /R_DATA ext4 defaults,nofail 0 2
```

Create the mountpoint, test it, and set ownership:

```bash
sudo mkdir -p /R_DATA
sudo mount -a
df -h /R_DATA
sudo chown "$USER:$USER" /R_DATA
mkdir -p /R_DATA/tilt_calibration /R_DATA/startup_check
```

Production recommendation:

- if the robot uses a dedicated SSD or data partition, mount it at `/R_DATA`
- keep `/R_DATA` writable by the robot user that launches ROS

## 9. Install Udev Rules And Linux Group Permissions

Install the stable symlink rules for GNSS and Arduino:

```bash
sudo cp ~/pilot_ws/src/pilot_control/config/99-arduino.rules /etc/udev/rules.d/
sudo cp ~/pilot_ws/src/pilot_control/config/99-ublox-gps.rules /etc/udev/rules.d/

sudo usermod -aG dialout,video "$USER"

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reboot or log out and back in so the new group membership takes effect.

After reboot, verify:

```bash
ls -l /dev/arduino
ls -l /dev/gps
ls -l /dev/v4l/by-id
lsusb | rg "8036|Arduino|Leonardo"
```

Expected device symlinks:

- `/dev/arduino`
- `/dev/gps`

The RGB cameras should also appear under `/dev/v4l/by-id/`.

On new robots, `/dev/arduino` should resolve to the LattePanda's onboard
Leonardo-compatible controller, usually a `/dev/ttyACM*` device behind the
stable symlink.

If `/dev/arduino` is missing:

- make sure the repo copy of `src/pilot_control/config/99-arduino.rules` was
  copied into `/etc/udev/rules.d/` and reloaded
- inspect the live ACM device with
  `udevadm info -a -n /dev/ttyACM0 | rg "idVendor|idProduct"`
- temporarily set `arduino.serial_port` in `~/pilot_config/robot.yaml` to the
  live `/dev/ttyACM*` path until the udev rule matches that robot
- treat `/dev/ttyACM*` as a temporary recovery path only; production should
  keep the stable `/dev/arduino` symlink because other launch flows still
  expect it

## 10. Flash The Arduino GPR Firmware

The robot-side GPR serial bridge is implemented by
`src/pilot_control/scripts/gpr_serial_bridge.py`, and the matching Arduino
sketch in this workspace is:

- `src/pilot_control/scripts/servo_gpr/servo_gpr.ino`

Flash that sketch to the LattePanda's onboard Arduino Leonardo-compatible
controller using the Leonardo board profile, for example
`arduino:avr:leonardo`.

Important:

- keep the Python bridge and Arduino sketch in sync
- do not substitute a different serial protocol unless you update both sides
- stop any process currently holding `/dev/ttyACM*` open before uploading
- after flashing, reconnect or reset the controller and confirm it appears at
  `/dev/arduino`
- if the board only comes back as `/dev/ttyACM*`, use that path temporarily in
  `~/pilot_config/robot.yaml` and fix the udev rule before field deployment

## 11. Verify The Camera Device Paths

The launch currently expects these exact See3CAM by-id paths:

- `/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0`
- `/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0`

Check the real device names on the robot:

```bash
ls -l /dev/v4l/by-id
```

If the camera serials or enumeration differ, update:

- `src/pilot_control/launch/robot_complete.launch.py`
- `src/pilot_control/scripts/startup_preflight.py`

## 12. Configure The Livox Ethernet Interface

The bundled MID360 config currently expects:

- host IP: `192.168.1.50`
- lidar IP: `192.168.1.127`

Verify in:

- `src/livox_ros_driver2/config/MID360_config.json`

Recommended production setup: use a single floating NetworkManager profile for
Livox, keep it off the default route, and let it autoconnect on whichever
Ethernet port currently has the sensor attached.

This is more flexible than binding the profile to one port-specific connection
name.

If an older port-specific Livox profile exists and you no longer want it, remove
it first:

```bash
nmcli connection show

sudo nmcli connection down "<old-livox-profile>" || true
sudo nmcli connection delete "<old-livox-profile>"
```

Create a new portable profile. Use either currently available Ethernet port for
the initial creation step, then clear the port binding:

```bash
nmcli device status

sudo nmcli connection add type ethernet \
  con-name livox-flex \
  ifname <one-ethernet-port-name>

sudo nmcli connection modify livox-flex \
  ipv4.method manual \
  ipv4.addresses "192.168.1.50/24" \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect yes \
  connection.autoconnect-priority 100

sudo nmcli connection modify livox-flex \
  connection.interface-name "" \
  802-3-ethernet.mac-address ""
```

Disable autoconnect on any generic wired profiles so they do not steal the port:

```bash
nmcli connection show

sudo nmcli connection modify "Wired connection 1" connection.autoconnect no
sudo nmcli connection modify "Wired connection 2" connection.autoconnect no
```

Bring the profile up once now:

```bash
sudo nmcli connection up livox-flex
```

Verify:

```bash
nmcli connection show --active
nmcli device status
ip -4 addr
ping -c 2 192.168.1.127
```

Important notes:

- only one Ethernet interface should hold `192.168.1.50/24` at a time
- do not assign the same static Livox host IP to both Ethernet ports
  simultaneously
- `connection.autoconnect yes` is the recommended automation if either physical
  port may be used for Livox and only one is connected at a time
- true "Livox product detection" is not a normal NetworkManager feature, since
  Ethernet does not expose remote-device vendor/product identity the way USB
  udev rules do

If you choose a different IP plan, update
`src/livox_ros_driver2/config/MID360_config.json` before launching.

## 13. Configure RF / Laptop Communication

The robot-side Zenoh router config is:

- `src/pilot_control/config/zenoh/zenohd_robot.json5`

It listens on:

- `tcp/0.0.0.0:7447`

The robot-side video stream defaults in `robot_complete.launch.py` currently
target:

- host `192.168.168.100`
- port `5600`

For production:

- ensure the robot can reach the laptop over the RF network
- allow `7447/tcp` through any firewall on the robot
- confirm the laptop IP used for video streaming is correct for this robot

The stream target can also be updated at runtime via `/stream_target_ip`, but
the launch defaults should still be reviewed per robot.

## 14. Configure CAN And ODrive Access

The launch expects:

- interface `can0`
- bitrate `250000`
- node IDs:
  - left: `0`
  - right: `1`
  - GPR: `2`

Manual bring-up command:

```bash
sudo ip link set can0 up type can bitrate 250000
ip -details link show can0
```

Important launch behavior:

- `robot_complete.launch.py` runs `sudo ip link set ...` internally
- if the launching user cannot run that non-interactively, launch startup may
  fail or stall

Production-safe options:

1. bring up `can0` before launching ROS
2. configure `can0` at boot with a systemd unit
3. allow passwordless `sudo` for the specific `ip link set can0 ...` command

Recommended production approach: bring `can0` up at boot with systemd.

Create `/etc/systemd/system/robot-can.service`:

```ini
[Unit]
Description=Bring up robot CAN interface
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set can0 down
ExecStart=/usr/sbin/ip link set can0 up type can bitrate 250000
ExecStop=/usr/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

Enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-can.service
systemctl status robot-can.service
```

Also verify the ODrives are configured to emit the cyclic messages required by
`odrive_can`:

- `heartbeat_msg_rate_ms`
- `encoder_msg_rate_ms`
- `iq_msg_rate_ms`
- `torques_msg_rate_ms`
- `error_msg_rate_ms`
- `temperature_msg_rate_ms`
- `bus_voltage_msg_rate_ms`

Without those, `/controller_status` and `/request_axis_state` will not work
reliably.

## 15. Review Robot-Specific Config Before First Launch

Before you ever run the production launch, review these files and confirm the
values match the physical robot:

### Required Review

- `src/pilot_control/launch/robot_complete.launch.py`
  - `wheel_radius`
  - `wheel_base`
  - `can_interface`
  - RGB camera by-id paths
  - `stream_host`
  - `stream_port`
  - `scan_mode`
- `src/livox_ros_driver2/config/MID360_config.json`
  - host IP
  - LiDAR IP
  - ports
- `src/pilot_control/config/fastlio_mid360.yaml`
  - `extrinsic_T`
  - `extrinsic_R`
- `src/pilot_control/config/99-arduino.rules`
- `src/pilot_control/config/99-ublox-gps.rules`

### Optional Review

- `src/pilot_control/config/manual_stereo_extrinsics.yaml`
  - only needed for certain camera calibration tooling

## 16. Optional Coverage Planner Robot Auth Provisioning

If the robot must be controlled by the Coverage Planner GUI over SSH, complete
the auth provisioning step after the workspace is built.

Expose `pilot_control_auth` on the global `PATH`:

```bash
sudo ln -sf \
  ~/pilot_ws/install/pilot_control/lib/pilot_control/pilot_control_auth \
  /usr/local/bin/pilot_control_auth

which pilot_control_auth
```

Provision the robot ID:

```bash
ROBOT_ID="<robot-id>"
echo "$ROBOT_ID" | sudo tee /etc/pilot_robot_id >/dev/null
```

Provision the auth secret:

```bash
sudo dd if=/dev/urandom of=/etc/pilot_auth_secret.key bs=32 count=1 status=none
```

Provision the six-digit PIN hash:

```bash
python3 - <<'PY' | sudo tee /etc/pilot_access_pin.hash >/dev/null
import getpass, os, base64, hashlib
pin = getpass.getpass("PIN (6 digits): ")
if len(pin) != 6 or not pin.isdigit():
    raise SystemExit("PIN must be exactly 6 digits")
iters = 200_000
salt = os.urandom(16)
dk = hashlib.pbkdf2_hmac("sha256", pin.encode(), salt, iters)
def b64(x): return base64.urlsafe_b64encode(x).decode().rstrip("=")
print(f"pbkdf2_sha256${iters}${b64(salt)}${b64(dk)}")
PY
```

Set permissions:

```bash
sudo mkdir -p /var/lib/pilot_control
sudo chown <robot-user>:<robot-user> \
  /etc/pilot_robot_id \
  /etc/pilot_access_pin.hash \
  /etc/pilot_auth_secret.key \
  /var/lib/pilot_control

sudo chmod 600 \
  /etc/pilot_robot_id \
  /etc/pilot_access_pin.hash \
  /etc/pilot_auth_secret.key
```

Verify locally:

```bash
pilot_control_auth login --robot-id "<robot-id>" --pin-stdin --json
```

Verify over SSH from the laptop:

```bash
ssh <robot-user>@<robot-ip> pilot_control_auth --help
```

## 17. Validate The Installation Before Field Use

Run these checks after the build and hardware setup are complete.

### 17.1 Library And Binary Checks

```bash
ldconfig -p | rg "livox_lidar_sdk|seekcamera|jxl"
which zenohd
ros2 pkg executables pilot_control | rg "startup_preflight|tilt_calibration|gpr_serial_bridge|odom_tilt_corrector"
gst-inspect-1.0 x264enc h264parse rtph264pay
gst-inspect-1.0 vaapih264enc || echo "VA-API encoder unavailable; software x264 path will be used"
```

### 17.2 Device Node Checks

```bash
ls -l /dev/arduino
ls -l /dev/gps
ls -l /dev/v4l/by-id
ip -details link show can0
```

### 17.3 Preflight

Run the built-in robot preflight:

```bash
source ~/.bashrc
ros2 run pilot_control startup_preflight
```

This checks:

- both RGB cameras
- thermal camera
- Livox connectivity and IMU
- RF reachability
- GPS
- left and right drive motors

Preflight reports are written under `/R_DATA/startup_check`.

### 17.4 Tilt Calibration

With the robot stationary on level ground:

```bash
ros2 run pilot_control tilt_calibration
```

This writes the latest calibration file under:

- `/R_DATA/tilt_calibration`

`robot_complete.launch.py` will auto-load the newest calibration file.

## 18. First Launch

For outdoor operation with GNSS enabled:

```bash
source ~/.bashrc
ros2 launch pilot_control robot_complete.launch.py
```

For indoor operation with GNSS disabled:

```bash
source ~/.bashrc
ros2 launch pilot_control robot_complete.launch.py scan_mode:=indoor
```

After launch, verify these topics appear:

```bash
ros2 topic list | rg "livox|Odometry|gps|controller_status|local_nav_grid"
```

Expected core topics:

- `/livox/lidar`
- `/livox/imu`
- `/Odometry`
- `/Odometry_tilt_corrected_diff`
- `/left/controller_status`
- `/right/controller_status`
- `/gpr/controller_status`
- `/local_nav_grid`
- `/gps/fix` when `scan_mode:=outdoor`

## 19. Recommended Production Boot Automation

For unattended field startup, run the launch under systemd.

Create `/etc/systemd/system/robot-complete.service`:

```ini
[Unit]
Description=ROOFUS robot production launch
After=network-online.target robot-can.service
Wants=network-online.target robot-can.service

[Service]
Type=simple
User=<robot-user>
WorkingDirectory=/home/<robot-user>/pilot_ws
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=ROS_DOMAIN_ID=0
Environment=LD_LIBRARY_PATH=/usr/local/lib
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/<robot-user>/pilot_ws/install/setup.bash && ros2 launch pilot_control robot_complete.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

If this robot should default to indoor mode, change `ExecStart` to:

```bash
ros2 launch pilot_control robot_complete.launch.py scan_mode:=indoor
```

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-complete.service
sudo systemctl start robot-complete.service
sudo systemctl status robot-complete.service
```

Follow logs:

```bash
journalctl -u robot-complete.service -f
```

Note:

- the launch already starts `zenohd`, so a separate Zenoh service is not
  required unless you intentionally split it out

## 20. Production Acceptance Checklist

The robot is production-ready only when all items below are true:

- Ubuntu 22.04 is installed and fully updated
- ROS 2 Humble is installed
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- all required repos are cloned under `~/pilot_ws/src`
- every repo is pinned to an approved tag or commit
- Livox SDK2 is installed into `/usr/local/lib`
- `libjxl` is installed system-wide under `/usr/local`
- Seek Thermal SDK is installed and detectable
- `zenohd` and the `ros2dds` plugin are installed
- Accel MPC Python dependencies resolve (`osqp` and `scipy`)
- the required robot-side GStreamer encoder elements resolve, with
  `vaapih264enc` present when Intel VA-API acceleration is expected
- workspace build succeeds for `odrive_can`, `livox_ros_driver2`, `fast_lio`,
  and `pilot_control`
- `serial` also builds if you are using the separate multi-repo layout that
  includes it
- `/R_DATA` exists and is writable
- the onboard Leonardo is flashed and `/dev/arduino` and `/dev/gps` exist
- RGB camera by-id paths match the launch configuration
- the Livox NIC is on the expected subnet
- `can0` is up at `250000` bitrate
- ODrive node IDs are `0`, `1`, and `2`
- ODrive cyclic status messages are enabled
- `startup_preflight` passes
- tilt calibration has been generated
- `robot_complete.launch.py` starts cleanly
- required robot-to-laptop comms work over RF
- optional Coverage Planner auth is provisioned if the GUI is part of the field workflow

## Quick Recovery Commands

Use these for fast sanity checks during deployment:

```bash
source ~/.bashrc
ip -details link show can0
ls -l /dev/arduino /dev/gps
lsusb | rg "8036|Arduino|Leonardo" || true
ls -l /dev/v4l/by-id
ldconfig -p | rg "livox_lidar_sdk|seekcamera|jxl"
zenohd --version
gst-inspect-1.0 x264enc h264parse rtph264pay
gst-inspect-1.0 vaapih264enc || echo "VA-API encoder unavailable; software x264 path will be used"
ros2 run pilot_control startup_preflight
ros2 run pilot_control tilt_calibration
ros2 launch pilot_control robot_complete.launch.py
```

## Optional Operator Shell Helpers

The current LattePanda also has a few shell shortcuts in `~/.bashrc`. These are
not required for robot functionality, but they are part of the present operator
workflow and are worth replicating if you want the new robot to feel the same.

Add:

```bash
cat <<'EOF' >> ~/.bashrc
alias canup='sudo ip link set can0 up type can bitrate 250000'
alias candown='sudo ip link set can0 down'
alias launchrobot='ros2 launch pilot_control robot_complete.launch.py'
piconbuild(){
  cd ~/pilot_ws
  colcon build --packages-select pilot_control
  source install/setup.bash
}
EOF
```

Then reload the shell:

```bash
source ~/.bashrc
```

## Known Hardcoded Values To Review Per Robot

Do not blindly reuse these without checking them on a fresh robot:

- camera by-id paths in `src/pilot_control/launch/robot_complete.launch.py`
- video stream host and port in `src/pilot_control/launch/robot_complete.launch.py`
- Livox host and sensor IPs in `src/livox_ros_driver2/config/MID360_config.json`
- wheel geometry in `src/pilot_control/launch/robot_complete.launch.py`
- LiDAR extrinsics in `src/pilot_control/config/fastlio_mid360.yaml`
- robot auth values in `/etc/pilot_robot_id`, `/etc/pilot_auth_secret.key`,
  and `/etc/pilot_access_pin.hash`

