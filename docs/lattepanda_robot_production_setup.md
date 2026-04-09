# LattePanda Robot Production Setup Guide

This guide provisions a fresh LattePanda to run the robot-side stack from:

- `src/pilot_control/launch/robot_complete.launch.py`

If this robot was restored from a previously captured image instead of a clean
install, do the post-clone uniqueness steps in
`lattepanda_post_clone_setup.md` before reconnecting it to the normal network.

## Scope

This guide covers:

- Ubuntu 22.04 baseline setup
- git and workspace setup
- ROS 2 Humble and required system packages
- Livox, Seek, JPEG XL, udev, CAN, and storage setup
- machine-local robot config at `~/pilot_config/robot.yaml`
- runtime environment and CPU isolation
- Livox networking
- validation and first launch

Assumptions:

- target OS is `Ubuntu 22.04 LTS`
- target ROS distro is `Humble`
- target workspace is `~/pilot_ws`
- target data mount is `/R_DATA`
- robot-specific config lives outside git in `~/pilot_config/robot.yaml`

## 1. Baseline Ubuntu Setup

Install Ubuntu 22.04, then run:

```bash
sudo apt update
sudo apt full-upgrade -y

sudo apt install -y \
  openssh-server curl wget gnupg lsb-release ca-certificates \
  software-properties-common jq vim tmux htop tree unzip \
  net-tools iproute2 iputils-ping usbutils pciutils

sudo systemctl enable --now ssh
sudo hostnamectl set-hostname <robot-hostname>
sudo timedatectl set-timezone <Region/City>
sudo timedatectl set-ntp true
```

Recommended on a field robot:

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

## 2. Git And Workspace Layout

Install git tooling:

```bash
sudo apt install -y git git-lfs
git lfs install
```

If needed, generate a repo access key:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
ssh-keygen -t ed25519 -C "<robot-label-or-email>"
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
cat ~/.ssh/id_ed25519.pub
```

Create the workspace:

```bash
mkdir -p ~/pilot_ws/src
cd ~/pilot_ws/src
```

### Option A: Monorepo

```bash
git clone --recurse-submodules <workspace-repo-url> ~/pilot_ws
cd ~/pilot_ws
git checkout <approved-tag-or-commit>
git submodule update --init --recursive
```

If this monorepo intentionally omits packages such as `serial` or
`pointcloud_to_grid`, that is expected. Use the monorepo-specific build command
later in this guide.

### Option B: Multi-Repo Workspace

Clone the required repos into `~/pilot_ws/src` so the final tree contains at
least:

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
```

Pin every repo to the approved release commit or tag used by your team.

## 3. Install ROS 2 Humble And System Dependencies

Install ROS 2 Humble and the non-ROS packages used by the current robot-side
stack:

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cyclonedds \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-dev \
  python3-numpy \
  python3-opencv \
  python3-serial \
  python3-yaml \
  build-essential \
  cmake \
  pkg-config \
  can-utils \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libapr1-dev \
  libbrotli-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  zenohd \
  zenoh-plugin-ros2dds
```

Initialize rosdep if this is the first ROS setup on the machine:

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

- `libjxl-dev` is not reliably available on Ubuntu 22.04 in the way this robot
  stack needs.
- Install `libbrotli-dev` above, then build `libjxl` from source in Step 5.

## 4. Install Seek Thermal SDK

The current workspace expects the Seek SDK to be installed system-wide if the
thermal tooling is used.

If you have a vendor `.deb`, install it:

```bash
cd ~/pilot_ws
sudo apt install ./src/pilot_control/seekthermal-sdk-dev-<version>_amd64.deb
```

Or install from your private vendor location if that is where the team stores
it.

Verify:

```bash
dpkg -s seekthermal-sdk-dev
ls /usr/include/seekcamera
ls /lib/libseekcamera* /usr/lib/libseekcamera* 2>/dev/null
```

## 5. Build And Install JPEG XL System-Wide

The current `pilot_control` prefers a system-wide `libjxl` install under
`/usr/local`.

```bash
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
ldconfig -p | grep -E 'libjxl|brotli'
```

Notes:

- Do not run `./deps.sh` if you already cloned normally and initialized
  submodules.
- `./deps.sh` is only for source trees that do not already contain the required
  submodules.

## 6. Create The Machine-Local Robot Config

Copy the example config outside the repo:

```bash
mkdir -p ~/pilot_config
cp -n ~/pilot_ws/src/pilot_control/config/robot_config.example.yaml ~/pilot_config/robot.yaml
```

Edit:

```bash
nano ~/pilot_config/robot.yaml
```

Review these values at minimum:

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
- `arduino.serial_port`
- `preflight.rf_target_ip`

If you intentionally store the config elsewhere:

```bash
export PILOT_ROBOT_CONFIG=/full/path/to/robot.yaml
```

## 7. Build The Workspace

Install remaining package dependencies:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

With the current `pilot_control/package.xml`, `rosdep` should resolve cleanly.
If it still complains about `opencv4`, your checkout is older than the current
guide. Update the manifest or temporarily use `--skip-keys opencv4` after
installing `libopencv-dev` and `python3-opencv`.

### Monorepo Build

Use this when `serial` is intentionally absent:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select \
  odrive_can livox_ros_driver2 fast_lio pilot_control
```

### Multi-Repo Build

Use this when `serial` exists in `~/pilot_ws/src`:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select \
  serial odrive_can livox_ros_driver2 fast_lio pilot_control
```

If `livox_ros_driver2` fails with a CMake error mentioning
`LIVOX_INTERFACES_INCLUDE_DIRECTORIES`, the checkout has the older Humble build
logic. Update `src/livox_ros_driver2/CMakeLists.txt` to the current revision.
Temporary workaround only:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select livox_ros_driver2 \
  --cmake-args -DHUMBLE_ROS=humble
```

## 8. Set The Production Runtime Environment

The current LattePanda keeps CycloneDDS on loopback only. Cross-network
robot-to-laptop communication is handled by Zenoh rather than raw DDS
multicast.

Create the local CycloneDDS config:

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

Append the runtime exports to `~/.bashrc`:

```bash
cat <<'EOF' >> ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///home/$USER/cyclone_loopback.xml
source /opt/ros/humble/setup.bash
source ~/pilot_ws/install/setup.bash
EOF
```

Reload and verify:

```bash
source ~/.bashrc
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DOMAIN_ID"
echo "$CYCLONEDDS_URI"
```

Expected:

- `rmw_cyclonedds_cpp`
- `0`
- `file:///home/$USER/cyclone_loopback.xml`

### 8.1 Match The Current CPU Isolation Setup

The current LattePanda reserves CPU cores `0-1` at boot, then pins important
LiDAR and SLAM nodes in `robot_complete.launch.py`.

Set:

```bash
sudo sed -i 's/^GRUB_CMDLINE_LINUX_DEFAULT=.*/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=0-1 nohz_full=0-1 rcu_nocbs=0-1"/' /etc/default/grub
sudo update-grub
sudo reboot
```

After reboot:

```bash
cat /proc/cmdline
```

Expected entries:

- `isolcpus=0-1`
- `nohz_full=0-1`
- `rcu_nocbs=0-1`

### 8.2 Optional Operator Shell Helpers

These are convenience helpers from the current LP:

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

## 9. Install Udev Rules And Permissions

```bash
sudo cp ~/pilot_ws/src/pilot_control/config/99-arduino.rules /etc/udev/rules.d/
sudo cp ~/pilot_ws/src/pilot_control/config/99-ublox-gps.rules /etc/udev/rules.d/

sudo usermod -aG dialout,video "$USER"

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in after changing group membership.

Verify:

```bash
ls -l /dev/arduino
ls -l /dev/gps
ls -l /dev/v4l/by-id
```

## 10. Create Or Verify `/R_DATA`

If the data SSD already exists, verify it:

```bash
lsblk -f
df -h /R_DATA
stat -c '%n %F %U:%G %a' /R_DATA
```

If you are provisioning a new data disk, format and mount it as ext4, then add
it to `/etc/fstab` with its UUID. Example mountpoint:

```bash
sudo mkdir -p /R_DATA
sudo chown "$USER:$USER" /R_DATA
```

Create the expected subdirectories:

```bash
mkdir -p /R_DATA/tilt_calibration
mkdir -p /R_DATA/startup_check
mkdir -p /R_DATA/unified_scans
mkdir -p /R_DATA/gpr_scans
mkdir -p /R_DATA/raw_maps
```

## 11. Configure The Livox Ethernet Interface

Recommended production setup: use a single floating NetworkManager profile for
Livox, keep it off the default route, and let it autoconnect on whichever
Ethernet port currently has the sensor attached.

If an older port-specific Livox profile exists and you no longer want it,
remove it first:

```bash
nmcli connection show
sudo nmcli connection down "<old-livox-profile>" || true
sudo nmcli connection delete "<old-livox-profile>"
```

Create the portable profile using either currently available Ethernet port for
the initial creation step, then clear the binding:

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

Disable competing generic wired profiles so they do not steal the port:

```bash
nmcli connection show
sudo nmcli connection modify "Wired connection 1" connection.autoconnect no
sudo nmcli connection modify "Wired connection 2" connection.autoconnect no
```

Bring it up once:

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
- do not assign the same Livox host IP to both Ethernet ports simultaneously
- true Livox product detection is not a normal NetworkManager feature on
  Ethernet

If you choose a different IP plan, update
`src/livox_ros_driver2/config/MID360_config.json` before launch.

## 12. Validation

Before first launch:

```bash
source ~/.bashrc
ldconfig -p | grep -E 'livox|seekcamera|jxl'
ip -details link show can0
ros2 run pilot_control startup_preflight
ros2 run pilot_control tilt_calibration
```

Robot launch:

```bash
ros2 launch pilot_control robot_complete.launch.py
```

Indoor scan mode if required:

```bash
ros2 launch pilot_control robot_complete.launch.py scan_mode:=indoor
```

## 13. Optional Boot Automation

If your deployment uses systemd for automatic bring-up, verify or create the
robot service units that:

- bring up `can0`
- source the ROS environment
- launch `robot_complete.launch.py`

Keep them disabled until the first manual validation passes.

## 14. Production Acceptance Checklist

The robot is ready only when all items below are true:

- every repo is pinned to the approved commit or tag
- Seek SDK is installed and detectable
- `libjxl` is installed system-wide under `/usr/local`
- `~/pilot_config/robot.yaml` matches the physical robot
- `/R_DATA` is mounted and writable
- the Livox NIC reaches `192.168.1.127`
- `/dev/arduino` and `/dev/gps` are correct
- camera by-id paths are correct
- the workspace build succeeds for `odrive_can`, `livox_ros_driver2`,
  `fast_lio`, and `pilot_control`
- `serial` also builds if you are using the separate multi-repo layout that
  includes it
- `startup_preflight` passes
- `tilt_calibration` passes
- `robot_complete.launch.py` starts cleanly
