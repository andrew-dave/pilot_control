# Operator Laptop Production Setup Guide

This guide provisions a fresh Ubuntu laptop to run the laptop-side operator
stack for:

- `ros2 launch pilot_control laptop_teleop.launch.py`
- `src/pilot_control/scripts/F2C/cpp/build/bdr_coverage_planner`

This guide targets the production planner path, not the standalone Python
planner at `src/pilot_control/scripts/F2C/f2c_gui.py`. The C++ planner is the
one that currently includes robot login, CSV upload, SSH/rsync transfer, and
Zenoh/DDS runtime integration.

## Scope

This guide covers:

- Ubuntu 22.04 baseline setup
- git and workspace setup for the laptop-side packages
- ROS 2 Humble, CycloneDDS, Zenoh, and build dependencies
- system-wide Fields2Cover install for full coverage-planning features
- `host_teleop` build and launch
- production coverage planner build and runtime setup
- machine-local laptop config at `~/pilot_config/laptop.yaml`
- SSH and robot registry setup for planner login and uploads
- validation and first launch

Assumptions:

- target OS is `Ubuntu 22.04 LTS`
- target ROS distro is `Humble`
- target workspace is `~/pilot_ws`
- laptop-specific teleop config lives outside git in `~/pilot_config/laptop.yaml`
- robot-side stack is already provisioned with
  `src/pilot_control/docs/lattepanda_robot_production_setup.md`
- robot auth setup from `src/pilot_control/robot_setup_instructions.txt` is
  complete on the robot

Optional guided path:

- run `src/pilot_control/install_operator_laptop_production.sh` from the repo to
  execute an interactive step-by-step installer
- the installer supports step skipping and writes a completed/skipped/failed
  summary at the end

## 1. Baseline Ubuntu Setup

Install Ubuntu 22.04, then run:

```bash
sudo apt update
sudo apt full-upgrade -y

sudo apt install -y \
  curl wget gnupg lsb-release ca-certificates software-properties-common \
  jq vim tmux htop tree unzip net-tools iproute2 iputils-ping \
  openssh-client rsync xterm

sudo timedatectl set-timezone <Region/City>
sudo timedatectl set-ntp true
```

## 2. Git, SSH, And Workspace Layout

Install git tooling:

```bash
sudo apt install -y git git-lfs
git lfs install
```

### 2.1 Configure Git Identity

Set the operator identity that will be used for local commits and tags:

```bash
git config --global user.name "<Your Name>"
git config --global user.email "<your-email@example.com>"
```

Verify:

```bash
git config --global --get user.name
git config --global --get user.email
```

### 2.2 Configure SSH Repo Access

Most production laptops should use SSH clone URLs rather than HTTPS.

Create a key if needed:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
test -f ~/.ssh/id_ed25519 || ssh-keygen -t ed25519 -C "<your-email@example.com>"
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
cat ~/.ssh/id_ed25519.pub
```

Add the printed public key to the git host account used for this workspace
before cloning.

Trust and test the git host:

```bash
touch ~/.ssh/known_hosts
chmod 644 ~/.ssh/known_hosts
ssh-keyscan -H <git-host> >> ~/.ssh/known_hosts
ssh -T git@<git-host>
```

Notes:

- replace `<git-host>` with your real git SSH host such as `github.com`,
  `gitlab.com`, or your team's internal forge
- if your forge uses a username other than `git`, use that username in the SSH
  test command
- some git hosts print a success message but still exit non-zero for `ssh -T`;
  that is acceptable if the message confirms authentication worked

Create the workspace:

```bash
mkdir -p ~/pilot_ws/src
cd ~/pilot_ws/src
```

The minimum source tree this laptop guide assumes is:

- `src/pilot_control`
- `src/ros_odrive`

### Option A: Monorepo

```bash
git clone --recurse-submodules <workspace-repo-ssh-url> ~/pilot_ws
cd ~/pilot_ws
git checkout <approved-tag-or-commit>
git submodule update --init --recursive
```

### Option B: Minimal Multi-Repo Workspace

```bash
cd ~/pilot_ws/src

git clone <pilot_control-repo-ssh-url> pilot_control
git clone <ros_odrive-repo-ssh-url> ros_odrive
```

Pin both repos to the approved release commit or tag used by your team.

## 3. Install ROS 2 Humble And Laptop Build Dependencies

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

Then install ROS, DDS, Zenoh, and the non-ROS packages needed to build the
current laptop-side stack:

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-cyclonedds \
  ros-humble-pcl-conversions \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential \
  cmake \
  pkg-config \
  libsdl2-dev \
  qtbase5-dev \
  libeigen3-dev \
  libpcl-dev \
  libcgal-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav \
  gstreamer1.0-x \
  gstreamer1.0-gl \
  zenohd \
  zenoh-plugin-ros2dds
```

Initialize `rosdep` if this is the first ROS setup on the machine:

```bash
sudo rosdep init
rosdep update
```

Use ROS in the current shell:

```bash
source /opt/ros/humble/setup.bash
```

Notes:

- If `/opt/ros/humble/setup.bash` already exists, the ROS repository bootstrap
  may already be present, but it is still safe to verify it.
- Keep the extra apt packages from this guide even if `rosdep install` later
  reports success.
- The current `pilot_control` CMake files require several build-time packages
  that are not fully represented in `package.xml`.
- The planner's embedded laptop-side video viewer uses GStreamer runtime
  elements including `rtph264depay`, `h264parse`, `avdec_h264`, and
  `autovideosink`, so the extra runtime plugin packages above are intentional
  and not just build dependencies.

## 4. Install Fields2Cover System-Wide

The production C++ coverage planner only has full path-planning functionality
when the native Fields2Cover C++ library is installed system-wide.

Install the upstream build prerequisites:

```bash
sudo apt install --no-install-recommends -y \
  software-properties-common \
  doxygen \
  g++ \
  libgdal-dev \
  libpython3-dev \
  python3 \
  python3-matplotlib \
  python3-tk \
  lcov \
  libgtest-dev \
  libtbb-dev \
  swig \
  libgeos-dev \
  gnuplot \
  libtinyxml2-dev \
  nlohmann-json3-dev

python3 -m pip install --user gcovr
```

Build and install Fields2Cover under `/usr/local`:

```bash
cd ~
git clone https://github.com/Fields2Cover/Fields2Cover.git
cd Fields2Cover
git checkout <approved-tag-or-commit>

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF ..
cmake --build . -- -j"$(nproc)"
sudo cmake --install .
sudo ldconfig
```

Verify:

```bash
ls /usr/local/include/fields2cover.h
ldconfig -p | grep -i fields2cover
```

If your team does not currently pin a Fields2Cover version, use a team-approved
commit before putting the laptop into field service.

## 5. Optional But Recommended Operator Extras

Install `open3d` for the planner's preferred 3D viewer path:

```bash
python3 -m pip install --user open3d
```

Notes:

- If `open3d` is missing, the planner falls back to `pcl_viewer` or
  `CloudCompare` if available.
- `open3d` is not required for teleop or for building the planner, but it is
  the best-supported interactive viewer path in the current UI.

If your deployment uses the `Laptop -> Cloud` upload tab, also install and
configure AWS CLI:

```bash
sudo apt install -y awscli
aws configure
aws sts get-caller-identity
```

## 6. Build The ROS Workspace

Install the remaining manifest-declared dependencies:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
```

Build the laptop-side ROS packages:

```bash
source /opt/ros/humble/setup.bash
cd ~/pilot_ws
colcon build --symlink-install --packages-select odrive_can pilot_control
```

## 7. Build The Production Coverage Planner

Build the C++ planner after the ROS workspace and Fields2Cover are available:

```bash
source /opt/ros/humble/setup.bash
source ~/pilot_ws/install/setup.bash
cd ~/pilot_ws/src/pilot_control/scripts/F2C/cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j"$(nproc)"
```

Expected CMake summary:

- `Fields2Cover: TRUE`
- `CGAL: TRUE`

If the build summary says Fields2Cover was not found, stop here and fix Step 4
before field use. The planner will otherwise launch with degraded or disabled
coverage-path features.

## 8. Set The Production Runtime Environment

The current laptop-side architecture is:

- CycloneDDS on loopback for local ROS node communication
- Zenoh for robot-to-laptop communication over the RF link

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
grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc || cat <<'EOF' >> ~/.bashrc
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

### 8.1 Create The Machine-Local Laptop Config

Keep laptop-specific teleop defaults outside git, just like the robot-side
`robot.yaml`.

Create:

```bash
mkdir -p ~/pilot_config
cp -n ~/pilot_ws/src/pilot_control/config/laptop_config.example.yaml ~/pilot_config/laptop.yaml
nano ~/pilot_config/laptop.yaml
```

Review these values at minimum:

- `teleop.robot_ip` must match the deployed robot IP you intend to reach over
  the Microhard link
- `teleop.use_xterm` controls whether `host_teleop` starts in a separate
  `xterm`
- `teleop.interactive_sdl` controls whether the SDL keyboard window is enabled
- `teleop.cmd_vel_enabled` controls whether teleop publishes `/cmd_vel`

If you intentionally store the config elsewhere:

```bash
export PILOT_LAPTOP_CONFIG=/full/path/to/laptop.yaml
```

## 9. Configure SSH For Non-Interactive Robot Access

The production coverage planner uses `ssh`, `scp`, and `rsync` in batch mode.
Password prompts are not allowed during planner login or CSV upload, so SSH key
access must already work non-interactively.

Create an operator key if needed:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
test -f ~/.ssh/id_ed25519 || ssh-keygen -t ed25519 -C "<laptop-label>"
chmod 600 ~/.ssh/id_ed25519
chmod 644 ~/.ssh/id_ed25519.pub
```

Install the key on the robot and verify batch-mode login:

```bash
ssh-copy-id <robot-user>@<robot-ip>
ssh -o BatchMode=yes <robot-user>@<robot-ip> true
```

Recommended for production: capture the robot host key now:

```bash
ssh-keyscan -t ed25519 <robot-ip>
```

You can paste that full line into the `known_hosts_entry` field in the robot
registry in Step 10 if you want the planner to pin a specific SSH host key.

## 10. Create The Planner Robot Registry

The planner looks for a user-level registry first, so do not edit the checked-in
repo file unless you intentionally want a shared default.

Create:

```bash
mkdir -p ~/.config/PilotControl/BDRCoveragePlanner

cat <<'EOF' > ~/.config/PilotControl/BDRCoveragePlanner/robots.json
{
  "robots": [
    {
      "robot_id": "Roofus#001",
      "host": "192.168.168.101",
      "ssh_user": "roofus",
      "robot_data_path": "/R_DATA",
      "default_remote_upload_dir": "/R_DATA/waypoints",
      "known_hosts_entry": ""
    }
  ]
}
EOF
```

Review these values at minimum:

- `robot_id` must match `/etc/pilot_robot_id` on the robot exactly
- `host` must match `teleop.robot_ip` in `~/pilot_config/laptop.yaml`
- `ssh_user` must match the robot login user
- `robot_data_path` should usually remain `/R_DATA`
- `default_remote_upload_dir` should usually remain `/R_DATA/waypoints`
- `known_hosts_entry` may be left empty, but production systems should prefer a
  pinned key from `ssh-keyscan`

## 11. Robot-Side Prerequisites For Full Planner Functionality

This is a laptop guide, but the planner's login and upload features still rely
on robot-side setup being complete.

Before field use, verify all of the following on the robot:

- the robot is provisioned per
  `src/pilot_control/docs/lattepanda_robot_production_setup.md`
- `pilot_control_auth` is callable over non-interactive SSH
- the robot user can write to `/R_DATA/waypoints`
- the deployed robot ID matches the registry entry from Step 10

Recommended verification from the laptop:

```bash
ssh -o BatchMode=yes <robot-user>@<robot-ip> pilot_control_auth --help
ssh -o BatchMode=yes <robot-user>@<robot-ip> mkdir -p /R_DATA/waypoints
```

If `pilot_control_auth` is not found over SSH, finish the robot-side auth steps
in `src/pilot_control/robot_setup_instructions.txt` before using the planner.

## 12. Validation

First verify the local install:

```bash
source ~/.bashrc
ros2 interface show odrive_can/srv/AxisState
ros2 pkg prefix pilot_control
test -x ~/pilot_ws/src/pilot_control/scripts/F2C/cpp/build/bdr_coverage_planner
zenohd --version
gst-inspect-1.0 rtph264depay h264parse avdec_h264 videoconvert autovideosink
```

### 12.1 Launch Laptop Teleop

Start the laptop bridge and SDL teleop window:

```bash
source ~/.bashrc
ros2 launch pilot_control laptop_teleop.launch.py
```

For a one-off override without editing `~/pilot_config/laptop.yaml`, append a
launch argument such as `robot_ip:=192.168.168.101`.

Expected:

- `zenohd` starts in client mode
- `host_teleop` launches cleanly
- the SDL teleop window opens in `xterm`

In another terminal, verify the local heartbeat:

```bash
source ~/.bashrc
ros2 topic echo /host_teleop/heartbeat --once
```

### 12.2 Launch The Coverage Planner

Start the production planner:

```bash
source ~/.bashrc
~/pilot_ws/src/pilot_control/scripts/F2C/cpp/build/bdr_coverage_planner
```

Expected initial behavior:

- the configured robot registry loads without error
- DDS status shows loopback/CycloneDDS healthy
- Zenoh status turns healthy after `laptop_teleop.launch.py` is already running

Expected production behavior after login:

- robot login succeeds with the configured `robot_id` and 6-digit access code
- mission CSV upload succeeds
- data transfer can reach the robot over SSH
- the 3D viewer opens if `open3d` is installed

If your deployment uses cloud upload, also verify:

```bash
aws sts get-caller-identity
```

## 13. Production Acceptance Checklist

The laptop is ready only when all items below are true:

- Ubuntu 22.04 is fully updated
- git `user.name` and `user.email` are configured
- SSH key-based access to the workspace git host works
- the workspace contains `pilot_control` and `odrive_can`
- `odrive_can` and `pilot_control` build successfully
- Fields2Cover is installed system-wide under `/usr/local`
- the planner build reports both Fields2Cover and CGAL found
- `zenohd` and `zenoh-plugin-ros2dds` are installed
- the required laptop-side GStreamer runtime elements resolve via
  `gst-inspect-1.0`
- `~/cyclone_loopback.xml` exists
- `~/pilot_config/laptop.yaml` matches the deployed robot and preferred teleop
  behavior
- `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, and `CYCLONEDDS_URI` are correct
- SSH batch-mode access to the robot succeeds without password prompts
- `~/.config/PilotControl/BDRCoveragePlanner/robots.json` matches the deployed
  robot
- `ros2 launch pilot_control laptop_teleop.launch.py` starts cleanly
- `bdr_coverage_planner` starts cleanly
- planner login and waypoint CSV upload succeed
- `awscli` is configured if the cloud-upload workflow is used
