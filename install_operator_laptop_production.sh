#!/usr/bin/env bash

set -u
set -o pipefail

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "This installer must be run with bash."
  echo "Example: bash install_operator_laptop_production.sh"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WORKSPACE_DIR="${HOME}/pilot_ws"
DEFAULT_FIELDS2COVER_DIR="${HOME}/Fields2Cover"
DEFAULT_REGISTRY_PATH="${HOME}/.config/PilotControl/BDRCoveragePlanner/robots.json"
DEFAULT_LAPTOP_CONFIG_PATH="${HOME}/pilot_config/laptop.yaml"
LOG_DIR="${HOME}/operator_laptop_setup_logs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/operator_laptop_setup_${TIMESTAMP}.log"
SUMMARY_FILE="${LOG_DIR}/operator_laptop_setup_${TIMESTAMP}_summary.txt"

WORKSPACE_DIR="${DEFAULT_WORKSPACE_DIR}"
FIELDS2COVER_DIR="${DEFAULT_FIELDS2COVER_DIR}"
GIT_HOST="github.com"
ROBOT_ID="Roofus#001"
ROBOT_HOST="192.168.168.101"
ROBOT_SSH_USER="roofus"
ROBOT_DATA_PATH="/R_DATA"
ROBOT_UPLOAD_DIR="/R_DATA/waypoints"
ROBOT_KNOWN_HOSTS_ENTRY=""

declare -a COMPLETED_STEPS=()
declare -a SKIPPED_STEPS=()
declare -a FAILED_STEPS=()
STEP_ERROR_MESSAGE=""
SUMMARY_WRITTEN=0

mkdir -p "${LOG_DIR}"
exec > >(tee -a "${LOG_FILE}") 2>&1

log() {
  printf '%s\n' "$*"
}

fail_step() {
  STEP_ERROR_MESSAGE="$1"
  log "[ERROR] ${STEP_ERROR_MESSAGE}"
  return 1
}

expand_path() {
  local path="$1"
  if [[ "${path}" == "~" ]]; then
    printf '%s\n' "${HOME}"
  elif [[ "${path}" == ~/* ]]; then
    printf '%s\n' "${HOME}/${path#~/}"
  else
    printf '%s\n' "${path}"
  fi
}

prompt_with_default() {
  local __var_name="$1"
  local prompt="$2"
  local default_value="${3:-}"
  local reply=""

  if [[ -n "${default_value}" ]]; then
    read -r -p "${prompt} [${default_value}]: " reply
    if [[ -z "${reply}" ]]; then
      reply="${default_value}"
    fi
  else
    read -r -p "${prompt}: " reply
  fi

  printf -v "${__var_name}" '%s' "${reply}"
}

ask_yes_no() {
  local prompt="$1"
  local default_answer="${2:-Y}"
  local reply=""
  local suffix="[Y/n]"

  if [[ "${default_answer}" =~ ^[Nn]$ ]]; then
    suffix="[y/N]"
  fi

  read -r -p "${prompt} ${suffix} " reply
  if [[ -z "${reply}" ]]; then
    reply="${default_answer}"
  fi

  [[ "${reply}" =~ ^[Yy]$ ]]
}

append_block_if_missing() {
  local file_path="$1"
  local marker="$2"
  local block_content="$3"

  touch "${file_path}"
  if ! grep -Fq "${marker}" "${file_path}"; then
    {
      printf '\n%s\n' "${block_content}"
    } >> "${file_path}"
  fi
}

write_summary() {
  if [[ "${SUMMARY_WRITTEN}" -eq 1 ]]; then
    return
  fi
  SUMMARY_WRITTEN=1

  {
    echo "Operator laptop setup summary"
    echo "Generated: $(date -Is)"
    echo "Log file: ${LOG_FILE}"
    echo

    echo "Completed steps:"
    if [[ "${#COMPLETED_STEPS[@]}" -eq 0 ]]; then
      echo "  - none"
    else
      for item in "${COMPLETED_STEPS[@]}"; do
        echo "  - ${item}"
      done
    fi
    echo

    echo "Skipped steps:"
    if [[ "${#SKIPPED_STEPS[@]}" -eq 0 ]]; then
      echo "  - none"
    else
      for item in "${SKIPPED_STEPS[@]}"; do
        echo "  - ${item}"
      done
    fi
    echo

    echo "Failed steps:"
    if [[ "${#FAILED_STEPS[@]}" -eq 0 ]]; then
      echo "  - none"
    else
      for item in "${FAILED_STEPS[@]}"; do
        echo "  - ${item}"
      done
    fi
  } > "${SUMMARY_FILE}"

  log
  log "Summary saved to: ${SUMMARY_FILE}"
  cat "${SUMMARY_FILE}"
}

trap write_summary EXIT

run_step() {
  local step_name="$1"
  local step_fn="$2"
  local choice=""

  STEP_ERROR_MESSAGE=""
  log
  log "================================================================"
  log "${step_name}"
  log "================================================================"

  while true; do
    read -r -p "[Enter]=run, s=skip, q=quit: " choice
    case "${choice,,}" in
      ""|r|run)
        break
        ;;
      s|skip)
        SKIPPED_STEPS+=("${step_name}")
        log "[SKIP] ${step_name}"
        return 0
        ;;
      q|quit)
        log "Exiting at user request."
        exit 0
        ;;
      *)
        log "Please choose run, skip, or quit."
        ;;
    esac
  done

  if "${step_fn}"; then
    COMPLETED_STEPS+=("${step_name}")
    log "[OK] ${step_name}"
    return 0
  fi

  if [[ -n "${STEP_ERROR_MESSAGE}" ]]; then
    FAILED_STEPS+=("${step_name} - ${STEP_ERROR_MESSAGE}")
  else
    FAILED_STEPS+=("${step_name}")
  fi

  log "[FAIL] ${step_name}"
  if ask_yes_no "Continue to the next step?" Y; then
    return 1
  fi

  exit 1
}

check_platform() {
  if [[ ! -r /etc/os-release ]]; then
    fail_step "Could not read /etc/os-release."
    return 1
  fi

  # shellcheck disable=SC1091
  . /etc/os-release
  log "Detected OS: ${PRETTY_NAME:-unknown}"

  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    log "This installer targets Ubuntu 22.04."
    if ! ask_yes_no "Continue anyway?" N; then
      fail_step "Unsupported operating system for this guide."
      return 1
    fi
  fi

  return 0
}

ros_humble_present() {
  [[ -f /opt/ros/humble/setup.bash ]] && return 0
  dpkg -s ros-humble-desktop >/dev/null 2>&1 && return 0
  dpkg -s ros-humble-ros-base >/dev/null 2>&1 && return 0
  return 1
}

is_universe_enabled() {
  if [[ -f /etc/apt/sources.list ]] && grep -Eq '^[[:space:]]*deb(-src)?[[:space:]].*(archive|security)\.ubuntu\.com/ubuntu.*\buniverse\b' /etc/apt/sources.list; then
    return 0
  fi

  if compgen -G "/etc/apt/sources.list.d/*.list" >/dev/null; then
    if grep -Eq '^[[:space:]]*deb(-src)?[[:space:]].*(archive|security)\.ubuntu\.com/ubuntu.*\buniverse\b' /etc/apt/sources.list.d/*.list 2>/dev/null; then
      return 0
    fi
  fi

  if compgen -G "/etc/apt/sources.list.d/*.sources" >/dev/null; then
    if grep -Eq '^Components:.*\buniverse\b' /etc/apt/sources.list.d/*.sources 2>/dev/null; then
      return 0
    fi
  fi

  return 1
}

has_custom_main_sources_list_entries() {
  [[ -f /etc/apt/sources.list ]] || return 1

  grep -E '^[[:space:]]*deb' /etc/apt/sources.list | \
    grep -vE '(archive\.ubuntu\.com/ubuntu|security\.ubuntu\.com/ubuntu|us\.archive\.ubuntu\.com/ubuntu|deb cdrom:)' \
    >/dev/null 2>&1
}

list_existing_ros_source_files() {
  [[ -f /etc/apt/sources.list.d/ros2.list ]] && printf '%s\n' "/etc/apt/sources.list.d/ros2.list"
  [[ -f /etc/apt/sources.list.d/ros2.sources ]] && printf '%s\n' "/etc/apt/sources.list.d/ros2.sources"
}

zenoh_packages_present() {
  dpkg -s zenohd >/dev/null 2>&1 && return 0
  dpkg -s zenoh-plugin-ros2dds >/dev/null 2>&1 && return 0
  return 1
}

list_existing_zenoh_source_hits() {
  if [[ -f /etc/apt/sources.list ]]; then
    grep -nH 'download\.eclipse\.org/zenoh/debian-repo' /etc/apt/sources.list 2>/dev/null || true
  fi

  if compgen -G "/etc/apt/sources.list.d/*.list" >/dev/null; then
    grep -nH 'download\.eclipse\.org/zenoh/debian-repo' /etc/apt/sources.list.d/*.list 2>/dev/null || true
  fi

  if compgen -G "/etc/apt/sources.list.d/*.sources" >/dev/null; then
    grep -nH 'download\.eclipse\.org/zenoh/debian-repo' /etc/apt/sources.list.d/*.sources 2>/dev/null || true
  fi
}

step_baseline_system() {
  local current_hostname
  local desired_hostname
  local current_timezone
  local desired_timezone

  sudo -v || { fail_step "sudo authentication failed."; return 1; }

  sudo apt update || { fail_step "apt update failed."; return 1; }
  sudo apt full-upgrade -y || { fail_step "apt full-upgrade failed."; return 1; }
  sudo apt install -y \
    curl wget gnupg lsb-release ca-certificates software-properties-common \
    jq vim tmux htop tree unzip net-tools iproute2 iputils-ping \
    openssh-client rsync xterm locales || { fail_step "Baseline package installation failed."; return 1; }

  if ask_yes_no "Set the laptop hostname now?" N; then
    current_hostname="$(hostnamectl --static 2>/dev/null || hostname)"
    prompt_with_default desired_hostname "Hostname" "${current_hostname}"
    if [[ -n "${desired_hostname}" && "${desired_hostname}" != "${current_hostname}" ]]; then
      sudo hostnamectl set-hostname "${desired_hostname}" || { fail_step "Failed to set hostname."; return 1; }
    fi
  fi

  if ask_yes_no "Set the timezone now?" Y; then
    current_timezone="$(timedatectl show -p Timezone --value 2>/dev/null || echo UTC)"
    prompt_with_default desired_timezone "Timezone" "${current_timezone}"
    if [[ -n "${desired_timezone}" ]]; then
      sudo timedatectl set-timezone "${desired_timezone}" || { fail_step "Failed to set timezone."; return 1; }
    fi
  fi

  sudo timedatectl set-ntp true || { fail_step "Failed to enable NTP."; return 1; }
  return 0
}

step_ros2_apt_bootstrap() {
  local ros_apt_source_version
  local ubuntu_codename
  local ros_source_files=()

  while IFS= read -r path; do
    [[ -n "${path}" ]] && ros_source_files+=("${path}")
  done < <(list_existing_ros_source_files)

  if ros_humble_present || [[ "${#ros_source_files[@]}" -gt 0 ]]; then
    log "Existing ROS setup detected on this machine."
    if ros_humble_present; then
      log "Detected ROS 2 Humble at /opt/ros/humble or via installed packages."
    fi
    if [[ "${#ros_source_files[@]}" -gt 0 ]]; then
      log "Detected ROS apt source files:"
      for path in "${ros_source_files[@]}"; do
        log "  - ${path}"
      done
    fi

    if [[ "${#ros_source_files[@]}" -gt 1 ]]; then
      fail_step "Multiple ROS apt source definitions exist already. Remove or consolidate them before rerunning ROS bootstrap."
      return 1
    fi

    if ask_yes_no "Skip ROS 2 apt bootstrap on this machine?" Y; then
      return 0
    fi

    if [[ "${#ros_source_files[@]}" -gt 0 ]]; then
      fail_step "Refusing to add ros2-apt-source on top of an existing ROS apt source definition."
      return 1
    fi
  fi

  sudo apt update || { fail_step "apt update failed before ROS bootstrap."; return 1; }
  sudo apt install -y locales software-properties-common curl || { fail_step "Failed to install ROS bootstrap prerequisites."; return 1; }
  sudo locale-gen en_US en_US.UTF-8 || { fail_step "locale-gen failed."; return 1; }
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || { fail_step "update-locale failed."; return 1; }
  export LANG=en_US.UTF-8

  if is_universe_enabled; then
    log "Universe repository already appears to be enabled."
  else
    if has_custom_main_sources_list_entries; then
      fail_step "Universe is not enabled, but /etc/apt/sources.list contains custom non-Ubuntu entries. Automatic add-apt-repository would be unsafe; enable Universe manually first."
      return 1
    fi

    sudo add-apt-repository universe -y || { fail_step "Failed to enable the universe repository."; return 1; }
  fi

  if dpkg -s ros2-apt-source >/dev/null 2>&1; then
    log "ros2-apt-source is already installed."
  else
    ros_apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    if [[ -z "${ros_apt_source_version}" ]]; then
      fail_step "Could not resolve the latest ros2-apt-source version."
      return 1
    fi

    ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
    curl -fL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb" || {
      fail_step "Failed to download ros2-apt-source."
      return 1
    }

    sudo dpkg -i /tmp/ros2-apt-source.deb || { fail_step "Failed to install ros2-apt-source."; return 1; }
  fi

  sudo apt update || { fail_step "apt update failed after ROS bootstrap."; return 1; }
  sudo apt upgrade -y || { fail_step "apt upgrade failed after ROS bootstrap."; return 1; }
  return 0
}

step_zenoh_apt_bootstrap() {
  local zenoh_hits=()
  local hit=""
  local keyring_path="/etc/apt/keyrings/zenoh-public-key.gpg"
  local source_list_path="/etc/apt/sources.list.d/zenoh.list"

  while IFS= read -r hit; do
    [[ -n "${hit}" ]] && zenoh_hits+=("${hit}")
  done < <(list_existing_zenoh_source_hits)

  if zenoh_packages_present || [[ "${#zenoh_hits[@]}" -gt 0 ]]; then
    log "Existing Zenoh setup detected on this machine."
    if zenoh_packages_present; then
      log "Detected installed Zenoh packages."
    fi
    if [[ "${#zenoh_hits[@]}" -gt 0 ]]; then
      log "Detected Zenoh apt source entries:"
      for hit in "${zenoh_hits[@]}"; do
        log "  - ${hit}"
      done
    fi

    if printf '%s\n' "${zenoh_hits[@]}" | grep -q '^/etc/apt/sources\.list:'; then
      fail_step "Zenoh is configured directly in /etc/apt/sources.list. Move it to /etc/apt/sources.list.d/zenoh.list before rerunning this step."
      return 1
    fi

    if ask_yes_no "Skip Zenoh apt bootstrap on this machine?" Y; then
      return 0
    fi
  fi

  sudo install -d -m 0755 /etc/apt/keyrings || { fail_step "Failed to create /etc/apt/keyrings."; return 1; }
  curl -fL https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output "${keyring_path}" || {
    fail_step "Failed to install the Zenoh apt signing key."
    return 1
  }

  echo "deb [signed-by=${keyring_path}] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee "${source_list_path}" >/dev/null || {
    fail_step "Failed to write ${source_list_path}."
    return 1
  }

  sudo apt update || { fail_step "apt update failed after Zenoh repository setup."; return 1; }
  return 0
}

step_install_ros_and_build_deps() {
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
    zenoh-plugin-ros2dds || { fail_step "Failed to install ROS 2 and laptop-side build dependencies."; return 1; }

  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    fail_step "ROS 2 Humble was installed, but /opt/ros/humble/setup.bash is missing."
    return 1
  fi

  return 0
}

step_git_identity() {
  local git_name
  local git_email
  local current_name
  local current_email

  sudo apt install -y git git-lfs || { fail_step "Failed to install git or git-lfs."; return 1; }
  git lfs install || { fail_step "git lfs install failed."; return 1; }

  current_name="$(git config --global --get user.name || true)"
  current_email="$(git config --global --get user.email || true)"

  prompt_with_default git_name "Git user.name" "${current_name}"
  prompt_with_default git_email "Git user.email" "${current_email}"

  if [[ -n "${git_name}" ]]; then
    git config --global user.name "${git_name}" || { fail_step "Failed to set git user.name."; return 1; }
  fi

  if [[ -n "${git_email}" ]]; then
    git config --global user.email "${git_email}" || { fail_step "Failed to set git user.email."; return 1; }
  fi

  if [[ -z "$(git config --global --get user.name || true)" || -z "$(git config --global --get user.email || true)" ]]; then
    fail_step "Git identity is still incomplete."
    return 1
  fi

  return 0
}

step_git_ssh_access() {
  local git_email
  local key_path
  local public_key_path
  local ssh_output
  local ssh_rc

  git_email="$(git config --global --get user.email || true)"
  prompt_with_default GIT_HOST "Git SSH host" "${GIT_HOST}"
  prompt_with_default key_path "SSH private key path" "~/.ssh/id_ed25519"
  key_path="$(expand_path "${key_path}")"
  public_key_path="${key_path}.pub"

  mkdir -p "${HOME}/.ssh" || { fail_step "Failed to create ~/.ssh."; return 1; }
  chmod 700 "${HOME}/.ssh" || { fail_step "Failed to set ~/.ssh permissions."; return 1; }

  if [[ ! -f "${key_path}" ]]; then
    ssh-keygen -t ed25519 -C "${git_email:-operator@local}" -f "${key_path}" || { fail_step "Failed to generate the SSH key."; return 1; }
  else
    log "Reusing existing SSH key at ${key_path}"
  fi

  chmod 600 "${key_path}" || { fail_step "Failed to set SSH private key permissions."; return 1; }
  chmod 644 "${public_key_path}" || { fail_step "Failed to set SSH public key permissions."; return 1; }
  touch "${HOME}/.ssh/known_hosts" || { fail_step "Failed to create ~/.ssh/known_hosts."; return 1; }
  chmod 644 "${HOME}/.ssh/known_hosts" || { fail_step "Failed to set ~/.ssh/known_hosts permissions."; return 1; }

  if ! ssh-keygen -F "${GIT_HOST}" -f "${HOME}/.ssh/known_hosts" >/dev/null 2>&1; then
    ssh-keyscan -H "${GIT_HOST}" >> "${HOME}/.ssh/known_hosts" 2>/dev/null || { fail_step "Failed to add the git host to known_hosts."; return 1; }
  fi

  log
  log "Add this public key to your git hosting account before continuing:"
  log "----------------------------------------------------------------"
  cat "${public_key_path}" || { fail_step "Could not read the SSH public key."; return 1; }
  log "----------------------------------------------------------------"

  if ! ask_yes_no "Have you added this public key to ${GIT_HOST}?" Y; then
    fail_step "Public key was not yet added to the git host."
    return 1
  fi

  ssh_output="$(ssh -T "git@${GIT_HOST}" 2>&1)"
  ssh_rc=$?
  log "${ssh_output}"

  if [[ ${ssh_rc} -eq 0 ]] || grep -Eqi 'authenticated|welcome to gitlab|successfully authenticated|logged in' <<<"${ssh_output}"; then
    return 0
  fi

  if ask_yes_no "The SSH test did not auto-validate. Did authentication still succeed?" N; then
    return 0
  fi

  fail_step "Git SSH authentication could not be confirmed."
  return 1
}

step_workspace_setup() {
  local mode_choice
  local repo_url
  local ref_name
  local pilot_control_url
  local ros_odrive_url

  prompt_with_default WORKSPACE_DIR "Workspace directory" "${WORKSPACE_DIR}"
  WORKSPACE_DIR="$(expand_path "${WORKSPACE_DIR}")"

  log
  log "Workspace setup modes:"
  log "  1) Use an existing workspace"
  log "  2) Clone a monorepo into ${WORKSPACE_DIR}"
  log "  3) Clone a minimal multi-repo workspace into ${WORKSPACE_DIR}/src"
  prompt_with_default mode_choice "Choose workspace mode" "1"

  case "${mode_choice}" in
    1)
      mkdir -p "${WORKSPACE_DIR}" || { fail_step "Failed to create the workspace directory."; return 1; }
      ;;
    2)
      if [[ -e "${WORKSPACE_DIR}" && -n "$(ls -A "${WORKSPACE_DIR}" 2>/dev/null)" ]]; then
        fail_step "Target workspace directory is not empty: ${WORKSPACE_DIR}"
        return 1
      fi
      prompt_with_default repo_url "Monorepo SSH URL"
      [[ -n "${repo_url}" ]] || { fail_step "Monorepo SSH URL is required."; return 1; }
      git clone --recurse-submodules "${repo_url}" "${WORKSPACE_DIR}" || { fail_step "Monorepo clone failed."; return 1; }
      prompt_with_default ref_name "Approved monorepo tag/commit (leave blank to keep current checkout)" ""
      if [[ -n "${ref_name}" ]]; then
        git -C "${WORKSPACE_DIR}" checkout "${ref_name}" || { fail_step "Failed to checkout the requested monorepo ref."; return 1; }
      fi
      git -C "${WORKSPACE_DIR}" submodule update --init --recursive || { fail_step "git submodule update failed."; return 1; }
      ;;
    3)
      mkdir -p "${WORKSPACE_DIR}/src" || { fail_step "Failed to create ${WORKSPACE_DIR}/src."; return 1; }
      prompt_with_default pilot_control_url "pilot_control repo SSH URL"
      prompt_with_default ros_odrive_url "ros_odrive repo SSH URL"
      [[ -n "${pilot_control_url}" ]] || { fail_step "pilot_control repo SSH URL is required."; return 1; }
      [[ -n "${ros_odrive_url}" ]] || { fail_step "ros_odrive repo SSH URL is required."; return 1; }

      if [[ ! -d "${WORKSPACE_DIR}/src/pilot_control/.git" ]]; then
        git clone "${pilot_control_url}" "${WORKSPACE_DIR}/src/pilot_control" || { fail_step "pilot_control clone failed."; return 1; }
      else
        log "Reusing existing repo: ${WORKSPACE_DIR}/src/pilot_control"
      fi

      if [[ ! -d "${WORKSPACE_DIR}/src/ros_odrive/.git" ]]; then
        git clone "${ros_odrive_url}" "${WORKSPACE_DIR}/src/ros_odrive" || { fail_step "ros_odrive clone failed."; return 1; }
      else
        log "Reusing existing repo: ${WORKSPACE_DIR}/src/ros_odrive"
      fi
      ;;
    *)
      fail_step "Unknown workspace setup mode: ${mode_choice}"
      return 1
      ;;
  esac

  if [[ ! -d "${WORKSPACE_DIR}/src/pilot_control" ]]; then
    fail_step "pilot_control was not found under ${WORKSPACE_DIR}/src."
    return 1
  fi

  if [[ ! -d "${WORKSPACE_DIR}/src/ros_odrive" ]]; then
    fail_step "ros_odrive was not found under ${WORKSPACE_DIR}/src."
    return 1
  fi

  return 0
}

step_rosdep_setup() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    fail_step "ROS 2 Humble is not installed yet."
    return 1
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || { fail_step "Failed to source /opt/ros/humble/setup.bash."; return 1; }

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || { fail_step "rosdep init failed."; return 1; }
  else
    log "rosdep is already initialized."
  fi

  rosdep update || { fail_step "rosdep update failed."; return 1; }
  return 0
}

step_fields2cover_prereqs() {
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
    nlohmann-json3-dev || { fail_step "Failed to install Fields2Cover prerequisites."; return 1; }

  python3 -m pip install --user gcovr || { fail_step "Failed to install gcovr."; return 1; }
  return 0
}

step_fields2cover_build() {
  local ref_name

  prompt_with_default FIELDS2COVER_DIR "Fields2Cover source directory" "${FIELDS2COVER_DIR}"
  FIELDS2COVER_DIR="$(expand_path "${FIELDS2COVER_DIR}")"

  if [[ ! -d "${FIELDS2COVER_DIR}/.git" ]]; then
    git clone https://github.com/Fields2Cover/Fields2Cover.git "${FIELDS2COVER_DIR}" || { fail_step "Fields2Cover clone failed."; return 1; }
  else
    log "Reusing existing Fields2Cover checkout at ${FIELDS2COVER_DIR}"
  fi

  prompt_with_default ref_name "Approved Fields2Cover tag/commit (leave blank to keep current checkout)" ""
  if [[ -n "${ref_name}" ]]; then
    git -C "${FIELDS2COVER_DIR}" checkout "${ref_name}" || { fail_step "Failed to checkout the requested Fields2Cover ref."; return 1; }
  fi

  cmake -S "${FIELDS2COVER_DIR}" -B "${FIELDS2COVER_DIR}/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF || { fail_step "Fields2Cover CMake configure failed."; return 1; }
  cmake --build "${FIELDS2COVER_DIR}/build" -- -j"$(nproc)" || { fail_step "Fields2Cover build failed."; return 1; }
  sudo cmake --install "${FIELDS2COVER_DIR}/build" || { fail_step "Fields2Cover install failed."; return 1; }
  sudo ldconfig || { fail_step "ldconfig failed after Fields2Cover install."; return 1; }

  if [[ ! -f /usr/local/include/fields2cover.h ]]; then
    fail_step "Fields2Cover header not found at /usr/local/include/fields2cover.h after install."
    return 1
  fi

  return 0
}

step_workspace_build() {
  if [[ ! -d "${WORKSPACE_DIR}/src/pilot_control" || ! -d "${WORKSPACE_DIR}/src/ros_odrive" ]]; then
    fail_step "Workspace source tree is incomplete."
    return 1
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || { fail_step "Failed to source /opt/ros/humble/setup.bash."; return 1; }
  cd "${WORKSPACE_DIR}" || { fail_step "Could not enter ${WORKSPACE_DIR}."; return 1; }

  rosdep install --from-paths src --ignore-src -y --rosdistro humble || { fail_step "rosdep install failed for the workspace."; return 1; }
  colcon build --symlink-install --packages-select odrive_can pilot_control || { fail_step "colcon build failed for odrive_can and pilot_control."; return 1; }

  if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    fail_step "Workspace build finished, but install/setup.bash is missing."
    return 1
  fi

  return 0
}

step_planner_build() {
  local planner_dir="${WORKSPACE_DIR}/src/pilot_control/scripts/F2C/cpp"
  local planner_binary="${planner_dir}/build/bdr_coverage_planner"

  if [[ ! -d "${planner_dir}" ]]; then
    fail_step "Planner source directory not found: ${planner_dir}"
    return 1
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || { fail_step "Failed to source /opt/ros/humble/setup.bash."; return 1; }
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash" || { fail_step "Failed to source ${WORKSPACE_DIR}/install/setup.bash."; return 1; }

  cmake -S "${planner_dir}" -B "${planner_dir}/build" -DCMAKE_BUILD_TYPE=Release || { fail_step "Planner CMake configure failed."; return 1; }
  cmake --build "${planner_dir}/build" -- -j"$(nproc)" || { fail_step "Planner build failed."; return 1; }

  if [[ ! -x "${planner_binary}" ]]; then
    fail_step "Planner binary is missing after build: ${planner_binary}"
    return 1
  fi

  if ! ldd "${planner_binary}" 2>/dev/null | grep -qi fields2cover; then
    log "Warning: could not confirm a dynamic Fields2Cover link in ${planner_binary}."
    log "Check the planner build log carefully before field use."
  fi

  return 0
}

step_runtime_environment() {
  local dds_config_path="${HOME}/cyclone_loopback.xml"
  local bashrc_path="${HOME}/.bashrc"
  local bashrc_block

  cat > "${dds_config_path}" <<'EOF'
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

  bashrc_block=$(cat <<EOF
# >>> pilot_control operator laptop setup >>>
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file://${HOME}/cyclone_loopback.xml
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
# <<< pilot_control operator laptop setup <<<
EOF
)

  append_block_if_missing "${bashrc_path}" "# >>> pilot_control operator laptop setup >>>" "${bashrc_block}"
  return 0
}

step_laptop_launch_config() {
  local config_dir
  local use_xterm_yaml="true"
  local interactive_sdl_yaml="true"
  local cmd_vel_enabled_yaml="true"

  prompt_with_default ROBOT_HOST "Robot IP or hostname for laptop_teleop" "${ROBOT_HOST}"

  if ask_yes_no "Launch host_teleop in xterm by default?" Y; then
    use_xterm_yaml="true"
  else
    use_xterm_yaml="false"
  fi

  if ask_yes_no "Enable the interactive SDL keyboard window by default?" Y; then
    interactive_sdl_yaml="true"
  else
    interactive_sdl_yaml="false"
  fi

  if ask_yes_no "Enable cmd_vel publishing from laptop teleop by default?" Y; then
    cmd_vel_enabled_yaml="true"
  else
    cmd_vel_enabled_yaml="false"
  fi

  config_dir="$(dirname "${DEFAULT_LAPTOP_CONFIG_PATH}")"
  mkdir -p "${config_dir}" || { fail_step "Failed to create ${config_dir}."; return 1; }

  if [[ -f "${DEFAULT_LAPTOP_CONFIG_PATH}" ]]; then
    if ! ask_yes_no "Overwrite ${DEFAULT_LAPTOP_CONFIG_PATH}?" N; then
      fail_step "Laptop teleop config already exists and was not overwritten."
      return 1
    fi
  fi

  cat > "${DEFAULT_LAPTOP_CONFIG_PATH}" <<EOF
# Copy this file outside the git repo, e.g. to ~/pilot_config/laptop.yaml
# laptop_teleop.launch.py will use ~/pilot_config/laptop.yaml by default, or the
# path in the PILOT_LAPTOP_CONFIG environment variable.
#
# Keep teleop.robot_ip aligned with the matching robot host in:
#   ~/.config/PilotControl/BDRCoveragePlanner/robots.json

teleop:
  robot_ip: "${ROBOT_HOST}"
  use_xterm: ${use_xterm_yaml}
  interactive_sdl: ${interactive_sdl_yaml}
  cmd_vel_enabled: ${cmd_vel_enabled_yaml}
EOF

  return 0
}

step_optional_extras() {
  if ask_yes_no "Install open3d for the planner's preferred 3D viewer?" Y; then
    python3 -m pip install --user open3d || { fail_step "Failed to install open3d."; return 1; }
  else
    log "Skipping open3d installation."
  fi

  if ask_yes_no "Install AWS CLI for the optional Laptop -> Cloud workflow?" N; then
    sudo apt install -y awscli || { fail_step "Failed to install awscli."; return 1; }
    log "Run 'aws configure' later if you plan to use cloud upload."
  else
    log "Skipping awscli installation."
  fi

  return 0
}

step_robot_registry() {
  local registry_dir

  prompt_with_default ROBOT_ID "Robot ID" "${ROBOT_ID}"
  prompt_with_default ROBOT_HOST "Robot IP or hostname" "${ROBOT_HOST}"
  prompt_with_default ROBOT_SSH_USER "Robot SSH user" "${ROBOT_SSH_USER}"
  prompt_with_default ROBOT_DATA_PATH "Robot data path" "${ROBOT_DATA_PATH}"
  prompt_with_default ROBOT_UPLOAD_DIR "Robot waypoint upload directory" "${ROBOT_UPLOAD_DIR}"

  if ask_yes_no "Pin the robot host key now with ssh-keyscan?" N; then
    ROBOT_KNOWN_HOSTS_ENTRY="$(ssh-keyscan -t ed25519 "${ROBOT_HOST}" 2>/dev/null | tail -n 1)"
    if [[ -z "${ROBOT_KNOWN_HOSTS_ENTRY}" ]]; then
      fail_step "Could not fetch the robot host key with ssh-keyscan."
      return 1
    fi
  else
    prompt_with_default ROBOT_KNOWN_HOSTS_ENTRY "known_hosts entry (leave blank for none)" "${ROBOT_KNOWN_HOSTS_ENTRY}"
  fi

  registry_dir="$(dirname "${DEFAULT_REGISTRY_PATH}")"
  mkdir -p "${registry_dir}" || { fail_step "Failed to create the registry directory."; return 1; }

  if [[ -f "${DEFAULT_REGISTRY_PATH}" ]]; then
    if ! ask_yes_no "Overwrite ${DEFAULT_REGISTRY_PATH}?" N; then
      fail_step "Registry file already exists and was not overwritten."
      return 1
    fi
  fi

  cat > "${DEFAULT_REGISTRY_PATH}" <<EOF
{
  "robots": [
    {
      "robot_id": "${ROBOT_ID}",
      "host": "${ROBOT_HOST}",
      "ssh_user": "${ROBOT_SSH_USER}",
      "robot_data_path": "${ROBOT_DATA_PATH}",
      "default_remote_upload_dir": "${ROBOT_UPLOAD_DIR}",
      "known_hosts_entry": "${ROBOT_KNOWN_HOSTS_ENTRY}"
    }
  ]
}
EOF

  return 0
}

step_robot_ssh_validation() {
  if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_SSH_USER}" ]]; then
    fail_step "Robot SSH validation needs both ROBOT_HOST and ROBOT_SSH_USER."
    return 1
  fi

  ssh -o BatchMode=yes "${ROBOT_SSH_USER}@${ROBOT_HOST}" true || { fail_step "Batch-mode SSH connection to the robot failed."; return 1; }
  ssh -o BatchMode=yes "${ROBOT_SSH_USER}@${ROBOT_HOST}" pilot_control_auth --help || { fail_step "pilot_control_auth was not reachable over SSH."; return 1; }
  return 0
}

step_local_validation() {
  local planner_binary="${WORKSPACE_DIR}/src/pilot_control/scripts/F2C/cpp/build/bdr_coverage_planner"

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || { fail_step "Failed to source /opt/ros/humble/setup.bash."; return 1; }
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash" || { fail_step "Failed to source ${WORKSPACE_DIR}/install/setup.bash."; return 1; }

  ros2 interface show odrive_can/srv/AxisState >/dev/null || { fail_step "Could not resolve odrive_can/srv/AxisState."; return 1; }
  ros2 pkg prefix pilot_control >/dev/null || { fail_step "Could not resolve the pilot_control package prefix."; return 1; }
  [[ -x "${planner_binary}" ]] || { fail_step "Planner binary is not executable: ${planner_binary}"; return 1; }
  [[ -f "${DEFAULT_LAPTOP_CONFIG_PATH}" ]] || { fail_step "Laptop teleop config is missing: ${DEFAULT_LAPTOP_CONFIG_PATH}"; return 1; }
  zenohd --version >/dev/null || { fail_step "zenohd is not available on PATH."; return 1; }
  gst-inspect-1.0 rtph264depay h264parse avdec_h264 videoconvert autovideosink >/dev/null || {
    fail_step "Required GStreamer laptop-side runtime elements are missing."
    return 1
  }

  return 0
}

main() {
  log "Operator laptop production setup"
  log "Repo directory: ${SCRIPT_DIR}"
  log "Log file: ${LOG_FILE}"

  check_platform || exit 1

  run_step "Step 1: Baseline Ubuntu setup" step_baseline_system
  run_step "Step 2: ROS 2 apt repository bootstrap" step_ros2_apt_bootstrap
  run_step "Step 3: Zenoh apt repository bootstrap" step_zenoh_apt_bootstrap
  run_step "Step 4: ROS 2 and build dependency install" step_install_ros_and_build_deps
  run_step "Step 5: Git identity setup" step_git_identity
  run_step "Step 6: Git SSH key and host access setup" step_git_ssh_access
  run_step "Step 7: Workspace creation and repo checkout" step_workspace_setup
  run_step "Step 8: rosdep setup" step_rosdep_setup
  run_step "Step 9: Fields2Cover prerequisites" step_fields2cover_prereqs
  run_step "Step 10: Fields2Cover build and install" step_fields2cover_build
  run_step "Step 11: ROS workspace build" step_workspace_build
  run_step "Step 12: Coverage planner build" step_planner_build
  run_step "Step 13: Runtime environment setup" step_runtime_environment
  run_step "Step 14: Machine-local laptop teleop config" step_laptop_launch_config
  run_step "Step 15: Optional extras (open3d, awscli)" step_optional_extras
  run_step "Step 16: Planner robot registry" step_robot_registry
  run_step "Step 17: Robot SSH validation" step_robot_ssh_validation
  run_step "Step 18: Local install validation" step_local_validation

  log
  log "Installer finished."
  log "Rerun this script later and skip completed steps if you only need the remaining pieces."
}

main "$@"
