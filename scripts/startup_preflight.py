#!/usr/bin/env python3
"""
ROOFUS V2 Startup Preflight Check
===================================

Standalone script that performs hardware health checks before deployment.
Designed to be triggered from the coverage planner UI via SSH.

Checks performed:
  1. RGB cameras (left + right) — connectivity, occlusion, sample images
  2. Thermal camera — connectivity, occlusion, sample image
  3. LiDAR (Livox MID360) — connectivity, publish rate, point count
  4. RF link (Microhard) — ping latency and packet loss
  5. GPS (u-blox ZED-F9P) — fix type, satellite count, accuracy
  6. Drive motors (left + right) — forward, backward, axial turns

Usage:
    ros2 run pilot_control startup_preflight

    # With custom parameters
    ros2 run pilot_control startup_preflight --ros-args \\
        -p rf_target_ip:=192.168.168.100 \\
        -p skip_motion_test:=false

Results are saved to: /R_DATA/startup_check/<YYYYMMDD_HHMMSS>/
A 'latest' symlink always points to the most recent report.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, Imu
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
from std_msgs.msg import String

# Livox driver publishes CustomMsg (not PointCloud2) — try to import it
try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
    LIVOX_CUSTOM_AVAILABLE = True
except ImportError:
    LivoxCustomMsg = None
    LIVOX_CUSTOM_AVAILABLE = False

import cv2
import numpy as np
import os
import sys
import time
import json
import glob
import shutil
import struct
import signal
import subprocess
import serial
import yaml
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Any

DEFAULT_ROBOT_CONFIG_FILE = os.environ.get(
    'PILOT_ROBOT_CONFIG',
    os.path.expanduser('~/pilot_config/robot.yaml')
)
DEFAULT_LEFT_CAMERA_DEVICE = '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0'
DEFAULT_RIGHT_CAMERA_DEVICE = '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0'


def load_robot_config(config_path: str, logger) -> Dict[str, Any]:
    """Load machine-local robot config from outside the git repo."""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}
        if not isinstance(config, dict):
            logger.warn(
                f'Robot config {config_path} ignored: top-level YAML must be a mapping')
            return {}
        logger.info(f'Robot config loaded from {config_path}')
        return config
    except FileNotFoundError:
        logger.info(f'Robot config not found at {config_path}; using built-in defaults')
    except Exception as exc:
        logger.warn(f'Failed to load robot config {config_path}: {exc}')
    return {}


def config_value(config: Dict[str, Any], dotted_key: str, default):
    current = config
    for part in dotted_key.split('.'):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


# ═══════════════════════════════════════════════════════════════════════
# Data classes for results
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class CameraResult:
    status: str = 'NOT_RUN'  # PASS, FAIL, WARN, NOT_RUN
    connected: bool = False
    brightness: float = 0.0
    variance: float = 0.0
    sharpness: float = 0.0       # Laplacian variance (edge content)
    edge_density: float = 0.0    # Percentage of Canny edge pixels
    entropy: float = 0.0         # Histogram entropy (information content)
    sample_image: str = ''
    reason: str = ''


@dataclass
class ThermalResult:
    status: str = 'NOT_RUN'
    connected: bool = False
    temp_range_c: float = 0.0
    min_temp_c: float = 0.0
    max_temp_c: float = 0.0
    center_temp_c: float = 0.0
    sample_image: str = ''
    reason: str = ''


@dataclass
class LidarResult:
    status: str = 'NOT_RUN'
    connected: bool = False
    rate_hz: float = 0.0
    avg_points: int = 0
    imu_ok: bool = False
    reason: str = ''


@dataclass
class RfResult:
    status: str = 'NOT_RUN'
    reachable: bool = False
    avg_rtt_ms: float = -1.0
    packet_loss_pct: float = 100.0
    target_ip: str = ''
    reason: str = ''


@dataclass
class GpsResult:
    status: str = 'NOT_RUN'
    connected: bool = False
    fix_type: int = 0
    fix_type_str: str = 'none'
    num_sats: int = 0
    h_acc_m: float = -1.0
    v_acc_m: float = -1.0
    pdop: float = -1.0
    latitude: float = 0.0
    longitude: float = 0.0
    reason: str = ''


@dataclass
class ManeuverResult:
    name: str = ''
    left_ok: bool = False
    right_ok: bool = False
    left_vel_avg: float = 0.0
    right_vel_avg: float = 0.0
    left_current_max: float = 0.0
    right_current_max: float = 0.0
    errors_detected: bool = False
    reason: str = ''


@dataclass
class MotorResult:
    status: str = 'NOT_RUN'
    armed_ok: bool = False
    disarmed_ok: bool = False
    maneuvers: List[ManeuverResult] = field(default_factory=list)
    reason: str = ''


# ═══════════════════════════════════════════════════════════════════════
# Subprocess manager for temporary hardware drivers
# ═══════════════════════════════════════════════════════════════════════

class SubprocessManager:
    """Manages temporary subprocesses with clean shutdown."""

    def __init__(self, logger):
        self.logger = logger
        self.processes: List[subprocess.Popen] = []

    def start(self, cmd: List[str], label: str = '',
              capture_output: bool = False) -> Optional[subprocess.Popen]:
        """Start a subprocess in its own process group."""
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE if capture_output else subprocess.DEVNULL,
                stderr=subprocess.PIPE if capture_output else subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(proc)
            if label:
                self.logger.info(f'  Started {label} (PID {proc.pid})')
            return proc
        except Exception as e:
            self.logger.error(f'  Failed to start {label}: {e}')
            return None

    def get_output(self, proc: subprocess.Popen, max_lines: int = 20) -> str:
        """Get captured stderr/stdout from a process (non-blocking)."""
        lines = []
        try:
            if proc.stderr:
                import select
                while select.select([proc.stderr], [], [], 0)[0]:
                    line = proc.stderr.readline()
                    if not line:
                        break
                    lines.append(line.decode('utf-8', errors='replace').rstrip())
                    if len(lines) >= max_lines:
                        break
        except Exception:
            pass
        return '\n'.join(lines)

    def stop_all(self):
        """Stop all managed subprocesses."""
        for proc in self.processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.processes.clear()

    def stop(self, proc: subprocess.Popen):
        """Stop a specific subprocess."""
        if proc in self.processes:
            self.processes.remove(proc)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════
# Main Preflight Node
# ═══════════════════════════════════════════════════════════════════════

class StartupPreflight(Node):
    def __init__(self):
        super().__init__('startup_preflight')

        self.declare_parameter('robot_config_file', DEFAULT_ROBOT_CONFIG_FILE)
        self.robot_config_file = str(self.get_parameter('robot_config_file').value)
        self.robot_config = load_robot_config(self.robot_config_file, self.get_logger())

        # ── Parameters ──
        # Camera
        # Camera serial-based paths (from /dev/v4l/by-id/ on the robot)
        # 3728... = Left camera, 0F12... = Right camera
        self.declare_parameter('left_device',
            config_value(self.robot_config, 'cameras.left_device', DEFAULT_LEFT_CAMERA_DEVICE))
        self.declare_parameter('right_device',
            config_value(self.robot_config, 'cameras.right_device', DEFAULT_RIGHT_CAMERA_DEVICE))
        self.declare_parameter('brightness_min', 15.0)
        self.declare_parameter('variance_min', 50.0)
        self.declare_parameter('sharpness_min', 50.0)      # Laplacian variance
        self.declare_parameter('edge_density_min', 1.5)     # % of Canny edge pixels
        self.declare_parameter('entropy_min', 4.5)          # Histogram entropy (bits)
        self.declare_parameter('thermal_range_min', 2.0)
        self.declare_parameter('camera_warmup_frames', 10)

        # LiDAR
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('lidar_min_rate_hz', 5.0)
        self.declare_parameter('lidar_min_points', 1000)
        self.declare_parameter('lidar_check_duration', 5.0)

        # RF
        self.declare_parameter(
            'rf_target_ip',
            config_value(self.robot_config, 'preflight.rf_target_ip', '192.168.168.100'))
        self.declare_parameter('rf_ping_count', 5)
        self.declare_parameter('rf_max_loss_pct', 20.0)
        self.declare_parameter('rf_max_rtt_ms', 200.0)

        # GPS
        self.declare_parameter(
            'gps_device',
            config_value(self.robot_config, 'gps.device', '/dev/gps'))
        self.declare_parameter('gps_baud', 38400)
        self.declare_parameter('gps_min_sats', 8)
        self.declare_parameter('gps_max_hacc_m', 5.0)
        self.declare_parameter('gps_max_pdop', 4.0)
        self.declare_parameter('gps_timeout_s', 15.0)
        self.declare_parameter('skip_gps_check', False)

        # Motors
        self.declare_parameter(
            'can_interface',
            config_value(self.robot_config, 'can.interface', 'can0'))
        self.declare_parameter(
            'can_bitrate',
            int(config_value(self.robot_config, 'can.bitrate', 250000)))
        self.declare_parameter(
            'left_node_id',
            int(config_value(self.robot_config, 'can.left_node_id', 0)))
        self.declare_parameter(
            'right_node_id',
            int(config_value(self.robot_config, 'can.right_node_id', 1)))
        self.declare_parameter('motor_test_vel', 0.3)
        self.declare_parameter('motor_test_duration', 1.5)
        self.declare_parameter('motor_stop_duration', 1.0)
        self.declare_parameter('motor_max_current', 5.0)
        self.declare_parameter('motor_vel_threshold', 0.05)
        self.declare_parameter('skip_motion_test', False)

        # Report
        self.declare_parameter('report_base_dir', '/R_DATA/startup_check')
        self.declare_parameter('max_reports', 30)

        # ── Get parameters ──
        self.left_device = str(self.get_parameter('left_device').value)
        self.right_device = str(self.get_parameter('right_device').value)
        self.brightness_min = float(self.get_parameter('brightness_min').value)
        self.variance_min = float(self.get_parameter('variance_min').value)
        self.sharpness_min = float(self.get_parameter('sharpness_min').value)
        self.edge_density_min = float(self.get_parameter('edge_density_min').value)
        self.entropy_min = float(self.get_parameter('entropy_min').value)
        self.thermal_range_min = float(self.get_parameter('thermal_range_min').value)
        self.camera_warmup_frames = int(self.get_parameter('camera_warmup_frames').value)

        self.lidar_topic = str(self.get_parameter('lidar_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.lidar_min_rate_hz = float(self.get_parameter('lidar_min_rate_hz').value)
        self.lidar_min_points = int(self.get_parameter('lidar_min_points').value)
        self.lidar_check_duration = float(self.get_parameter('lidar_check_duration').value)

        self.rf_target_ip = str(self.get_parameter('rf_target_ip').value)
        self.rf_ping_count = int(self.get_parameter('rf_ping_count').value)
        self.rf_max_loss_pct = float(self.get_parameter('rf_max_loss_pct').value)
        self.rf_max_rtt_ms = float(self.get_parameter('rf_max_rtt_ms').value)

        self.gps_device = str(self.get_parameter('gps_device').value)
        self.gps_baud = int(self.get_parameter('gps_baud').value)
        self.gps_min_sats = int(self.get_parameter('gps_min_sats').value)
        self.gps_max_hacc_m = float(self.get_parameter('gps_max_hacc_m').value)
        self.gps_max_pdop = float(self.get_parameter('gps_max_pdop').value)
        self.gps_timeout_s = float(self.get_parameter('gps_timeout_s').value)
        self.skip_gps_check = bool(self.get_parameter('skip_gps_check').value)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.left_node_id = int(self.get_parameter('left_node_id').value)
        self.right_node_id = int(self.get_parameter('right_node_id').value)
        self.motor_test_vel = float(self.get_parameter('motor_test_vel').value)
        self.motor_test_duration = float(self.get_parameter('motor_test_duration').value)
        self.motor_stop_duration = float(self.get_parameter('motor_stop_duration').value)
        self.motor_max_current = float(self.get_parameter('motor_max_current').value)
        self.motor_vel_threshold = float(self.get_parameter('motor_vel_threshold').value)
        self.skip_motion_test = bool(self.get_parameter('skip_motion_test').value)

        self.report_base_dir = str(self.get_parameter('report_base_dir').value)
        self.max_reports = int(self.get_parameter('max_reports').value)

        # ── State for motor feedback ──
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_iq = 0.0
        self.right_iq = 0.0
        self.left_axis_state = 0
        self.right_axis_state = 0
        self.left_errors = 0
        self.right_errors = 0
        self.left_vel_time = 0.0
        self.right_vel_time = 0.0

        # ── State for LiDAR ──
        self.lidar_timestamps: List[float] = []
        self.lidar_point_counts: List[int] = []
        self.imu_received = False

        # ── Subprocess manager ──
        self.subproc = SubprocessManager(self.get_logger())

        # ── Latched report publisher ──
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.report_pub = self.create_publisher(String, '/preflight/report', latched_qos)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ROOFUS V2 STARTUP PREFLIGHT CHECK')
        self.get_logger().info('=' * 60)

    # ═══════════════════════════════════════════════════════════════
    # CHECK 1 & 2: RGB Cameras
    # ═══════════════════════════════════════════════════════════════

    def check_rgb_camera(self, name: str, device: str, output_dir: str) -> CameraResult:
        """Check a single RGB camera: connectivity, occlusion, save sample."""
        result = CameraResult()
        self.get_logger().info(f'  [{name}] Opening {device}...')

        # Check device exists
        if not os.path.exists(device):
            result.status = 'FAIL'
            result.reason = f'device not found: {device}'
            self.get_logger().error(f'  [{name}] {result.reason}')
            return result

        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            result.status = 'FAIL'
            result.reason = 'could not open camera'
            self.get_logger().error(f'  [{name}] {result.reason}')
            return result

        result.connected = True

        # Grab warmup frames (auto-exposure settle)
        frame = None
        for i in range(self.camera_warmup_frames):
            ret, frame = cap.read()
            if not ret:
                result.status = 'FAIL'
                result.reason = f'failed to grab frame {i+1}'
                cap.release()
                self.get_logger().error(f'  [{name}] {result.reason}')
                return result

        cap.release()

        if frame is None:
            result.status = 'FAIL'
            result.reason = 'no frames captured'
            return result

        # Save sample image
        sample_path = os.path.join(output_dir, f'{name}_sample.png')
        cv2.imwrite(sample_path, frame)
        result.sample_image = f'{name}_sample.png'

        # ── Multi-metric occlusion analysis ──
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Basic stats
        result.brightness = float(np.mean(gray))
        result.variance = float(np.var(gray.astype(np.float64)))

        # 2. Sharpness — Laplacian variance measures edge content.
        #    A real scene has abundant edges (high value); tape/obstruction
        #    produces a blurry, featureless image (low value).
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        result.sharpness = float(np.var(laplacian))

        # 3. Edge density — percentage of pixels that are Canny edges.
        #    A real scene has 3-15%+ edge pixels; an obstructed camera < 1%.
        edges = cv2.Canny(gray, 50, 150)
        result.edge_density = float(np.count_nonzero(edges) / edges.size * 100.0)

        # 4. Histogram entropy — information content in bits.
        #    A real scene has a wide pixel distribution (entropy > 5);
        #    tape/cover compresses into a narrow band (entropy < 4).
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).flatten()
        hist_norm = hist / hist.sum()
        # Avoid log(0)
        hist_nonzero = hist_norm[hist_norm > 0]
        result.entropy = float(-np.sum(hist_nonzero * np.log2(hist_nonzero)))

        # ── Evaluate: fail on ANY metric indicating obstruction ──
        issues = []
        if result.brightness < self.brightness_min:
            issues.append(f'too dark ({result.brightness:.0f}<{self.brightness_min:.0f})')
        if result.variance < self.variance_min:
            issues.append(f'low variance ({result.variance:.0f}<{self.variance_min:.0f})')
        if result.sharpness < self.sharpness_min:
            issues.append(f'no detail ({result.sharpness:.0f}<{self.sharpness_min:.0f})')
        if result.edge_density < self.edge_density_min:
            issues.append(f'no edges ({result.edge_density:.1f}%<{self.edge_density_min:.1f}%)')
        if result.entropy < self.entropy_min:
            issues.append(f'low entropy ({result.entropy:.1f}<{self.entropy_min:.1f})')

        if issues:
            result.status = 'FAIL'
            result.reason = 'obstructed? ' + '; '.join(issues)
        else:
            result.status = 'PASS'

        self.get_logger().info(
            f'  [{name}] {result.status} — bright={result.brightness:.0f} '
            f'var={result.variance:.0f} sharp={result.sharpness:.0f} '
            f'edge={result.edge_density:.1f}% entropy={result.entropy:.1f}')
        return result

    # ═══════════════════════════════════════════════════════════════
    # CHECK 3: Thermal Camera
    # ═══════════════════════════════════════════════════════════════

    def check_thermal_camera(self, output_dir: str) -> ThermalResult:
        """Check thermal camera using the thermal_check C++ helper."""
        result = ThermalResult()
        self.get_logger().info('  [thermal] Running thermal_check helper...')

        sample_path = os.path.join(output_dir, 'thermal_sample.png')

        # Find the thermal_check executable
        thermal_check_cmd = None
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'thermal_check'),
            # Installed path (ros2 run puts executables in lib/<pkg>/)
            'thermal_check',
        ]

        # Try to find via ros2 package path
        try:
            pkg_result = subprocess.run(
                ['ros2', 'pkg', 'prefix', 'pilot_control'],
                capture_output=True, text=True, timeout=5
            )
            if pkg_result.returncode == 0:
                pkg_path = pkg_result.stdout.strip()
                possible_paths.insert(0, os.path.join(pkg_path, 'lib', 'pilot_control', 'thermal_check'))
        except Exception:
            pass

        for path in possible_paths:
            if os.path.isfile(path) and os.access(path, os.X_OK):
                thermal_check_cmd = path
                break

        if not thermal_check_cmd:
            # Fallback: just check if Seek USB device is connected
            self.get_logger().warn('  [thermal] thermal_check not found, checking USB only')
            try:
                lsusb = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
                # Seek Thermal vendor ID is 289d
                if '289d:' in lsusb.stdout.lower():
                    result.connected = True
                    result.status = 'WARN'
                    result.reason = 'USB device detected but frame check unavailable (thermal_check not built)'
                else:
                    result.status = 'FAIL'
                    result.reason = 'Seek thermal camera USB device not detected'
            except Exception as e:
                result.status = 'FAIL'
                result.reason = f'USB check failed: {e}'
            self.get_logger().info(f'  [thermal] {result.status} — {result.reason}')
            return result

        # Run thermal_check helper
        try:
            proc = subprocess.run(
                [thermal_check_cmd, sample_path, '15', str(self.thermal_range_min)],
                capture_output=True, text=True, timeout=25
            )

            output = proc.stdout.strip()
            if not output:
                result.status = 'FAIL'
                result.reason = 'thermal_check produced no output'
                self.get_logger().error(f'  [thermal] {result.reason}')
                return result

            # Parse output: "PASS range=8.7 min=18.2 max=26.9 center=22.1 w=320 h=240"
            parts = output.split()
            status_word = parts[0] if parts else ''

            result.connected = (status_word in ('PASS', 'FAIL') and 'not_connected' not in output)

            for part in parts[1:]:
                if '=' in part:
                    key, val = part.split('=', 1)
                    try:
                        if key == 'range':
                            result.temp_range_c = float(val)
                        elif key == 'min':
                            result.min_temp_c = float(val)
                        elif key == 'max':
                            result.max_temp_c = float(val)
                        elif key == 'center':
                            result.center_temp_c = float(val)
                    except ValueError:
                        pass

            if status_word == 'PASS':
                result.status = 'PASS'
                result.sample_image = 'thermal_sample.png'
            elif 'not_connected' in output:
                result.status = 'FAIL'
                result.reason = 'thermal camera not connected'
            elif 'timeout' in output:
                result.status = 'FAIL'
                result.reason = 'thermal camera timeout (no frame received)'
            else:
                result.status = 'FAIL'
                result.reason = f'low temp range ({result.temp_range_c:.1f}C < {self.thermal_range_min}C), obstructed?'
                result.sample_image = 'thermal_sample.png'

        except subprocess.TimeoutExpired:
            result.status = 'FAIL'
            result.reason = 'thermal_check timed out'
        except Exception as e:
            result.status = 'FAIL'
            result.reason = f'thermal_check error: {e}'

        self.get_logger().info(
            f'  [thermal] {result.status} — range={result.temp_range_c:.1f}C '
            f'min={result.min_temp_c:.1f} max={result.max_temp_c:.1f}')
        return result

    # ═══════════════════════════════════════════════════════════════
    # CHECK 4: LiDAR
    # ═══════════════════════════════════════════════════════════════

    def _lidar_callback(self, msg):
        """Collect LiDAR message stats (works with both PointCloud2 and CustomMsg)."""
        self.lidar_timestamps.append(time.time())
        # Handle PointCloud2
        if hasattr(msg, 'width') and hasattr(msg, 'point_step'):
            if msg.width > 0 and msg.height > 0:
                self.lidar_point_counts.append(msg.width * msg.height)
            elif msg.point_step > 0:
                self.lidar_point_counts.append(len(msg.data) // msg.point_step)
            else:
                self.lidar_point_counts.append(0)
        # Handle Livox CustomMsg
        elif hasattr(msg, 'point_num'):
            self.lidar_point_counts.append(msg.point_num)
        else:
            self.lidar_point_counts.append(1)  # At least we received something

    def _imu_callback(self, msg: Imu):
        """Note that IMU is being received."""
        self.imu_received = True

    def check_lidar(self) -> LidarResult:
        """Check LiDAR by temporarily starting Livox driver."""
        result = LidarResult()
        self.lidar_timestamps.clear()
        self.lidar_point_counts.clear()
        self.imu_received = False

        self.get_logger().info('  [lidar] Starting Livox driver...')

        # Check if Livox driver is already running
        already_running = False
        try:
            topic_list = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True, text=True, timeout=5
            )
            if '/livox/imu' in topic_list.stdout:
                already_running = True
                self.get_logger().info('  [lidar] Livox driver already running')
        except Exception:
            pass

        livox_proc = None
        if not already_running:
            # Find config file
            config_file = None
            try:
                pkg_result = subprocess.run(
                    ['ros2', 'pkg', 'prefix', 'livox_ros_driver2'],
                    capture_output=True, text=True, timeout=5
                )
                if pkg_result.returncode == 0:
                    pkg_path = pkg_result.stdout.strip()
                    candidate = os.path.join(pkg_path, 'share', 'livox_ros_driver2',
                                             'config', 'MID360_config.json')
                    if os.path.exists(candidate):
                        config_file = candidate
            except Exception:
                pass

            if not config_file:
                for path in ['/opt/ros/humble/share/livox_ros_driver2/config/MID360_config.json']:
                    if os.path.exists(path):
                        config_file = path
                        break

            if not config_file:
                result.status = 'FAIL'
                result.reason = 'Livox config file not found'
                self.get_logger().error(f'  [lidar] {result.reason}')
                return result

            cmd = [
                'ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node',
                '--ros-args',
                '-p', 'xfer_format:=1', '-p', 'multi_topic:=0',
                '-p', 'data_src:=0', '-p', 'publish_freq:=10.0',
                '-p', 'output_data_type:=0', '-p', 'frame_id:=livox_frame',
                '-p', f'user_config_path:={config_file}',
            ]
            self.get_logger().info(f'  [lidar] Config: {config_file}')
            livox_proc = self.subproc.start(cmd, 'Livox driver',
                                            capture_output=True)
            if not livox_proc:
                result.status = 'FAIL'
                result.reason = 'failed to start Livox driver'
                return result

            # Wait for driver to initialize — the Livox driver takes ~3s to
            # connect to the LiDAR and create publishers. ros2 topic list is
            # unreliable for detecting them (DDS discovery delay), so we use
            # a fixed wait plus process liveness check.
            init_wait = 5
            self.get_logger().info(f'  [lidar] Waiting {init_wait}s for driver to initialize...')
            for i in range(init_wait):
                time.sleep(1)
                if livox_proc.poll() is not None:
                    result.status = 'FAIL'
                    result.reason = f'Livox driver exited prematurely (code={livox_proc.returncode})'
                    driver_log = self.subproc.get_output(livox_proc)
                    if driver_log:
                        result.reason += f': {driver_log[:200]}'
                    self.get_logger().error(f'  [lidar] {result.reason}')
                    self.subproc.stop(livox_proc)
                    return result
            self.get_logger().info(f'  [lidar] Driver init wait complete')

        # Subscribe to LiDAR topic — Livox driver publishes CustomMsg by default,
        # even with xfer_format:=1. Try CustomMsg first, fall back to PointCloud2.
        lidar_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        lidar_qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        lidar_subs = []
        if LIVOX_CUSTOM_AVAILABLE:
            self.get_logger().info('  [lidar] Subscribing with CustomMsg + PointCloud2')
            lidar_subs.append(self.create_subscription(
                LivoxCustomMsg, self.lidar_topic, self._lidar_callback, lidar_qos))
            lidar_subs.append(self.create_subscription(
                LivoxCustomMsg, self.lidar_topic, self._lidar_callback, lidar_qos_reliable))
        else:
            self.get_logger().info('  [lidar] CustomMsg not available, using PointCloud2 only')
        # Also subscribe as PointCloud2 in case driver is configured for it
        lidar_subs.append(self.create_subscription(
            PointCloud2, self.lidar_topic, self._lidar_callback, lidar_qos))
        lidar_subs.append(self.create_subscription(
            PointCloud2, self.lidar_topic, self._lidar_callback, lidar_qos_reliable))
        # IMU subscriptions
        imu_subs = [
            self.create_subscription(Imu, self.imu_topic, self._imu_callback, lidar_qos),
            self.create_subscription(Imu, self.imu_topic, self._imu_callback, lidar_qos_reliable),
        ]

        # Allow DDS discovery time
        self.get_logger().info('  [lidar] Waiting for subscription matching...')
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f'  [lidar] Collecting data for {self.lidar_check_duration}s...')
        t0 = time.time()
        while time.time() - t0 < self.lidar_check_duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Cleanup subscriptions and driver
        for sub in lidar_subs + imu_subs:
            self.destroy_subscription(sub)

        # Grab driver output before stopping
        if livox_proc:
            driver_log = self.subproc.get_output(livox_proc)
            if driver_log:
                for line in driver_log.split('\n')[:10]:
                    self.get_logger().info(f'  [lidar] driver: {line}')
            self.get_logger().info('  [lidar] Stopping Livox driver...')
            self.subproc.stop(livox_proc)

        # Evaluate
        n = len(self.lidar_timestamps)
        if n == 0:
            result.status = 'FAIL'
            result.reason = 'no LiDAR messages received'
            self.get_logger().error(f'  [lidar] {result.reason}')
            return result

        result.connected = True
        result.imu_ok = self.imu_received

        if n >= 2:
            dt = self.lidar_timestamps[-1] - self.lidar_timestamps[0]
            result.rate_hz = round((n - 1) / dt, 1) if dt > 0 else 0.0
        else:
            result.rate_hz = 0.0

        result.avg_points = int(np.mean(self.lidar_point_counts)) if self.lidar_point_counts else 0

        # Determine status
        issues = []
        if result.rate_hz < self.lidar_min_rate_hz:
            issues.append(f'low rate ({result.rate_hz:.1f} < {self.lidar_min_rate_hz} Hz)')
        if result.avg_points < self.lidar_min_points:
            issues.append(f'low point count ({result.avg_points} < {self.lidar_min_points})')
        if not result.imu_ok:
            issues.append('no IMU data received')

        if issues:
            result.status = 'WARN' if result.rate_hz > 0 else 'FAIL'
            result.reason = '; '.join(issues)
        else:
            result.status = 'PASS'

        self.get_logger().info(
            f'  [lidar] {result.status} — rate={result.rate_hz:.1f}Hz, '
            f'points={result.avg_points}, imu={result.imu_ok}')
        return result

    # ═══════════════════════════════════════════════════════════════
    # CHECK 5: RF Link
    # ═══════════════════════════════════════════════════════════════

    def check_rf(self) -> RfResult:
        """Check RF connection to laptop via ping."""
        result = RfResult(target_ip=self.rf_target_ip)
        self.get_logger().info(
            f'  [rf] Pinging {self.rf_target_ip} ({self.rf_ping_count} packets)...')

        try:
            proc = subprocess.run(
                ['ping', '-c', str(self.rf_ping_count), '-W', '2', '-q', self.rf_target_ip],
                capture_output=True, text=True, timeout=30
            )
            output = proc.stdout

            # Parse packet loss
            import re
            loss_match = re.search(r'(\d+(?:\.\d+)?)% packet loss', output)
            if loss_match:
                result.packet_loss_pct = float(loss_match.group(1))

            # Parse RTT avg
            rtt_match = re.search(
                r'rtt min/avg/max/mdev = [\d.]+/([\d.]+)/[\d.]+/[\d.]+ ms', output)
            if rtt_match:
                result.avg_rtt_ms = float(rtt_match.group(1))

            result.reachable = proc.returncode == 0 and result.packet_loss_pct < 100

            # Evaluate
            if not result.reachable:
                result.status = 'FAIL'
                result.reason = f'cannot reach {self.rf_target_ip}'
            elif result.packet_loss_pct > self.rf_max_loss_pct:
                result.status = 'FAIL'
                result.reason = (f'high packet loss ({result.packet_loss_pct:.0f}% > '
                                 f'{self.rf_max_loss_pct:.0f}%)')
            elif result.avg_rtt_ms > self.rf_max_rtt_ms:
                result.status = 'WARN'
                result.reason = (f'high latency ({result.avg_rtt_ms:.0f}ms > '
                                 f'{self.rf_max_rtt_ms:.0f}ms)')
            else:
                result.status = 'PASS'

        except subprocess.TimeoutExpired:
            result.status = 'FAIL'
            result.reason = 'ping command timed out'
        except Exception as e:
            result.status = 'FAIL'
            result.reason = f'ping error: {e}'

        self.get_logger().info(
            f'  [rf] {result.status} — rtt={result.avg_rtt_ms:.0f}ms, '
            f'loss={result.packet_loss_pct:.0f}%')
        return result

    # ═══════════════════════════════════════════════════════════════
    # CHECK 6: GPS
    # ═══════════════════════════════════════════════════════════════

    def check_gps(self) -> GpsResult:
        """Check GPS via direct serial UBX parsing (same as gps_driver.cpp)."""
        result = GpsResult()

        if self.skip_gps_check:
            result.status = 'SKIP'
            result.reason = 'GPS check skipped by parameter'
            self.get_logger().info(f'  [gps] {result.reason}')
            return result

        self.get_logger().info(f'  [gps] Opening {self.gps_device}...')

        # Only try /dev/gps — don't blindly fall back to /dev/ttyACM0
        # which could be a completely different device
        devices_to_try = [self.gps_device]
        if self.gps_device != '/dev/ttyACM0':
            # Only try ACM0 fallback if it's a symlink to help identify GPS
            acm0_path = '/dev/ttyACM0'
            if os.path.exists(acm0_path):
                try:
                    # Quick sanity: check if udevadm identifies it as u-blox
                    udev_check = subprocess.run(
                        ['udevadm', 'info', '--query=all', '--name=' + acm0_path],
                        capture_output=True, text=True, timeout=3
                    )
                    if 'u-blox' in udev_check.stdout.lower() or '1546:' in udev_check.stdout:
                        devices_to_try.append(acm0_path)
                except Exception:
                    # If udevadm fails, still try it but with short timeout validation
                    devices_to_try.append(acm0_path)

        ser = None
        opened_dev = None
        for dev in devices_to_try:
            try:
                ser = serial.Serial(dev, self.gps_baud, timeout=1,
                                    write_timeout=2)
                opened_dev = dev
                self.get_logger().info(f'  [gps] Opened {dev}')
                break
            except Exception:
                continue

        if ser is None:
            result.status = 'FAIL'
            result.reason = f'cannot open GPS serial port ({self.gps_device})'
            self.get_logger().error(f'  [gps] {result.reason}')
            return result

        # Quick validation: read a small amount to check if the device
        # produces any data at all (GPS outputs continuous NMEA/UBX)
        self.get_logger().info('  [gps] Validating device responds...')
        try:
            probe = ser.read(64)
            if len(probe) == 0:
                ser.close()
                result.status = 'FAIL'
                result.reason = f'device {opened_dev} produced no data (not a GPS?)'
                self.get_logger().error(f'  [gps] {result.reason}')
                return result
        except Exception as e:
            ser.close()
            result.status = 'FAIL'
            result.reason = f'device {opened_dev} read failed: {e}'
            self.get_logger().error(f'  [gps] {result.reason}')
            return result

        result.connected = True

        # UBX protocol constants
        UBX_SYNC1 = 0xB5
        UBX_SYNC2 = 0x62

        # Configure receiver to output UBX-NAV-PVT (same as gps_driver.cpp)
        self.get_logger().info('  [gps] Configuring receiver for UBX output...')
        self._configure_gps_ubx(ser, UBX_SYNC1, UBX_SYNC2)
        time.sleep(0.3)  # Wait for config to apply

        # Read serial data and look for UBX-NAV-PVT (class=0x01, id=0x07, len=92)
        self.get_logger().info(f'  [gps] Waiting for NAV-PVT packets (up to {self.gps_timeout_s}s)...')
        t0 = time.time()
        buf = bytearray()
        best_pvt = None

        try:
            while time.time() - t0 < self.gps_timeout_s:
                data = ser.read(256)
                if not data:
                    continue
                buf.extend(data)

                # Scan for UBX frames
                while len(buf) >= 8:
                    # Find sync
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == UBX_SYNC1 and buf[i + 1] == UBX_SYNC2:
                            idx = i
                            break
                    if idx < 0:
                        buf = buf[-1:]  # Keep last byte in case it's 0xB5
                        break
                    if idx > 0:
                        buf = buf[idx:]

                    # Need at least header (6) + payload_len + checksum (2)
                    if len(buf) < 6:
                        break
                    cls = buf[2]
                    msg_id = buf[3]
                    payload_len = buf[4] | (buf[5] << 8)
                    total_len = 6 + payload_len + 2

                    if len(buf) < total_len:
                        break  # Wait for more data

                    # Extract frame
                    frame = buf[:total_len]
                    buf = buf[total_len:]

                    # Check if NAV-PVT
                    if cls == 0x01 and msg_id == 0x07 and payload_len == 92:
                        payload = frame[6:6 + 92]
                        pvt = self._parse_nav_pvt(payload)
                        if pvt is not None:
                            # Keep the best fix we've seen
                            if best_pvt is None or pvt['fix_type'] > best_pvt['fix_type']:
                                best_pvt = pvt
                            # If we have a 3D fix, we can stop early
                            if pvt['fix_type'] >= 3:
                                best_pvt = pvt
                                break

                if best_pvt and best_pvt['fix_type'] >= 3:
                    break

        except Exception as e:
            self.get_logger().warn(f'  [gps] Serial read error: {e}')
        finally:
            ser.close()

        if best_pvt is None:
            result.status = 'FAIL'
            result.reason = 'no NAV-PVT packets received'
            self.get_logger().error(f'  [gps] {result.reason}')
            return result

        # Fill result from best PVT
        result.fix_type = best_pvt['fix_type']
        fix_names = {0: 'none', 1: 'dead_reckoning', 2: '2D', 3: '3D', 4: '3D+DR', 5: 'time_only'}
        result.fix_type_str = fix_names.get(result.fix_type, f'unknown({result.fix_type})')
        result.num_sats = best_pvt['num_sv']
        result.h_acc_m = best_pvt['h_acc_m']
        result.v_acc_m = best_pvt['v_acc_m']
        result.pdop = best_pvt['pdop']
        result.latitude = best_pvt['lat']
        result.longitude = best_pvt['lon']

        # Evaluate
        issues = []
        if result.fix_type < 2:
            issues.append(f'no fix (type={result.fix_type_str})')
        elif result.fix_type < 3:
            issues.append(f'only 2D fix')
        if result.num_sats < self.gps_min_sats:
            issues.append(f'low sats ({result.num_sats} < {self.gps_min_sats})')
        if result.h_acc_m > self.gps_max_hacc_m:
            issues.append(f'poor accuracy ({result.h_acc_m:.1f}m > {self.gps_max_hacc_m}m)')
        if result.pdop > self.gps_max_pdop:
            issues.append(f'high PDOP ({result.pdop:.1f} > {self.gps_max_pdop})')

        if not issues:
            result.status = 'PASS'
        elif result.fix_type < 2:
            result.status = 'FAIL'
            result.reason = '; '.join(issues)
        else:
            result.status = 'WARN'
            result.reason = '; '.join(issues)

        self.get_logger().info(
            f'  [gps] {result.status} — fix={result.fix_type_str}, '
            f'sats={result.num_sats}, hAcc={result.h_acc_m:.1f}m, '
            f'pDOP={result.pdop:.1f}')
        return result

    def _send_ubx(self, ser, msg_class: int, msg_id: int, payload: bytes):
        """Send a UBX message with proper checksum (same logic as gps_driver.cpp)."""
        length = len(payload)
        header = bytes([0xB5, 0x62, msg_class, msg_id,
                        length & 0xFF, (length >> 8) & 0xFF])
        body = bytes([msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF]) + payload
        ck_a = 0
        ck_b = 0
        for b in body:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        packet = header + payload + bytes([ck_a, ck_b])
        ser.write(packet)

    def _configure_gps_ubx(self, ser, sync1: int, sync2: int):
        """Configure GPS receiver to output UBX-NAV-PVT (mirrors gps_driver.cpp)."""
        try:
            # Ensure write_timeout is set so we don't block forever
            if ser.write_timeout is None or ser.write_timeout > 2:
                ser.write_timeout = 2

            # 1. Enable UBX-NAV-PVT output (CFG-MSG: class=0x01, id=0x07, rate=1)
            self._send_ubx(ser, 0x06, 0x01, bytes([0x01, 0x07, 0x01]))

            # 2. Disable common NMEA messages to reduce serial traffic
            nmea_msgs = [
                (0xF0, 0x00),  # GGA
                (0xF0, 0x01),  # GLL
                (0xF0, 0x02),  # GSA
                (0xF0, 0x03),  # GSV
                (0xF0, 0x04),  # RMC
                (0xF0, 0x05),  # VTG
            ]
            for cls, mid in nmea_msgs:
                self._send_ubx(ser, 0x06, 0x01, bytes([cls, mid, 0x00]))

            # Flush any pending data
            ser.reset_input_buffer()
        except serial.SerialTimeoutException:
            self.get_logger().warn('  [gps] Config write timed out (device may not be a GPS)')
        except Exception as e:
            self.get_logger().warn(f'  [gps] Config write warning: {e}')

    def _parse_nav_pvt(self, payload: bytes) -> Optional[Dict[str, Any]]:
        """Parse a 92-byte UBX-NAV-PVT payload (same struct as gps_driver.cpp)."""
        if len(payload) < 92:
            return None
        try:
            # Unpack key fields (little-endian)
            # Offsets match UbxNavPvt struct in gps_driver.cpp
            fix_type = payload[20]
            flags = payload[21]
            num_sv = payload[23]
            lon_raw = struct.unpack_from('<i', payload, 24)[0]   # 1e-7 deg
            lat_raw = struct.unpack_from('<i', payload, 28)[0]   # 1e-7 deg
            h_acc_mm = struct.unpack_from('<I', payload, 40)[0]  # mm
            v_acc_mm = struct.unpack_from('<I', payload, 44)[0]  # mm
            pdop_raw = struct.unpack_from('<H', payload, 76)[0]  # 0.01

            gnss_fix_ok = bool(flags & 0x01)

            return {
                'fix_type': fix_type,
                'gnss_fix_ok': gnss_fix_ok,
                'num_sv': num_sv,
                'lat': lat_raw * 1e-7,
                'lon': lon_raw * 1e-7,
                'h_acc_m': h_acc_mm * 1e-3,
                'v_acc_m': v_acc_mm * 1e-3,
                'pdop': pdop_raw * 0.01,
            }
        except Exception:
            return None

    # ═══════════════════════════════════════════════════════════════
    # CHECK 7: Motor Test
    # ═══════════════════════════════════════════════════════════════

    def _left_status_cb(self, msg: ControllerStatus):
        self.left_vel = float(msg.vel_estimate)
        self.left_iq = float(msg.iq_setpoint)
        self.left_axis_state = int(msg.axis_state)
        self.left_errors = int(msg.active_errors)
        self.left_vel_time = time.time()

    def _right_status_cb(self, msg: ControllerStatus):
        self.right_vel = float(msg.vel_estimate)
        self.right_iq = float(msg.iq_setpoint)
        self.right_axis_state = int(msg.axis_state)
        self.right_errors = int(msg.active_errors)
        self.right_vel_time = time.time()

    def check_motors(self) -> MotorResult:
        """Run motor test: arm, maneuver, monitor, disarm."""
        result = MotorResult()

        if self.skip_motion_test:
            result.status = 'SKIP'
            result.reason = 'motion test skipped by parameter'
            self.get_logger().info(f'  [motors] {result.reason}')
            return result

        self.get_logger().info('  [motors] Setting up CAN and ODrive nodes...')

        # Step 1: Bring up CAN
        can_ok = False
        try:
            # Try setting up the CAN interface
            can_result = subprocess.run(
                ['sudo', 'ip', 'link', 'set', self.can_interface, 'up',
                 'type', 'can', 'bitrate', str(self.can_bitrate)],
                capture_output=True, text=True, timeout=5
            )
            if can_result.returncode == 0:
                can_ok = True
            else:
                # Maybe already up? Check if it exists
                check = subprocess.run(
                    ['ip', 'link', 'show', self.can_interface],
                    capture_output=True, text=True, timeout=5
                )
                if check.returncode == 0 and 'UP' in check.stdout.upper():
                    can_ok = True
                    self.get_logger().info(f'  [motors] CAN interface already up')
                else:
                    self.get_logger().error(
                        f'  [motors] CAN setup failed: {can_result.stderr.strip()}')
        except Exception as e:
            self.get_logger().error(f'  [motors] CAN setup error: {e}')

        if not can_ok:
            result.status = 'FAIL'
            result.reason = f'CAN interface {self.can_interface} not available (adapter connected?)'
            self.get_logger().error(f'  [motors] {result.reason}')
            return result

        # Step 2: Start ODrive CAN nodes
        left_cmd = [
            'ros2', 'run', 'odrive_can', 'odrive_can_node', '--ros-args',
            '-r', '__ns:=/left', '-r', '__node:=odrive_can_left_preflight',
            '-p', f'node_id:={self.left_node_id}',
            '-p', f'interface:={self.can_interface}'
        ]
        right_cmd = [
            'ros2', 'run', 'odrive_can', 'odrive_can_node', '--ros-args',
            '-r', '__ns:=/right', '-r', '__node:=odrive_can_right_preflight',
            '-p', f'node_id:={self.right_node_id}',
            '-p', f'interface:={self.can_interface}'
        ]
        left_proc = self.subproc.start(left_cmd, 'Left ODrive')
        right_proc = self.subproc.start(right_cmd, 'Right ODrive')
        if not left_proc or not right_proc:
            result.status = 'FAIL'
            result.reason = 'failed to start ODrive nodes'
            self._motor_cleanup()
            return result

        time.sleep(2)  # Let ODrive nodes initialize

        # Step 3: Create publishers and subscribers
        left_pub = self.create_publisher(ControlMessage, '/left/control_message', 10)
        right_pub = self.create_publisher(ControlMessage, '/right/control_message', 10)
        left_status_sub = self.create_subscription(
            ControllerStatus, '/left/controller_status',
            self._left_status_cb, qos_profile_sensor_data)
        right_status_sub = self.create_subscription(
            ControllerStatus, '/right/controller_status',
            self._right_status_cb, qos_profile_sensor_data)

        left_axis_client = self.create_client(AxisState, '/left/request_axis_state')
        right_axis_client = self.create_client(AxisState, '/right/request_axis_state')

        # Wait for services
        self.get_logger().info('  [motors] Waiting for ODrive services...')
        services_ok = True
        t0 = time.time()
        while time.time() - t0 < 10:
            if left_axis_client.service_is_ready() and right_axis_client.service_is_ready():
                break
            time.sleep(0.2)
        else:
            services_ok = False

        if not services_ok:
            result.status = 'FAIL'
            result.reason = 'ODrive services not available within 10s'
            self.get_logger().error(f'  [motors] {result.reason}')
            self._motor_cleanup(left_pub, right_pub, left_status_sub, right_status_sub,
                                left_axis_client, right_axis_client)
            return result

        # Wait for feedback
        self.get_logger().info('  [motors] Waiting for encoder feedback...')
        t0 = time.time()
        while time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.left_vel_time > 0 and self.right_vel_time > 0:
                break
        else:
            result.status = 'FAIL'
            result.reason = 'no encoder feedback received'
            self.get_logger().error(f'  [motors] {result.reason}')
            self._motor_cleanup(left_pub, right_pub, left_status_sub, right_status_sub,
                                left_axis_client, right_axis_client)
            return result

        # Step 4: Arm motors
        self.get_logger().info('  [motors] Arming motors...')
        armed = self._arm_motors(left_axis_client, right_axis_client)
        if not armed:
            result.status = 'FAIL'
            result.reason = 'failed to arm motors'
            self.get_logger().error(f'  [motors] {result.reason}')
            self._motor_cleanup(left_pub, right_pub, left_status_sub, right_status_sub,
                                left_axis_client, right_axis_client)
            return result
        result.armed_ok = True

        # Step 5: Run maneuvers
        v = self.motor_test_vel
        # Note: right ODrive positive direction is inverted (invert_right=true
        # in diff_drive_controller), so to turn the robot left we need
        # L=+v (forward), R=-v (which the right ODrive interprets as forward).
        maneuvers = [
            ('forward',     +v, +v),
            ('backward',    -v, -v),
            ('axial_left',  +v, -v),
            ('axial_right', -v, +v),
        ]

        for name, left_v, right_v in maneuvers:
            self.get_logger().info(f'  [motors] Maneuver: {name} (L={left_v:+.2f}, R={right_v:+.2f})')
            m_result = self._run_maneuver(
                name, left_v, right_v, left_pub, right_pub)
            result.maneuvers.append(m_result)

        # Step 6: Disarm
        self.get_logger().info('  [motors] Disarming motors...')
        result.disarmed_ok = self._disarm_motors(left_axis_client, right_axis_client)

        # Cleanup
        self._motor_cleanup(left_pub, right_pub, left_status_sub, right_status_sub,
                            left_axis_client, right_axis_client)

        # Evaluate
        all_ok = all(m.left_ok and m.right_ok and not m.errors_detected
                     for m in result.maneuvers)
        if all_ok:
            result.status = 'PASS'
        else:
            failed = [m.name for m in result.maneuvers
                      if not m.left_ok or not m.right_ok or m.errors_detected]
            result.status = 'FAIL'
            result.reason = f'failed maneuvers: {", ".join(failed)}'

        self.get_logger().info(f'  [motors] {result.status}')
        return result

    def _arm_motors(self, left_client, right_client) -> bool:
        """Arm both motors to CLOSED_LOOP_CONTROL."""
        req = AxisState.Request()
        req.axis_requested_state = 8  # CLOSED_LOOP_CONTROL

        future_l = left_client.call_async(req)
        future_r = right_client.call_async(req)

        # Wait for service responses
        t0 = time.time()
        while time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future_l.done() and future_r.done():
                break

        # Spin a few more times to receive updated status after arming
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        armed = (self.left_axis_state == 8 and self.right_axis_state == 8
                 and self.left_errors == 0 and self.right_errors == 0)

        if not armed:
            self.get_logger().warn(
                f'  [motors] Arming state: L={self.left_axis_state} R={self.right_axis_state} '
                f'L_err=0x{self.left_errors:X} R_err=0x{self.right_errors:X}')

        return armed

    def _disarm_motors(self, left_client, right_client) -> bool:
        """Disarm both motors to IDLE."""
        req = AxisState.Request()
        req.axis_requested_state = 1  # IDLE

        future_l = left_client.call_async(req)
        future_r = right_client.call_async(req)

        t0 = time.time()
        while time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future_l.done() and future_r.done():
                break
        time.sleep(0.3)
        return True

    def _run_maneuver(self, name: str, left_v: float, right_v: float,
                      left_pub, right_pub) -> ManeuverResult:
        """Execute a single maneuver and monitor feedback."""
        mr = ManeuverResult(name=name)
        left_vels = []
        right_vels = []
        left_currents = []
        right_currents = []

        # Send velocity commands for motor_test_duration
        t0 = time.time()
        while time.time() - t0 < self.motor_test_duration:
            msg_l = ControlMessage()
            msg_l.control_mode = 2   # VELOCITY_CONTROL
            msg_l.input_mode = 1     # PASSTHROUGH
            msg_l.input_vel = float(left_v)
            msg_l.input_torque = 0.0
            left_pub.publish(msg_l)

            msg_r = ControlMessage()
            msg_r.control_mode = 2   # VELOCITY_CONTROL
            msg_r.input_mode = 1     # PASSTHROUGH
            msg_r.input_vel = float(right_v)
            msg_r.input_torque = 0.0
            right_pub.publish(msg_r)

            rclpy.spin_once(self, timeout_sec=0.05)

            # Record feedback (skip first 0.5s for ramp-up)
            if time.time() - t0 > 0.5:
                left_vels.append(self.left_vel)
                right_vels.append(self.right_vel)
                left_currents.append(abs(self.left_iq))
                right_currents.append(abs(self.right_iq))

            # Check for errors
            if self.left_errors != 0 or self.right_errors != 0:
                mr.errors_detected = True
                mr.reason = (f'ODrive errors: L=0x{self.left_errors:X}, '
                             f'R=0x{self.right_errors:X}')

        # Stop command — send zero and wait for wheels to actually stop
        stopped = False
        t0 = time.time()
        while time.time() - t0 < self.motor_stop_duration:
            msg = ControlMessage()
            msg.control_mode = 2   # VELOCITY_CONTROL
            msg.input_mode = 1     # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            left_pub.publish(msg)
            right_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            # Check if both wheels have actually stopped
            if (abs(self.left_vel) < 0.02 and abs(self.right_vel) < 0.02
                    and time.time() - t0 > 0.2):
                stopped = True
                break
        # Extra settle time after stop to let mechanical vibration die down
        if stopped:
            time.sleep(0.2)
        else:
            time.sleep(0.3)

        # Evaluate
        if left_vels:
            mr.left_vel_avg = float(np.mean(left_vels))
            mr.left_current_max = float(np.max(left_currents)) if left_currents else 0.0
        if right_vels:
            mr.right_vel_avg = float(np.mean(right_vels))
            mr.right_current_max = float(np.max(right_currents)) if right_currents else 0.0

        # Check velocity direction matches command
        thresh = self.motor_vel_threshold
        if left_v > 0:
            mr.left_ok = mr.left_vel_avg > thresh
        elif left_v < 0:
            mr.left_ok = mr.left_vel_avg < -thresh
        else:
            mr.left_ok = abs(mr.left_vel_avg) < thresh

        if right_v > 0:
            mr.right_ok = mr.right_vel_avg > thresh
        elif right_v < 0:
            mr.right_ok = mr.right_vel_avg < -thresh
        else:
            mr.right_ok = abs(mr.right_vel_avg) < thresh

        # Current check — warn but don't hard-fail unless extreme
        if mr.left_current_max > self.motor_max_current:
            mr.reason += f' L_current_high({mr.left_current_max:.2f}A>{self.motor_max_current:.0f}A)'
        if mr.right_current_max > self.motor_max_current:
            mr.reason += f' R_current_high({mr.right_current_max:.2f}A>{self.motor_max_current:.0f}A)'

        if mr.errors_detected:
            mr.left_ok = False
            mr.right_ok = False

        self.get_logger().info(
            f'    {name}: L_vel={mr.left_vel_avg:+.3f} R_vel={mr.right_vel_avg:+.3f} '
            f'L_Iq={mr.left_current_max:.2f}A R_Iq={mr.right_current_max:.2f}A '
            f'L_ok={mr.left_ok} R_ok={mr.right_ok}')
        return mr

    def _motor_cleanup(self, left_pub=None, right_pub=None,
                       left_sub=None, right_sub=None,
                       left_client=None, right_client=None):
        """Clean up motor test resources."""
        # Send zero commands
        if left_pub and right_pub:
            msg = ControlMessage()
            msg.control_mode = 2   # VELOCITY_CONTROL
            msg.input_mode = 1     # PASSTHROUGH
            msg.input_vel = 0.0
            msg.input_torque = 0.0
            for _ in range(5):
                left_pub.publish(msg)
                right_pub.publish(msg)
                time.sleep(0.02)

        # Destroy ROS interfaces
        for obj in [left_pub, right_pub, left_sub, right_sub, left_client, right_client]:
            if obj is not None:
                try:
                    self.destroy_publisher(obj) if hasattr(obj, 'publish') else None
                    self.destroy_subscription(obj) if hasattr(obj, 'msg_type') else None
                    self.destroy_client(obj) if hasattr(obj, 'call_async') else None
                except Exception:
                    pass

        # Stop ODrive nodes
        self.subproc.stop_all()

        # Tear down CAN
        try:
            subprocess.run(
                ['sudo', 'ip', 'link', 'set', self.can_interface, 'down'],
                capture_output=True, timeout=5
            )
            self.get_logger().info('  [motors] CAN interface down')
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════════════
    # ORCHESTRATOR
    # ═══════════════════════════════════════════════════════════════

    def run_all_checks(self) -> Dict[str, Any]:
        """Run all preflight checks and return results."""
        start_time = time.time()
        timestamp = datetime.now()
        folder_name = timestamp.strftime('%Y%m%d_%H%M%S')

        # Create output directory
        output_dir = os.path.join(self.report_base_dir, folder_name)
        os.makedirs(output_dir, exist_ok=True)

        results = {}

        # ── Phase 1: Camera checks ──
        self.get_logger().info('')
        self.get_logger().info('PHASE 1: CAMERA CHECKS')
        self.get_logger().info('-' * 40)
        results['left_rgb'] = self.check_rgb_camera('left_rgb', self.left_device, output_dir)
        results['right_rgb'] = self.check_rgb_camera('right_rgb', self.right_device, output_dir)
        results['thermal'] = self.check_thermal_camera(output_dir)

        # ── Phase 2: LiDAR check ──
        self.get_logger().info('')
        self.get_logger().info('PHASE 2: LIDAR CHECK')
        self.get_logger().info('-' * 40)
        results['lidar'] = self.check_lidar()

        # ── Phase 3: RF check ──
        self.get_logger().info('')
        self.get_logger().info('PHASE 3: RF CONNECTION')
        self.get_logger().info('-' * 40)
        results['rf'] = self.check_rf()

        # ── Phase 4: GPS check ──
        self.get_logger().info('')
        self.get_logger().info('PHASE 4: GPS SIGNAL')
        self.get_logger().info('-' * 40)
        results['gps'] = self.check_gps()

        # ── Phase 5: Motor test ──
        self.get_logger().info('')
        self.get_logger().info('PHASE 5: MOTOR TEST')
        self.get_logger().info('-' * 40)
        results['motors'] = self.check_motors()

        # ── Generate report ──
        duration_s = time.time() - start_time

        # Determine overall status
        statuses = []
        for key, val in results.items():
            if isinstance(val, (CameraResult, ThermalResult, LidarResult,
                                RfResult, GpsResult, MotorResult)):
                statuses.append(val.status)

        if any(s == 'FAIL' for s in statuses):
            overall = 'FAIL'
        elif any(s == 'WARN' for s in statuses):
            overall = 'WARN'
        elif all(s in ('PASS', 'SKIP', 'NOT_RUN') for s in statuses):
            overall = 'PASS'
        else:
            overall = 'UNKNOWN'

        report = {
            'timestamp': timestamp.isoformat(),
            'folder': folder_name,
            'folder_path': output_dir,
            'duration_s': round(duration_s, 1),
            'overall': overall,
            'checks': {}
        }

        for key, val in results.items():
            report['checks'][key] = asdict(val)

        # Save JSON report
        report_path = os.path.join(output_dir, 'preflight_report.json')
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        # Update 'latest' symlink
        latest_link = os.path.join(self.report_base_dir, 'latest')
        try:
            if os.path.islink(latest_link):
                os.unlink(latest_link)
            os.symlink(output_dir, latest_link)
        except Exception as e:
            self.get_logger().warn(f'Could not update latest symlink: {e}')

        # Publish to latched topic
        report_msg = String()
        report_msg.data = json.dumps(report, default=str)
        self.report_pub.publish(report_msg)

        # Prune old reports
        self._prune_old_reports()

        # Print summary
        self._print_summary(report, results)

        return report

    def _prune_old_reports(self):
        """Remove old report folders beyond max_reports."""
        try:
            folders = sorted(glob.glob(os.path.join(self.report_base_dir, '20*')))
            if len(folders) > self.max_reports:
                for old in folders[:-self.max_reports]:
                    shutil.rmtree(old, ignore_errors=True)
                    self.get_logger().info(f'Pruned old report: {os.path.basename(old)}')
        except Exception:
            pass

    def _print_summary(self, report: Dict, results: Dict):
        """Print a formatted summary table."""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('         ROOFUS V2 PREFLIGHT REPORT')
        self.get_logger().info('=' * 60)

        check_display = [
            ('Left RGB Camera',  results.get('left_rgb')),
            ('Right RGB Camera', results.get('right_rgb')),
            ('Thermal Camera',   results.get('thermal')),
            ('LiDAR (MID360)',   results.get('lidar')),
            ('RF Link',          results.get('rf')),
            ('GPS Signal',       results.get('gps')),
            ('Motor Test',       results.get('motors')),
        ]

        for label, r in check_display:
            if r is None:
                continue
            status = r.status
            detail = ''

            if isinstance(r, CameraResult):
                detail = (f'sharp={r.sharpness:.0f} edge={r.edge_density:.1f}% '
                          f'entropy={r.entropy:.1f}')
            elif isinstance(r, ThermalResult):
                detail = f'range={r.temp_range_c:.1f}C'
            elif isinstance(r, LidarResult):
                detail = f'{r.rate_hz:.1f}Hz, {r.avg_points}pts'
            elif isinstance(r, RfResult):
                detail = f'{r.avg_rtt_ms:.0f}ms, {r.packet_loss_pct:.0f}% loss'
            elif isinstance(r, GpsResult):
                detail = f'{r.fix_type_str}, {r.num_sats}sats, {r.h_acc_m:.1f}m'
            elif isinstance(r, MotorResult):
                ok_count = sum(1 for m in r.maneuvers if m.left_ok and m.right_ok)
                detail = f'{ok_count}/{len(r.maneuvers)} maneuvers OK'

            # Status indicator
            if status == 'PASS':
                indicator = 'PASS'
            elif status == 'WARN':
                indicator = 'WARN'
            elif status == 'FAIL':
                indicator = 'FAIL'
            elif status == 'SKIP':
                indicator = 'SKIP'
            else:
                indicator = '----'

            self.get_logger().info(f'  {label:<22s} : {indicator:<4s}  ({detail})')

            if r.reason:
                self.get_logger().info(f'    -> {r.reason}')

        self.get_logger().info('-' * 60)
        self.get_logger().info(f'  Overall: {report["overall"]}  '
                               f'(completed in {report["duration_s"]:.1f}s)')
        self.get_logger().info(f'  Report:  {report["folder_path"]}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = StartupPreflight()

    try:
        report = node.run_all_checks()
        # Exit with 0 always (advisory mode)
        # The overall status is in the report for consumers to check
        rclpy.shutdown()
        sys.exit(0)
    except KeyboardInterrupt:
        node.get_logger().info('Preflight interrupted')
        node.subproc.stop_all()
        rclpy.shutdown()
        sys.exit(1)
    except Exception as e:
        node.get_logger().error(f'Preflight error: {e}')
        import traceback
        traceback.print_exc()
        node.subproc.stop_all()
        rclpy.shutdown()
        sys.exit(2)


if __name__ == '__main__':
    main()
