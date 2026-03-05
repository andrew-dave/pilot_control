#!/usr/bin/env python3
"""
Standalone FAST-LIO covariance auto-tuner with bounded maneuver sequences.

What it does:
1) Writes candidate covariance values into FAST-LIO YAML.
2) Optionally restarts the pipeline (using user-provided stop/start shell commands).
3) Runs a bounded command sequence (L/R/FWD/BWD and combined maneuvers).
4) Scores drift from odometry.
5) Iteratively searches for better covariances.

Notes:
- This script is intentionally conservative in motion limits to keep the robot
  inside a small area.
- It assumes a running ROS2 graph with a node consuming /cmd_vel.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


@dataclass
class Segment:
    block: str
    name: str
    linear_x: float
    angular_z: float
    duration_sec: float


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class BoundedManeuverRunner(Node):
    def __init__(self, cmd_topic: str, odom_topic: str, context: Context) -> None:
        super().__init__("kf_covariance_autotune_runner", context=context)

        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos_pub)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_sub
        )
        self.last_pose: Optional[Pose2D] = None
        self.last_odom_wall_time: float = 0.0

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_pose = Pose2D(
            x=float(p.x),
            y=float(p.y),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.last_odom_wall_time = time.time()

    def clear_odom_cache(self) -> None:
        self.last_pose = None

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        tw = Twist()
        tw.linear.x = float(linear_x)
        tw.angular.z = float(angular_z)
        self.cmd_pub.publish(tw)


def build_bounded_sequence() -> List[Segment]:
    """
    Sequence includes L/R/FWD/BWD and combinations.
    Designed to keep motion bounded and include rotation->translation transitions.
    """
    seq: List[Segment] = []

    # Block 1: in-place left/right cancellation
    seq += [
        Segment("spin_cancel", "left_spin", 0.0, 0.85, 1.0),
        Segment("spin_cancel", "right_spin", 0.0, -0.85, 1.0),
    ]

    # Block 2: forward/backward cancellation
    seq += [
        Segment("fb_cancel", "forward", 0.20, 0.0, 1.3),
        Segment("fb_cancel", "backward", -0.20, 0.0, 1.3),
    ]

    # Block 3: rotation -> translation coupling stress
    seq += [
        Segment("rot_trans_a", "left_spin", 0.0, 0.90, 0.9),
        Segment("rot_trans_a", "forward", 0.22, 0.0, 1.0),
        Segment("rot_trans_a", "right_spin", 0.0, -0.90, 0.9),
        Segment("rot_trans_a", "backward", -0.22, 0.0, 1.0),
    ]

    # Block 4: mirrored coupling stress
    seq += [
        Segment("rot_trans_b", "right_spin", 0.0, -0.90, 0.9),
        Segment("rot_trans_b", "forward", 0.22, 0.0, 1.0),
        Segment("rot_trans_b", "left_spin", 0.0, 0.90, 0.9),
        Segment("rot_trans_b", "backward", -0.22, 0.0, 1.0),
    ]

    # Block 5: compact figure-8 style arcs (combined maneuvers)
    seq += [
        Segment("figure8", "arc_left_fwd", 0.18, 0.55, 1.4),
        Segment("figure8", "arc_right_fwd", 0.18, -0.55, 1.4),
        Segment("figure8", "arc_right_bwd", -0.18, -0.55, 1.1),
        Segment("figure8", "arc_left_bwd", -0.18, 0.55, 1.1),
    ]

    return seq


def wait_for_odom(
    runner: BoundedManeuverRunner,
    executor: SingleThreadedExecutor,
    timeout_sec: float = 10.0,
    min_odom_wall_time: float = 0.0,
) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        executor.spin_once(timeout_sec=0.1)
        if (
            runner.last_pose is not None
            and runner.last_odom_wall_time > float(min_odom_wall_time)
        ):
            return True
    return False


def run_maneuver_trial(
    runner: BoundedManeuverRunner,
    executor: SingleThreadedExecutor,
    sequence: List[Segment],
    max_radius_m: float,
    command_rate_hz: float,
    settle_before_sec: float,
    settle_after_sec: float,
) -> Dict[str, float]:
    if runner.last_pose is None:
        raise RuntimeError("No odometry available before trial start.")

    hz = max(5.0, command_rate_hz)
    dt = 1.0 / hz

    # Pre-trial settle (stationary) for stable bias / baseline.
    t0 = time.time()
    while time.time() - t0 < settle_before_sec:
        runner.publish_cmd(0.0, 0.0)
        executor.spin_once(timeout_sec=dt)

    if runner.last_pose is None:
        raise RuntimeError("Odometry lost during pre-trial settle.")
    origin = runner.last_pose

    block_starts: Dict[str, Pose2D] = {}
    block_ends: Dict[str, Pose2D] = {}
    current_block = ""
    max_radius_seen = 0.0
    out_of_bounds = 0

    for seg in sequence:
        if seg.block != current_block:
            current_block = seg.block
            if runner.last_pose is not None:
                block_starts[current_block] = runner.last_pose

        seg_start = time.time()
        while time.time() - seg_start < seg.duration_sec:
            runner.publish_cmd(seg.linear_x, seg.angular_z)
            executor.spin_once(timeout_sec=dt)

            if runner.last_pose is not None:
                dx = runner.last_pose.x - origin.x
                dy = runner.last_pose.y - origin.y
                radius = math.hypot(dx, dy)
                if radius > max_radius_seen:
                    max_radius_seen = radius
                if radius > max_radius_m:
                    out_of_bounds += 1
                    runner.publish_cmd(0.0, 0.0)
                    break

        if runner.last_pose is not None:
            block_ends[seg.block] = runner.last_pose
        if out_of_bounds > 0:
            break

    # Post-trial settle.
    t1 = time.time()
    while time.time() - t1 < settle_after_sec:
        runner.publish_cmd(0.0, 0.0)
        executor.spin_once(timeout_sec=dt)

    if runner.last_pose is None:
        raise RuntimeError("Odometry lost after trial.")

    final = runner.last_pose
    final_dist = math.hypot(final.x - origin.x, final.y - origin.y)
    final_yaw = abs(wrap_angle(final.yaw - origin.yaw))

    # Block closure metrics emphasize rotation->translation induced residuals.
    def block_error(block_name: str) -> Tuple[float, float]:
        if block_name not in block_starts or block_name not in block_ends:
            return (5.0, math.pi)  # large penalty if missing
        s = block_starts[block_name]
        e = block_ends[block_name]
        return (math.hypot(e.x - s.x, e.y - s.y), abs(wrap_angle(e.yaw - s.yaw)))

    e_spin, yaw_spin = block_error("spin_cancel")
    e_fb, yaw_fb = block_error("fb_cancel")
    e_rta, yaw_rta = block_error("rot_trans_a")
    e_rtb, yaw_rtb = block_error("rot_trans_b")
    e_fig8, yaw_fig8 = block_error("figure8")

    # Weighted score (lower is better).
    score = (
        2.5 * e_rta
        + 2.5 * e_rtb
        + 1.5 * e_spin
        + 1.2 * e_fb
        + 1.0 * e_fig8
        + 1.5 * final_dist
        + 0.5 * (yaw_spin + yaw_fb + yaw_rta + yaw_rtb + yaw_fig8)
        + 0.8 * final_yaw
    )
    if out_of_bounds > 0:
        score += 100.0 + 10.0 * out_of_bounds

    runner.publish_cmd(0.0, 0.0)
    return {
        "score": float(score),
        "final_dist_m": float(final_dist),
        "final_yaw_rad": float(final_yaw),
        "max_radius_m": float(max_radius_seen),
        "oob_events": float(out_of_bounds),
        "e_spin_m": float(e_spin),
        "e_fb_m": float(e_fb),
        "e_rta_m": float(e_rta),
        "e_rtb_m": float(e_rtb),
        "e_fig8_m": float(e_fig8),
    }


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid YAML root in {path}")
    return data


def write_yaml(path: Path, data: dict) -> None:
    # Write atomically: avoid leaving a truncated/invalid YAML if interrupted.
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(tmp_fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())

        # Validate the generated YAML before replacing the live config.
        with tmp_path.open("r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f)
        if not isinstance(loaded, dict):
            raise RuntimeError(f"Generated YAML root is invalid for {path}")

        os.replace(tmp_path, path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def set_covariances(data: dict, params: Dict[str, float]) -> None:
    root = data.get("/**", {}).get("ros__parameters", {})
    mapping = root.get("mapping", {})
    for k in ("acc_cov", "gyr_cov", "b_acc_cov", "b_gyr_cov"):
        if k in params:
            mapping[k] = float(params[k])
    if "time_sync_en" in params:
        root.setdefault("common", {})["time_sync_en"] = bool(params["time_sync_en"])
    if "time_offset_lidar_to_imu" in params:
        root.setdefault("common", {})["time_offset_lidar_to_imu"] = float(
            params["time_offset_lidar_to_imu"]
        )
    root["mapping"] = mapping
    data.setdefault("/**", {})["ros__parameters"] = root


def read_current_covariances(data: dict) -> Dict[str, float]:
    root = data.get("/**", {}).get("ros__parameters", {})
    mapping = root.get("mapping", {})
    return {
        "acc_cov": float(mapping.get("acc_cov", 0.02)),
        "gyr_cov": float(mapping.get("gyr_cov", 2.5e-5)),
        "b_acc_cov": float(mapping.get("b_acc_cov", 1.5e-6)),
        "b_gyr_cov": float(mapping.get("b_gyr_cov", 5e-8)),
    }


def clamp_params(p: Dict[str, float]) -> Dict[str, float]:
    bounds = {
        "acc_cov": (0.008, 0.05),
        "gyr_cov": (1.0e-5, 8.0e-5),
        "b_acc_cov": (3.0e-7, 2.0e-5),
        "b_gyr_cov": (5.0e-9, 5.0e-7),
    }
    q = dict(p)
    for k, (lo, hi) in bounds.items():
        q[k] = float(min(hi, max(lo, q[k])))
    return q


def run_shell(cmd: str, timeout_sec: float) -> int:
    try:
        proc = subprocess.run(
            cmd,
            shell=True,
            text=True,
            timeout=timeout_sec,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if proc.stdout:
            print(proc.stdout.strip())
        return int(proc.returncode)
    except subprocess.TimeoutExpired:
        print(f"[WARN] Command timeout ({timeout_sec:.1f}s): {cmd}")
        return 124


def stop_cmd_may_kill_self(cmd: str) -> bool:
    """
    Best-effort guard against stop commands that match this autotune process
    via --launch-cmd / --stop-cmd arguments.
    """
    s = (cmd or "").lower()
    if "pkill" not in s or "-f" not in s:
        return False
    argv = " ".join(sys.argv).lower()
    risky_markers = [
        "robot_complete.launch.py",
        "ros2 launch pilot_control",
    ]
    return any(marker in s and marker in argv for marker in risky_markers)


def run_shell_background(cmd: str, log_path: Optional[Path] = None) -> subprocess.Popen:
    stdout_target = subprocess.DEVNULL
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stdout_target = open(log_path, "a", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        shell=True,
        stdout=stdout_target,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return proc


def stop_background_process(proc: Optional[subprocess.Popen], grace_sec: float = 8.0) -> None:
    if proc is None:
        return
    try:
        if proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGTERM)
        t0 = time.time()
        while time.time() - t0 < grace_sec:
            if proc.poll() is not None:
                return
            time.sleep(0.2)
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    except Exception as ex:
        print(f"[WARN] Failed to stop background process {proc.pid}: {ex}")


def maybe_restart_pipeline(
    stop_cmd: str,
    launch_cmd: str,
    stop_timeout: float,
    launch_timeout: float,
    post_launch_sleep_sec: float,
    launch_log_path: Optional[Path],
) -> Optional[subprocess.Popen]:
    if stop_cmd.strip():
        if stop_cmd_may_kill_self(stop_cmd):
            print(
                "[WARN] Skipping stop-cmd because it may terminate this autotune "
                "process via pkill -f pattern match."
            )
        else:
            if "pkill -f robot_complete.launch.py" in stop_cmd:
                print(
                    "[WARN] stop-cmd pattern may match this autotune process. "
                    "Prefer: pkill -f '[r]obot_complete.launch.py' || true"
                )
            rc = run_shell(stop_cmd, timeout_sec=stop_timeout)
            if rc != 0:
                print(f"[WARN] stop command returned {rc}")
    if launch_cmd.strip():
        # Launch commands are often long-running (e.g. ros2 launch ...).
        # Start them in a detached background process and then wait for odometry.
        proc = run_shell_background(launch_cmd, log_path=launch_log_path)
        print(f"[INFO] launch command started in background (pid={proc.pid})")
        if post_launch_sleep_sec > 0.0:
            time.sleep(post_launch_sleep_sec)
        if proc.poll() is not None:
            print(
                f"[WARN] launch process exited early with code {proc.returncode}. "
                f"Check log: {launch_log_path}"
            )
        return proc
    return None


def scan_launch_log_for_known_errors(log_path: Optional[Path], start_offset: int) -> List[str]:
    if log_path is None or (not log_path.exists()):
        return []
    try:
        with log_path.open("r", encoding="utf-8", errors="ignore") as f:
            f.seek(max(0, int(start_offset)))
            chunk = f.read(300000).lower()
    except Exception:
        return []

    hints: List[str] = []
    if "cannot find device \"can0\"" in chunk:
        hints.append("CAN interface can0 missing.")
    if "failed to initialize socket can interface: can0" in chunk:
        hints.append("ODrive CAN node failed to bind can0.")
    if "odrive services not available after waiting" in chunk:
        hints.append("diff_drive_controller could not find ODrive services.")
    if "device '/dev/v4l/by-id/" in chunk and "is busy" in chunk:
        hints.append("Camera device busy (likely stale process still running).")
    if "zenoh_bridge_dds-1" in chunk and "process has died" in chunk:
        hints.append("zenoh bridge process died during startup.")
    return hints


def format_params(p: Dict[str, float]) -> str:
    return (
        f"acc_cov={p['acc_cov']:.7g}, "
        f"gyr_cov={p['gyr_cov']:.7g}, "
        f"b_acc_cov={p['b_acc_cov']:.7g}, "
        f"b_gyr_cov={p['b_gyr_cov']:.7g}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Auto-tune FAST-LIO covariances with bounded maneuvers."
    )
    parser.add_argument(
        "--config",
        default="/home/roofus/pilot_ws/src/pilot_control/config/fastlio_mid360.yaml",
        help="Path to FAST-LIO YAML config.",
    )
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/Odometry_tilt_corrected_diff")
    parser.add_argument("--max-radius", type=float, default=1.6)
    parser.add_argument("--command-rate", type=float, default=20.0)
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument(
        "--step-scale", type=float, default=0.35, help="Initial multiplicative step size."
    )
    parser.add_argument(
        "--decay", type=float, default=0.8, help="Step scale decay per outer iteration."
    )
    parser.add_argument("--settle-before", type=float, default=4.0)
    parser.add_argument("--settle-after", type=float, default=2.0)
    parser.add_argument(
        "--launch-cmd",
        default="",
        help="Shell command that starts your pipeline (optional, long-running is fine).",
    )
    parser.add_argument(
        "--stop-cmd",
        default="",
        help="Shell command to stop previous pipeline before relaunch (optional).",
    )
    parser.add_argument("--launch-timeout", type=float, default=20.0)
    parser.add_argument("--stop-timeout", type=float, default=20.0)
    parser.add_argument(
        "--post-launch-sleep",
        type=float,
        default=2.5,
        help="Seconds to wait after background launch before odometry wait.",
    )
    parser.add_argument(
        "--odom-wait-timeout",
        type=float,
        default=25.0,
        help="Seconds to wait for odometry after each (re)launch.",
    )
    parser.add_argument(
        "--launch-log",
        default="",
        help="Path to append launch stdout/stderr (recommended for debugging).",
    )
    parser.add_argument(
        "--launch-retries",
        type=int,
        default=1,
        help="Additional relaunch attempts when fresh odometry is not received.",
    )
    parser.add_argument(
        "--set-time-sync-true",
        action="store_true",
        help="Force common.time_sync_en=true during autotuning.",
    )
    parser.add_argument(
        "--results-csv",
        default="",
        help="CSV output path. Defaults to scripts/test_scripts/kf_autotune_results_<ts>.csv",
    )
    args = parser.parse_args()

    cfg_path = Path(args.config).expanduser().resolve()
    if not cfg_path.exists():
        print(f"[ERROR] Config file not found: {cfg_path}")
        return 2

    if not args.results_csv:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.results_csv = str(
            cfg_path.parent.parent / "scripts" / "test_scripts" / f"kf_autotune_results_{ts}.csv"
        )
    csv_path = Path(args.results_csv).expanduser().resolve()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    launch_log_path = (
        Path(args.launch_log).expanduser().resolve()
        if args.launch_log
        else (csv_path.parent / "kf_autotune_launch.log")
    )

    backup_path = cfg_path.with_suffix(cfg_path.suffix + ".autotune_backup")
    if not backup_path.exists():
        shutil.copy2(cfg_path, backup_path)
        print(f"[INFO] Backup created: {backup_path}")

    data0 = load_yaml(cfg_path)
    current = clamp_params(read_current_covariances(data0))
    sequence = build_bounded_sequence()

    print(f"[INFO] Starting params: {format_params(current)}")
    print(f"[INFO] Results CSV: {csv_path}")
    if args.launch_cmd:
        print("[INFO] launch-cmd is enabled.")
    else:
        print(
            "[WARN] launch-cmd is empty. Ensure pipeline is already running and "
            "restarts are handled externally."
        )

    fields = [
        "trial",
        "score",
        "acc_cov",
        "gyr_cov",
        "b_acc_cov",
        "b_gyr_cov",
        "final_dist_m",
        "final_yaw_rad",
        "max_radius_m",
        "oob_events",
        "e_spin_m",
        "e_fb_m",
        "e_rta_m",
        "e_rtb_m",
        "e_fig8_m",
    ]
    context = Context()
    executor: Optional[SingleThreadedExecutor] = None
    runner: Optional[BoundedManeuverRunner] = None
    launched_proc: Optional[subprocess.Popen] = None
    best = dict(current)
    best_score = float("inf")
    step_scale = max(0.08, args.step_scale)
    interrupted = False
    completed = False
    try:
        rclpy.init(args=None, context=context)
        executor = SingleThreadedExecutor(context=context)
        runner = BoundedManeuverRunner(args.cmd_topic, args.odom_topic, context=context)
        executor.add_node(runner)

        with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
            writer = csv.DictWriter(fcsv, fieldnames=fields)
            writer.writeheader()

            param_order = ["acc_cov", "gyr_cov", "b_acc_cov", "b_gyr_cov"]
            trial_count = 0

            while trial_count < args.trials:
                candidates: List[Dict[str, float]] = []
                candidates.append(dict(best))
                for k in param_order:
                    up = dict(best)
                    dn = dict(best)
                    up[k] = up[k] * (1.0 + step_scale)
                    dn[k] = dn[k] * (1.0 - step_scale)
                    candidates.append(clamp_params(up))
                    candidates.append(clamp_params(dn))

                tested_this_round = []
                for cand in candidates:
                    if trial_count >= args.trials:
                        break

                    trial_count += 1
                    cfg = load_yaml(cfg_path)
                    params_to_set = dict(cand)
                    if args.set_time_sync_true:
                        params_to_set["time_sync_en"] = True
                    set_covariances(cfg, params_to_set)
                    write_yaml(cfg_path, cfg)

                    print(f"\n[TRIAL {trial_count}] {format_params(cand)}")
                    prev_odom_time = runner.last_odom_wall_time
                    runner.clear_odom_cache()
                    odom_ready = False

                    # Preferred restart path: if launch-cmd is provided, manage lifecycle.
                    if args.launch_cmd.strip():
                        retries = max(0, int(args.launch_retries))
                        for attempt in range(retries + 1):
                            log_start = (
                                int(launch_log_path.stat().st_size)
                                if launch_log_path.exists()
                                else 0
                            )
                            stop_background_process(launched_proc, grace_sec=8.0)
                            launched_proc = maybe_restart_pipeline(
                                args.stop_cmd,
                                args.launch_cmd,
                                args.stop_timeout,
                                args.launch_timeout,
                                args.post_launch_sleep,
                                launch_log_path,
                            )
                            odom_ready = wait_for_odom(
                                runner,
                                executor,
                                timeout_sec=args.odom_wait_timeout,
                                min_odom_wall_time=prev_odom_time,
                            )
                            if odom_ready:
                                break
                            hints = scan_launch_log_for_known_errors(
                                launch_log_path, start_offset=log_start
                            )
                            if hints:
                                print(
                                    "[WARN] Launch health hints: "
                                    + " | ".join(sorted(set(hints)))
                                )
                            if attempt < retries:
                                print(
                                    f"[WARN] No fresh odometry after launch attempt "
                                    f"{attempt + 1}/{retries + 1}; retrying..."
                                )
                    else:
                        maybe_restart_pipeline(
                            args.stop_cmd,
                            "",
                            args.stop_timeout,
                            args.launch_timeout,
                            args.post_launch_sleep,
                            launch_log_path,
                        )
                        odom_ready = wait_for_odom(
                            runner,
                            executor,
                            timeout_sec=args.odom_wait_timeout,
                            min_odom_wall_time=prev_odom_time,
                        )

                    if not odom_ready:
                        print("[WARN] No odometry received; assigning penalty.")
                        metrics = {
                            "score": 9999.0,
                            "final_dist_m": 99.0,
                            "final_yaw_rad": math.pi,
                            "max_radius_m": 99.0,
                            "oob_events": 9.0,
                            "e_spin_m": 99.0,
                            "e_fb_m": 99.0,
                            "e_rta_m": 99.0,
                            "e_rtb_m": 99.0,
                            "e_fig8_m": 99.0,
                        }
                    else:
                        metrics = run_maneuver_trial(
                            runner=runner,
                            executor=executor,
                            sequence=sequence,
                            max_radius_m=args.max_radius,
                            command_rate_hz=args.command_rate,
                            settle_before_sec=args.settle_before,
                            settle_after_sec=args.settle_after,
                        )

                    row = {
                        "trial": trial_count,
                        "acc_cov": cand["acc_cov"],
                        "gyr_cov": cand["gyr_cov"],
                        "b_acc_cov": cand["b_acc_cov"],
                        "b_gyr_cov": cand["b_gyr_cov"],
                        **metrics,
                    }
                    writer.writerow(row)
                    fcsv.flush()
                    os.fsync(fcsv.fileno())

                    print(
                        f"[TRIAL {trial_count}] score={metrics['score']:.5f}, "
                        f"final_dist={metrics['final_dist_m']:.4f} m, "
                        f"max_radius={metrics['max_radius_m']:.3f} m"
                    )

                    tested_this_round.append((cand, float(metrics["score"])))
                    if metrics["score"] < best_score:
                        best_score = float(metrics["score"])
                        best = dict(cand)
                        print(
                            f"[BEST] Updated: score={best_score:.5f}  {format_params(best)}"
                        )

                # Keep best from this local neighborhood and shrink step.
                tested_this_round.sort(key=lambda x: x[1])
                if tested_this_round:
                    best = dict(tested_this_round[0][0])
                step_scale = max(0.08, step_scale * args.decay)
                print(f"[INFO] Next step-scale: {step_scale:.4f}")

        # Write best back to config.
        cfgf = load_yaml(cfg_path)
        final_set = dict(best)
        if args.set_time_sync_true:
            final_set["time_sync_en"] = True
        set_covariances(cfgf, final_set)
        write_yaml(cfg_path, cfgf)
        completed = True
    except KeyboardInterrupt:
        interrupted = True
        print("\n[INFO] Ctrl+C received. Shutting down cleanly...")
    finally:
        # Persist best-known parameters even on interruption.
        try:
            cfgf = load_yaml(cfg_path)
            final_set = dict(best)
            if args.set_time_sync_true:
                final_set["time_sync_en"] = True
            set_covariances(cfgf, final_set)
            write_yaml(cfg_path, cfgf)
        except Exception as ex:
            print(f"[WARN] Failed to write final/best config during shutdown: {ex}")

        try:
            if runner is not None:
                runner.publish_cmd(0.0, 0.0)
        except Exception:
            pass

        stop_background_process(launched_proc, grace_sec=8.0)
        if args.stop_cmd.strip():
            if stop_cmd_may_kill_self(args.stop_cmd):
                print(
                    "[WARN] Skipping stop-cmd on shutdown because it may "
                    "terminate this autotune process."
                )
            else:
                rc = run_shell(args.stop_cmd, timeout_sec=args.stop_timeout)
                if rc != 0:
                    print(f"[WARN] stop command returned {rc} during shutdown.")

        if executor is not None and runner is not None:
            try:
                executor.remove_node(runner)
            except Exception:
                pass
            try:
                runner.destroy_node()
            except Exception:
                pass
        if context.ok():
            rclpy.shutdown(context=context)

    if completed:
        print("\n=== AUTOTUNE COMPLETE ===")
        print(f"Best score: {best_score:.5f}")
        print(f"Best params: {format_params(best)}")
        print(f"Config updated: {cfg_path}")
        print(f"CSV results: {csv_path}")
        print(f"Backup file: {backup_path}")
        return 0

    print("\n=== AUTOTUNE STOPPED EARLY ===")
    print(f"Best-so-far params written: {format_params(best)}")
    print(f"Config updated: {cfg_path}")
    print(f"CSV results: {csv_path}")
    print(f"Backup file: {backup_path}")
    if interrupted:
        return 130
    return 1


if __name__ == "__main__":
    sys.exit(main())
