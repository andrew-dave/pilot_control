#!/usr/bin/env python3
"""
pilot_control_mission - robot-side mission control CLI

Implements:
  pilot_control_mission start --token <TOKEN> --csv-path </abs/path/on/robot>
  pilot_control_mission set-speed --token <TOKEN> --speed <MPS>

Validates the session token issued by pilot_control_auth and, if valid, publishes a
std_msgs/String message to /start_waypoint_navigation locally on the robot or updates
the running MPC controller speed over the robot-local parameter service.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import hmac
import json
import math
import os
import sys
import time
from typing import Any


ROBOT_ID_PATH = "/etc/pilot_robot_id"
AUTH_SECRET_PATH = "/etc/pilot_auth_secret.key"


def _eprint(msg: str) -> None:
    sys.stderr.write(msg.rstrip() + "\n")


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _read_bytes(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def _b64url_decode_nopad(data: str) -> bytes:
    s = data.strip().encode("ascii")
    pad = b"=" * ((4 - (len(s) % 4)) % 4)
    return base64.urlsafe_b64decode(s + pad)


def _load_robot_id() -> str:
    rid = _read_text(ROBOT_ID_PATH).strip()
    if not rid:
        raise RuntimeError(f"{ROBOT_ID_PATH} is empty")
    return rid


def _load_secret() -> bytes:
    secret = _read_bytes(AUTH_SECRET_PATH).strip()
    if not secret:
        raise RuntimeError(f"{AUTH_SECRET_PATH} is empty")
    return secret


def _verify_token(secret: bytes, token: str) -> dict[str, Any]:
    parts = token.strip().split(".")
    if len(parts) != 2:
        raise RuntimeError("Invalid token format")

    payload_b64 = parts[0].encode("ascii")
    sig_b64 = parts[1]

    expected_sig = hmac.new(secret, payload_b64, hashlib.sha256).digest()
    got_sig = _b64url_decode_nopad(sig_b64)

    if not hmac.compare_digest(expected_sig, got_sig):
        raise RuntimeError("Invalid token signature")

    payload_json = _b64url_decode_nopad(parts[0])
    try:
        payload = json.loads(payload_json.decode("utf-8"))
    except Exception as e:
        raise RuntimeError(f"Invalid token payload JSON: {e}") from e

    if not isinstance(payload, dict):
        raise RuntimeError("Invalid token payload type")

    return payload


def _require_scope(payload: dict[str, Any], scope: str) -> None:
    scopes = payload.get("scopes", [])
    if not isinstance(scopes, list) or scope not in scopes:
        raise RuntimeError(f"Missing required scope: {scope}")


def _read_token_from_args(args: argparse.Namespace) -> str:
    token = (args.token or "").strip()
    if getattr(args, "token_stdin", False):
        try:
            token = sys.stdin.readline().strip()
        except Exception:
            token = ""
    if not token:
        raise RuntimeError("token is required")
    return token


def _authorize_request(args: argparse.Namespace, scope: str) -> tuple[str, int]:
    token = _read_token_from_args(args)

    robot_id_actual = _load_robot_id()
    secret = _load_secret()
    payload = _verify_token(secret, token)

    # Validate robot_id binding
    robot_id_token = str(payload.get("robot_id", "")).strip()
    if not robot_id_token or robot_id_token != robot_id_actual:
        raise RuntimeError("Token robot_id does not match this robot")

    # Validate expiry (exp <= 0 means no expiry)
    exp = 0
    exp_raw = payload.get("exp", None)
    if exp_raw is not None:
        try:
            exp = int(exp_raw)
        except Exception:
            raise RuntimeError("Token missing/invalid exp")

    if exp > 0:
        now = int(time.time())
        if exp <= now:
            raise RuntimeError("Token expired")

    # Validate scope
    _require_scope(payload, scope)
    return robot_id_actual, exp


def _format_expiry(exp: int) -> tuple[str, bool]:
    expires_at = ""
    expires_never = exp <= 0
    if exp > 0:
        expires_at = dt.datetime.fromtimestamp(exp, tz=dt.timezone.utc).isoformat().replace("+00:00", "Z")
    return expires_at, expires_never


def cmd_start(args: argparse.Namespace) -> int:
    try:
        robot_id_actual, exp = _authorize_request(args, "mission:start")
    except Exception as e:
        _eprint(str(e))
        return 1

    csv_path = args.csv_path.strip()
    if not csv_path:
        _eprint("csv_path is required")
        return 1

    if not os.path.isabs(csv_path):
        _eprint("csv_path must be an absolute path on the robot")
        return 1

    if not os.path.exists(csv_path):
        _eprint(f"CSV file not found: {csv_path}")
        return 1

    # Publish locally on the robot
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as e:
        _eprint(f"ROS2 Python environment not available (rclpy import failed): {e}")
        return 1

    try:
        rclpy.init(args=None)
        node = Node("pilot_control_mission_cli")
        pub = node.create_publisher(String, args.topic, 10)

        msg = String()
        msg.data = csv_path

        # Publish a few times to improve robustness for late-joining subscribers.
        for _ in range(3):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)

        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        _eprint(f"Failed to publish mission start: {e}")
        return 1

    expires_at, expires_never = _format_expiry(exp)

    out = {
        "ok": True,
        "robot_id": robot_id_actual,
        "topic": args.topic,
        "csv_path": csv_path,
        "expires_at": expires_at,
        "expires_never": expires_never,
    }
    sys.stdout.write(json.dumps(out, separators=(",", ":"), sort_keys=True) + "\n")
    return 0


def cmd_set_speed(args: argparse.Namespace) -> int:
    try:
        robot_id_actual, exp = _authorize_request(args, "mission:start")
    except Exception as e:
        _eprint(str(e))
        return 1

    try:
        speed_mps = float(args.speed)
    except Exception:
        _eprint("speed must be a finite float")
        return 1

    if not math.isfinite(speed_mps) or speed_mps < 0.0:
        _eprint("speed must be >= 0")
        return 1

    node_name = str(args.node or "").strip()
    if not node_name:
        _eprint("node is required")
        return 1

    parameter_name = str(args.parameter or "").strip()
    if not parameter_name:
        _eprint("parameter is required")
        return 1

    service_name = node_name.rstrip("/") + "/set_parameters"

    try:
        import rclpy as rclpy_module
        from rclpy.node import Node
        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
    except Exception as e:
        _eprint(f"ROS2 Python environment not available (rclpy import failed): {e}")
        return 1

    node = None
    try:
        rclpy_module.init(args=None)
        node = Node("pilot_control_mission_cli")
        client = node.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service unavailable: {service_name}")

        param = ParameterMsg()
        param.name = parameter_name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = speed_mps

        req = SetParameters.Request()
        req.parameters = [param]
        future = client.call_async(req)
        rclpy_module.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done():
            raise RuntimeError(f"Timed out waiting for {service_name}")

        response = future.result()
        if response is None:
            raise RuntimeError(f"{service_name} returned no response")

        for result in response.results:
            if not result.successful:
                raise RuntimeError(result.reason or f"{service_name} rejected parameter update")
    except Exception as e:
        _eprint(f"Failed to set MPC speed: {e}")
        return 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy_module.shutdown()
        except Exception:
            pass

    expires_at, expires_never = _format_expiry(exp)
    out = {
        "ok": True,
        "robot_id": robot_id_actual,
        "node": node_name,
        "parameter": parameter_name,
        "value": speed_mps,
        "service": service_name,
        "expires_at": expires_at,
        "expires_never": expires_never,
    }
    sys.stdout.write(json.dumps(out, separators=(",", ":"), sort_keys=True) + "\n")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pilot_control_mission")
    sub = p.add_subparsers(dest="cmd", required=True)

    start = sub.add_parser("start", help="Start waypoint navigation (robot-local publish)")
    token_group = start.add_mutually_exclusive_group(required=True)
    token_group.add_argument("--token", help="Session token from pilot_control_auth (not recommended; visible in process list)")
    token_group.add_argument("--token-stdin", dest="token_stdin", action="store_true", help="Read token from stdin")
    start.add_argument("--csv-path", "--csv_path", dest="csv_path", required=True, help="Absolute CSV path on robot")
    start.add_argument("--topic", default="/start_waypoint_navigation", help="ROS2 topic to publish (default: /start_waypoint_navigation)")
    start.add_argument("--json", action="store_true", help="Reserved (JSON is always used on success)")
    start.set_defaults(func=cmd_start)

    set_speed = sub.add_parser("set-speed", help="Update MPC speed using the robot-local parameter service")
    token_group = set_speed.add_mutually_exclusive_group(required=True)
    token_group.add_argument("--token", help="Session token from pilot_control_auth (not recommended; visible in process list)")
    token_group.add_argument("--token-stdin", dest="token_stdin", action="store_true", help="Read token from stdin")
    set_speed.add_argument("--speed", required=True, help="Desired linear speed in m/s")
    set_speed.add_argument("--node", default="/mpc_accel_autonomous_controller", help="Target node name (default: /mpc_accel_autonomous_controller)")
    set_speed.add_argument("--parameter", default="desired_linear_speed", help="Parameter to update (default: desired_linear_speed)")
    set_speed.add_argument("--json", action="store_true", help="Reserved (JSON is always used on success)")
    set_speed.set_defaults(func=cmd_set_speed)

    return p


def main() -> int:
    try:
        parser = build_parser()
        args = parser.parse_args()
        return int(args.func(args))
    except KeyboardInterrupt:
        _eprint("Interrupted")
        return 130
    except Exception as e:
        _eprint(str(e))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

