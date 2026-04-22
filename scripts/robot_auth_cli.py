#!/usr/bin/env python3
"""
pilot_control_auth - robot-side authentication CLI

Implements:
  pilot_control_auth login --robot-id <ROBOT_ID> (--pin <PIN> | --pin-stdin) [--ttl-sec N] [--json]

On success prints JSON to stdout:
  {"robot_id": "...", "token": "...", "expires_at": "2026-02-12T10:11:12Z", "scopes": ["mission:start"]}
  If --ttl-sec is 0 or negative, the session does not expire and expires_at is empty.

Security model:
  - robot_id must match /etc/pilot_robot_id
  - PIN is verified against a salted PBKDF2 hash at /etc/pilot_access_pin.hash
  - failed attempts are rate-limited using /var/lib/pilot_control/auth_state.json
  - token is HMAC-SHA256 signed using /etc/pilot_auth_secret.key
"""

from __future__ import annotations

import argparse
import base64
import dataclasses
import datetime as dt
import errno
import hashlib
import hmac
import json
import os
import sys
import time
from typing import Any


ROBOT_ID_PATH = "/etc/pilot_robot_id"
PIN_HASH_PATH = "/etc/pilot_access_pin.hash"
AUTH_SECRET_PATH = "/etc/pilot_auth_secret.key"
AUTH_STATE_PATH = "/var/lib/pilot_control/auth_state.json"


def _eprint(msg: str) -> None:
    sys.stderr.write(msg.rstrip() + "\n")


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _read_bytes(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def _b64url_nopad(data: bytes) -> bytes:
    return base64.urlsafe_b64encode(data).rstrip(b"=")


def _b64decode_any(b64: str) -> bytes:
    # Accept standard or urlsafe base64 (with/without padding)
    s = b64.strip().encode("ascii")
    pad = b"=" * ((4 - (len(s) % 4)) % 4)
    return base64.urlsafe_b64decode(s + pad)


@dataclasses.dataclass
class AuthState:
    failed_attempts: int = 0
    lock_until: float = 0.0  # unix time (seconds)

    @classmethod
    def load(cls, path: str) -> "AuthState":
        try:
            with open(path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            return cls(
                failed_attempts=int(obj.get("failed_attempts", 0) or 0),
                lock_until=float(obj.get("lock_until", 0.0) or 0.0),
            )
        except FileNotFoundError:
            return cls()
        except Exception:
            # Fail closed: don't allow login if state is unreadable/corrupt.
            raise RuntimeError(f"Auth state file is unreadable or corrupt: {path}")

    def save(self, path: str) -> None:
        d = os.path.dirname(path)
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as e:
            raise RuntimeError(f"Cannot create auth state directory {d}: {e}") from e

        tmp = f"{path}.tmp"
        payload = {"failed_attempts": self.failed_attempts, "lock_until": self.lock_until}
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(payload, f, separators=(",", ":"), sort_keys=True)
                f.write("\n")
            os.replace(tmp, path)
        except OSError as e:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise RuntimeError(f"Cannot write auth state file {path}: {e}") from e


def _rate_limit_delay_seconds(failed_attempts: int) -> int:
    # Exponential backoff with a reasonable cap.
    # failed_attempts=1 => 2s, 2=>4s, 3=>8s ... capped at 300s.
    exp = min(max(failed_attempts, 1), 8)
    return min(300, 2**exp)


def _load_robot_id() -> str:
    rid = _read_text(ROBOT_ID_PATH).strip()
    if not rid:
        raise RuntimeError(f"{ROBOT_ID_PATH} is empty")
    return rid


def _parse_pin_hash(line: str) -> tuple[int, bytes, bytes]:
    """
    Parse a hash line of the form:
      pbkdf2_sha256$<iterations>$<salt_b64>$<hash_b64>
    """
    parts = line.strip().split("$")
    if len(parts) != 4 or parts[0] != "pbkdf2_sha256":
        raise RuntimeError(
            f"Unsupported PIN hash format in {PIN_HASH_PATH}. "
            "Expected: pbkdf2_sha256$<iterations>$<salt_b64>$<hash_b64>"
        )
    iters = int(parts[1])
    salt = _b64decode_any(parts[2])
    digest = _b64decode_any(parts[3])
    if iters <= 0 or not salt or not digest:
        raise RuntimeError(f"Invalid PIN hash parameters in {PIN_HASH_PATH}")
    return iters, salt, digest


def _verify_pin(pin: str) -> bool:
    line = _read_text(PIN_HASH_PATH).strip()
    iters, salt, expected = _parse_pin_hash(line)
    derived = hashlib.pbkdf2_hmac("sha256", pin.encode("utf-8"), salt, iters)
    return hmac.compare_digest(derived, expected)


def _load_auth_secret() -> bytes:
    secret = _read_bytes(AUTH_SECRET_PATH).strip()
    if not secret:
        raise RuntimeError(f"{AUTH_SECRET_PATH} is empty")
    return secret


def _sign_token(secret: bytes, payload: dict[str, Any]) -> str:
    payload_json = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    payload_b64 = _b64url_nopad(payload_json)
    sig = hmac.new(secret, payload_b64, hashlib.sha256).digest()
    sig_b64 = _b64url_nopad(sig)
    return payload_b64.decode("ascii") + "." + sig_b64.decode("ascii")


def _iso_utc(ts: int) -> str:
    return dt.datetime.fromtimestamp(ts, tz=dt.timezone.utc).isoformat().replace("+00:00", "Z")


def cmd_login(args: argparse.Namespace) -> int:
    now = time.time()

    # Rate limit before doing any expensive operations
    state = AuthState.load(AUTH_STATE_PATH)
    if state.lock_until and state.lock_until > now:
        remaining = int(state.lock_until - now)
        _eprint(f"Too many failed attempts. Try again in {remaining}s.")
        return 1

    try:
        robot_id_actual = _load_robot_id()
    except Exception as e:
        _eprint(str(e))
        return 1

    robot_id_requested = (args.robot_id or "").strip()
    if not robot_id_requested:
        _eprint("robot_id is required")
        return 1

    # Read PIN (prefer stdin so it doesn't appear in process lists)
    pin = (args.pin or "").strip()
    if args.pin_stdin:
        try:
            pin = sys.stdin.readline().strip()
        except Exception:
            pin = ""

    if len(pin) != 6 or not pin.isdigit():
        _eprint("PIN must be exactly 6 digits")
        return 1

    # Fail fast on robot_id mismatch (counts toward rate limit)
    if robot_id_requested != robot_id_actual:
        state.failed_attempts += 1
        delay = _rate_limit_delay_seconds(state.failed_attempts)
        state.lock_until = time.time() + delay
        try:
            state.save(AUTH_STATE_PATH)
        except Exception as e:
            _eprint(str(e))
            return 1
        _eprint("robot_id does not match this robot")
        return 1

    # Verify PIN
    try:
        ok = _verify_pin(pin)
    except Exception as e:
        _eprint(str(e))
        return 1

    if not ok:
        state.failed_attempts += 1
        delay = _rate_limit_delay_seconds(state.failed_attempts)
        state.lock_until = time.time() + delay
        try:
            state.save(AUTH_STATE_PATH)
        except Exception as e:
            _eprint(str(e))
            return 1
        _eprint("Invalid PIN")
        return 1

    # Success: reset limiter
    state.failed_attempts = 0
    state.lock_until = 0.0
    try:
        state.save(AUTH_STATE_PATH)
    except Exception as e:
        _eprint(str(e))
        return 1

    try:
        secret = _load_auth_secret()
    except Exception as e:
        _eprint(str(e))
        return 1

    iat = int(time.time())
    ttl = int(args.ttl_sec)
    scopes = ["mission:start"]

    payload = {"robot_id": robot_id_actual, "iat": iat, "scopes": scopes}
    expires_at = ""
    expires_never = ttl <= 0
    if ttl > 0:
        exp = iat + ttl
        payload["exp"] = exp
        expires_at = _iso_utc(exp)
    else:
        payload["exp"] = 0
        payload["exp_disabled"] = True
    token = _sign_token(secret, payload)

    out = {
        "robot_id": robot_id_actual,
        "token": token,
        "expires_at": expires_at,
        "expires_never": expires_never,
        "scopes": scopes,
    }

    # Keep stdout machine-readable
    sys.stdout.write(json.dumps(out, separators=(",", ":"), sort_keys=True) + "\n")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="pilot_control_auth")
    sub = p.add_subparsers(dest="cmd", required=True)

    login = sub.add_parser("login", help="Login using robot_id + PIN; returns session token JSON")
    login.add_argument("--robot-id", required=True, help="Robot ID (must match /etc/pilot_robot_id)")
    pin_group = login.add_mutually_exclusive_group(required=True)
    pin_group.add_argument("--pin", help="6-digit PIN (not recommended; visible in process list)")
    pin_group.add_argument("--pin-stdin", action="store_true", help="Read PIN from stdin")
    login.add_argument("--ttl-sec", type=int, default=0,
                       help="Session TTL in seconds (0 disables expiry; default: 0)")
    login.add_argument("--json", action="store_true", help="Reserved (JSON is always used on success)")
    login.set_defaults(func=cmd_login)

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

