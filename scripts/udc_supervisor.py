#!/usr/bin/env python3
"""udc_supervisor.py — bounded restart supervisor for unified_data_collector.

Spawns ``unified_data_collector`` as a subprocess, monitors its exit code, and
restarts it on crash up to 3 times in 60 s.  After that it gives up and
publishes ``{"state": "DEAD_MAX_RESTARTS", ...}`` on ``/udc/health`` every 5 s
so the OCU's Start-Scan gate can refuse to arm a scan and ask the operator to
restart the launch.

Why a wrapper instead of ``respawn=True`` on the ROS Node?
    - ``respawn=True`` is unbounded: a UDC bug that loops on startup would
      restart forever and silently mask the real failure from the operator.
    - ROS 2 Humble's ``Node`` action does not expose a ``respawn_max_retries``
      parameter (added in Iron).  This wrapper backports that semantics.

False-positive avoidance (operator: "no false positives, very expensive"):
    - Exit code 0                 → clean exit, do NOT restart, do NOT count.
    - Exit signal SIGTERM (143)   → OCU teardown via pkill, do NOT count.
    - Exit signal SIGINT  (130)   → Ctrl-C / launch shutdown, do NOT count.
    - Anything else (segfault, abort, std::_Exit(2) from the GStreamer
      hard-fail path, OOM-kill, etc.) → counted as a crash.

Signal forwarding:
    SIGTERM / SIGINT received by the supervisor itself are forwarded to the
    child UDC process and the wrapper exits cleanly without restarting.  This
    preserves the OCU's existing teardown contract — pkill on the supervisor
    process tears down UDC the same way the OCU's pkill on UDC used to.

Topic contract: matches the schema published by UnifiedDataCollector itself
(see ``unified_data_collector.cpp``::``publishHealth``).  Topic is
``/udc/health`` (std_msgs/String JSON), QoS ``transient_local`` so the OCU sees
the latched DEAD_MAX_RESTARTS message even if it subscribes after the wrapper
publishes it.
"""

import collections
import json
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


CRASH_WINDOW_SECONDS = 60.0
MAX_CRASHES_IN_WINDOW = 3
RESTART_DELAY_SECONDS = 2.0
DEAD_REPUBLISH_PERIOD_SECONDS = 5.0
# Exit codes that mean "clean shutdown" — never counted as a crash.
# 128 + signal_number is the convention for shell-style exit codes when a
# process is terminated by a signal.
CLEAN_EXIT_CODES = {0, 128 + signal.SIGTERM, 128 + signal.SIGINT}


def _is_clean_exit(returncode: int) -> bool:
    """Return True if ``returncode`` represents an operator-initiated shutdown.

    ``subprocess.Popen.returncode`` is negative when the child was terminated
    by a signal (``-N``) and positive (or zero) otherwise.  We accept both
    encodings so we never miscount a teardown as a crash.
    """
    if returncode in CLEAN_EXIT_CODES:
        return True
    if returncode < 0 and -returncode in (signal.SIGTERM, signal.SIGINT):
        return True
    return False


class UdcSupervisor(Node):
    def __init__(self, child_argv):
        super().__init__('udc_supervisor')
        self._child_argv = child_argv
        # Latched publisher so a late-subscribing OCU still sees DEAD.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._health_pub = self.create_publisher(String, '/udc/health', latched_qos)
        self._crash_times = collections.deque()
        self._child = None
        self._stop_requested = False
        self._dead_state_active = False
        # Republish DEAD_MAX_RESTARTS every 5 s once we give up.  Idle while
        # UDC is alive (UDC publishes its own RECORDING/STARTING/etc. heartbeat
        # at 1 Hz from inside the process).
        self._dead_republish_timer = self.create_timer(
            DEAD_REPUBLISH_PERIOD_SECONDS, self._republish_dead_if_active
        )

    # ---- public lifecycle --------------------------------------------------

    def request_stop(self, signum):
        """Forward SIGTERM/SIGINT to the child and stop the supervise loop."""
        self._stop_requested = True
        if self._child is not None and self._child.poll() is None:
            try:
                self._child.send_signal(signum)
            except ProcessLookupError:
                pass

    def supervise(self) -> int:
        """Run the spawn/monitor loop. Returns wrapper exit code."""
        while not self._stop_requested:
            self.get_logger().info(
                f'Spawning unified_data_collector: {shlex.join(self._child_argv)}'
            )
            try:
                self._child = subprocess.Popen(self._child_argv)
            except FileNotFoundError as exc:
                self.get_logger().fatal(f'unified_data_collector binary not found: {exc}')
                self._publish_dead(reason=f'binary not found: {exc}')
                self._dead_state_active = True
                return 127

            returncode = self._child.wait()
            self._child = None

            if self._stop_requested:
                self.get_logger().info(
                    f'Supervisor stop requested (child exit={returncode}); '
                    'NOT restarting.'
                )
                return 0

            if _is_clean_exit(returncode):
                self.get_logger().info(
                    f'unified_data_collector exited cleanly (rc={returncode}); '
                    'NOT restarting.'
                )
                return 0

            now = time.monotonic()
            self._crash_times.append(now)
            while self._crash_times and (now - self._crash_times[0]) > CRASH_WINDOW_SECONDS:
                self._crash_times.popleft()
            crash_count = len(self._crash_times)

            self.get_logger().error(
                f'unified_data_collector CRASHED (rc={returncode}). '
                f'Crash {crash_count}/{MAX_CRASHES_IN_WINDOW} in last '
                f'{CRASH_WINDOW_SECONDS:.0f}s.'
            )

            if crash_count >= MAX_CRASHES_IN_WINDOW:
                reason = (
                    f'unified_data_collector crashed {crash_count} times in '
                    f'{CRASH_WINDOW_SECONDS:.0f}s (last rc={returncode}). '
                    'Operator must restart the OCU/launch.'
                )
                self.get_logger().fatal(reason)
                self._publish_dead(reason=reason)
                self._dead_state_active = True
                # Stay alive so the latched DEAD_MAX_RESTARTS message keeps
                # being republished every DEAD_REPUBLISH_PERIOD_SECONDS to
                # any new OCU subscriber.  rclpy.spin handles the timer.
                return self._wait_until_stopped()

            self.get_logger().warn(
                f'Restarting unified_data_collector in {RESTART_DELAY_SECONDS:.1f}s...'
            )
            self._sleep_interruptible(RESTART_DELAY_SECONDS)
        return 0

    # ---- internals ---------------------------------------------------------

    def _publish_dead(self, reason: str):
        payload = json.dumps({
            'state': 'DEAD_MAX_RESTARTS',
            'thermal_connect_seen': False,
            'recording_active': False,
            'paused': False,
            'total_rows_enqueued': 0,
            'last_row_enqueued_ns': 0,
            'dropped_thermal_empty_total': 0,
            'dropped_left_empty_total': 0,
            'dropped_right_empty_total': 0,
            'uptime_ms': 0,
            'supervisor_reason': reason,
        })
        msg = String()
        msg.data = payload
        self._health_pub.publish(msg)

    def _republish_dead_if_active(self):
        if not self._dead_state_active:
            return
        # Re-publish the latched DEAD message so any newly-subscribed OCU
        # gets it via TRANSIENT_LOCAL, and so any reader that was briefly
        # disconnected re-syncs on reconnect.
        self._publish_dead(reason='republish (operator: restart OCU/launch)')

    def _sleep_interruptible(self, seconds: float):
        deadline = time.monotonic() + seconds
        while not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.1, remaining))

    def _wait_until_stopped(self) -> int:
        # Don't return until SIGTERM/SIGINT — keeps the timer alive so the
        # latched DEAD_MAX_RESTARTS message is republished periodically.
        while not self._stop_requested:
            time.sleep(0.2)
        return 0


def _rewrite_params_for_child(params_file: str, supervisor_name: str = 'udc_supervisor',
                              child_name: str = 'unified_data_collector') -> str:
    """Rewrite a ROS 2 params YAML so its top-level key matches the child node name.

    ROS 2 launch generates a params file keyed by the launched action's node
    name (``udc_supervisor``).  Our child UDC binary expects the YAML to be
    keyed by its own node name (``unified_data_collector``).  Rather than
    duplicate the param block in the launch file, we rewrite the supervisor's
    own params YAML in-place to be a valid params file for the child.
    """
    try:
        with open(params_file, 'r', encoding='utf-8') as f:
            tree = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return params_file
    if not isinstance(tree, dict):
        return params_file
    if supervisor_name in tree and child_name not in tree:
        tree[child_name] = tree.pop(supervisor_name)
    elif supervisor_name in tree and child_name in tree:
        # Merge supervisor block into child block, supervisor-side wins.
        sup_block = tree.pop(supervisor_name)
        if isinstance(sup_block, dict) and isinstance(tree[child_name], dict):
            tree[child_name].update(sup_block)
        else:
            tree[child_name] = sup_block
    fd, out_path = tempfile.mkstemp(prefix='udc_params_', suffix='.yaml')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        yaml.safe_dump(tree, f)
    return out_path


def _build_child_argv_from_supervisor_argv(argv):
    """Construct the child UDC command-line by reusing the supervisor's --ros-args.

    When the supervisor is launched via ``Node(executable='udc_supervisor.py')``,
    ``ros2 launch`` invokes us with::

        udc_supervisor.py --ros-args -r __node:=udc_supervisor \\
                                     --params-file /tmp/launch_params_XYZ

    We:
        1. Locate the ``unified_data_collector`` binary in the same install tree.
        2. Rewrite the params YAML so its top-level key is ``unified_data_collector``.
        3. Build a child command line that points at the rewritten YAML and
           remaps ``__node:=unified_data_collector``.
    """
    udc_binary = os.environ.get('UDC_BINARY')
    if not udc_binary:
        # Same install layout as the supervisor itself.
        here = os.path.dirname(os.path.realpath(__file__))
        candidate = os.path.join(here, 'unified_data_collector')
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            udc_binary = candidate
    if not udc_binary:
        print('udc_supervisor: cannot locate unified_data_collector binary; '
              'set $UDC_BINARY or place it next to udc_supervisor.py',
              file=sys.stderr)
        return None

    child_argv = [udc_binary, '--ros-args', '-r', '__node:=unified_data_collector']

    # Walk argv looking for --params-file <path> and forward (rewritten).
    i = 1
    while i < len(argv):
        tok = argv[i]
        if tok == '--params-file' and (i + 1) < len(argv):
            rewritten = _rewrite_params_for_child(argv[i + 1])
            child_argv += ['--params-file', rewritten]
            i += 2
            continue
        if tok in ('--ros-args', '-r') or tok == '__node:=udc_supervisor':
            # Skip — we already added our own --ros-args/-r above.
            i += 1
            continue
        if tok.startswith('__node:='):
            i += 1
            continue
        i += 1
    return child_argv


def main():
    rclpy.init(args=sys.argv)

    # Mode A (preferred, used by ros2 launch):
    #   udc_supervisor.py --ros-args -r __node:=udc_supervisor --params-file FOO
    # Mode B (manual / testing):
    #   udc_supervisor.py -- /path/to/unified_data_collector --ros-args ...
    if '--' in sys.argv:
        idx = sys.argv.index('--')
        child_argv = sys.argv[idx + 1:]
    else:
        child_argv = _build_child_argv_from_supervisor_argv(sys.argv) or []

    if not child_argv:
        print('udc_supervisor: missing child argv (expected `-- unified_data_collector ...`)',
              file=sys.stderr)
        sys.exit(2)

    supervisor = UdcSupervisor(child_argv)

    # rclpy.spin in a background thread so timers fire while .supervise() is
    # blocked on subprocess.wait.  spin_until_future_complete won't work here
    # because we never have a future to wait on.
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(supervisor,), daemon=True
    )
    spin_thread.start()

    def _on_signal(signum, _frame):
        supervisor.get_logger().info(
            f'Supervisor received signal {signum}; forwarding to child and exiting.'
        )
        supervisor.request_stop(signum)

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    rc = supervisor.supervise()

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(rc)


if __name__ == '__main__':
    main()
