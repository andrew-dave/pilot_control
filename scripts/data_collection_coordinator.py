#!/usr/bin/env python3
"""
Data Collection Coordinator Node
================================
Central coordinator for managing data collection across multiple nodes.

Provides unified services for:
- start_dc: Start all data collection (creates new section, then GNSS -> B -> G -> R)
- start_gnss_precapture: Begin temporary raw GNSS capture before /dc/start
- pause_dc: Pause all data collection (visual, GPR, rosbag)
- resume_dc: Resume all data collection
- end_and_save_dc: End collection and save with partial/complete tag
- end_and_delete_dc: End collection and delete all session data

This node coordinates:
- gps_driver (raw UBX logging for PPK / post-processing)
- unified_data_collector (thermal + cameras + odometry)
- gpr_scan_controller (GPR + rosbag)
- raw_map_saver (point cloud maps)

Section folders are created dynamically by this coordinator at /dc/start time,
using the actual collection start time (not the pipeline launch time).

Note: Linear actuator control is handled internally by gpr_scan_controller
when /gpr_scan/start or /gpr_scan/stop is called. The coordinator does NOT
call /gpr_line_start or /gpr_line_stop directly to avoid duplicate actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import os
import re
import shutil
import json
import time
from pathlib import Path
from datetime import datetime


class DataCollectionCoordinator(Node):
    def __init__(self):
        super().__init__('data_collection_coordinator')
        
        # Use reentrant callback group to allow service calls from within service callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('base_data_directory', '/R_DATA')
        self.declare_parameter('scan_mode', 'outdoor')
        self.declare_parameter('start_sequence_delay', 1.0)  # Delay between start steps
        self.declare_parameter('gnss_temp_directory', '/tmp')
        self.declare_parameter('gnss_precapture_timeout_sec', 1800.0)
        self.declare_parameter('gnss_post_stop_delay_sec', 30.0)
        self.declare_parameter('gnss_validation_min_file_size_bytes', 524288)
        self.declare_parameter('gnss_validation_min_duration_sec', 60.0)
        # ----------------------------------------------------------------
        # Per-mission session metadata. Pushed by the OCU's New Scan
        # Information modal via SetParameters before the OCU calls
        # /dc/start. Read fresh inside ensure_mission_session() and
        # create_new_section() so a mid-mission re-prompt (operator backs
        # out to Dashboard, picks a different building, comes back) takes
        # effect on the next /dc/start without a coordinator restart.
        #
        # Empty strings are valid: they trigger the fallback path
        # ('BDR_test' / 'unknown_operator' / 'metric'). That keeps the
        # coordinator usable from `ros2 service call` without the OCU.
        # ----------------------------------------------------------------
        self.declare_parameter('building_name', '')
        self.declare_parameter('operator_name', '')
        self.declare_parameter('units_preference', '')
        
        # Get parameters
        self.base_data_dir = self.get_parameter('base_data_directory').value
        self.scan_mode = str(self.get_parameter('scan_mode').value).strip().lower()
        if self.scan_mode not in ('indoor', 'outdoor'):
            raise ValueError(
                f"scan_mode must be 'indoor' or 'outdoor', got '{self.scan_mode}'")
        self.gnss_enabled = self.scan_mode == 'outdoor'
        self.start_delay = self.get_parameter('start_sequence_delay').value
        self.gnss_temp_directory = self.get_parameter('gnss_temp_directory').value
        self.gnss_precapture_timeout_sec = float(
            self.get_parameter('gnss_precapture_timeout_sec').value)
        self.gnss_post_stop_delay_sec = float(
            self.get_parameter('gnss_post_stop_delay_sec').value)
        self.gnss_validation_min_file_size_bytes = int(
            self.get_parameter('gnss_validation_min_file_size_bytes').value)
        self.gnss_validation_min_duration_sec = float(
            self.get_parameter('gnss_validation_min_duration_sec').value)
        
        # Dynamic state — set on each /dc/start
        self.day_folder = ''
        self.building_folder = ''   # /R_DATA/<day>/<building_slug>
        self.section_folder = ''
        self.section_timestamp = ''

        # Mission-scoped GNSS state. Raw UBX is logged once across an entire
        # operator mission (multiple /dc/start ↔ /dc/end_and_save cycles) into
        # a single .ubx file, instead of per-section. PPK uses the mission
        # file plus per-section start/end timestamps to slice playback.
        self.mission_folder = ''
        self.mission_timestamp = ''
        self.mission_gnss_folder = ''
        self.mission_gnss_path = ''
        self.mission_gnss_started_at = None
        self.mission_gnss_active = False
        self.mission_section_history = []
        self.mission_active_section_start = None

        # State tracking
        self.is_paused = False
        self.collection_active = True   # Assume active — stop services should always work
        self.collection_started = False
        self.gnss_data_folder = ''
        self.gnss_session_file = ''
        self.gnss_raw_file = ''
        self.gnss_precapture_active = False
        self.gnss_precapture_path = ''
        self.gnss_precapture_started_at = None
        self.gnss_precapture_started_monotonic = None
        self.preserved_gnss_precapture_paths = []
        self.latest_gnss_health = {}
        self.latest_gps_driver_boot_time = None
        self.session_gps_driver_boot_time = None
        self.gnss_driver_restarts = 0
        self.gnss_invalid = False
        self.gnss_invalid_reasons = []
        self.gnss_warning_reasons = []
        self.reset_component_state()
        
        # ==================== Service Servers ====================
        self.start_srv = self.create_service(
            Trigger, '/dc/start', self.start_callback,
            callback_group=self.callback_group)
        self.start_gnss_precapture_srv = self.create_service(
            Trigger, '/dc/start_gnss_precapture', self.start_gnss_precapture_callback,
            callback_group=self.callback_group)
        self.pause_srv = self.create_service(
            Trigger, '/dc/pause', self.pause_callback,
            callback_group=self.callback_group)
        self.resume_srv = self.create_service(
            Trigger, '/dc/resume', self.resume_callback,
            callback_group=self.callback_group)
        self.end_save_srv = self.create_service(
            SetBool, '/dc/end_and_save', self.end_and_save_callback,
            callback_group=self.callback_group)
        self.end_delete_srv = self.create_service(
            Trigger, '/dc/end_and_delete', self.end_and_delete_callback,
            callback_group=self.callback_group)
        self.finalize_mission_srv = self.create_service(
            Trigger, '/dc/finalize_mission', self.finalize_mission_callback,
            callback_group=self.callback_group)
        self.cancel_scan_srv = self.create_service(
            Trigger, '/dc/cancel_scan', self.cancel_scan_callback,
            callback_group=self.callback_group)
        
        # ==================== Service Clients ====================
        # unified_data_collector services
        self.video_record_client = self.create_client(
            SetBool, '/video_record_set', callback_group=self.callback_group)
        self.udc_pause_client = self.create_client(
            Trigger, '/udc/pause', callback_group=self.callback_group)
        self.udc_resume_client = self.create_client(
            Trigger, '/udc/resume', callback_group=self.callback_group)
        
        # gpr_scan_controller services
        self.gpr_scan_start_client = self.create_client(
            Trigger, '/gpr_scan/start', callback_group=self.callback_group)
        self.gpr_pause_client = self.create_client(
            Trigger, '/gpr_scan/pause', callback_group=self.callback_group)
        self.gpr_resume_client = self.create_client(
            Trigger, '/gpr_scan/resume', callback_group=self.callback_group)
        self.gpr_stop_client = self.create_client(
            Trigger, '/gpr_scan/stop', callback_group=self.callback_group)
        
        # rosbag control (via gpr_scan_controller)
        self.rosbag_start_client = self.create_client(
            Trigger, '/rosbag/start', callback_group=self.callback_group)
        self.rosbag_pause_client = self.create_client(
            Trigger, '/rosbag/pause', callback_group=self.callback_group)
        self.rosbag_resume_client = self.create_client(
            Trigger, '/rosbag/resume', callback_group=self.callback_group)
        self.rosbag_stop_client = self.create_client(
            Trigger, '/rosbag/stop', callback_group=self.callback_group)
        
        # gps raw logging control
        self.gps_raw_log_set_client = self.create_client(
            SetBool, '/gps/raw_log_set', callback_group=self.callback_group)
        self.gps_raw_log_promote_client = self.create_client(
            Trigger, '/gps/raw_log_promote', callback_group=self.callback_group)
        self.gps_raw_health_snapshot_client = self.create_client(
            Trigger, '/gps/raw_health_snapshot', callback_group=self.callback_group)

        # raw_map_saver service
        self.save_map_client = self.create_client(
            Trigger, '/save_raw_map', callback_group=self.callback_group)
        
        # Directory update services (for multi-section support)
        self.gpr_set_dir_client = self.create_client(
            Trigger, '/gpr_scan/set_directory', callback_group=self.callback_group)
        self.udc_set_dir_client = self.create_client(
            Trigger, '/udc/set_directory', callback_group=self.callback_group)
        self.map_set_dir_client = self.create_client(
            Trigger, '/raw_map_saver/set_directory', callback_group=self.callback_group)
        
        # Parameter setting clients
        self.gpr_set_params_client = self.create_client(
            SetParameters, '/gpr_scan_controller/set_parameters', callback_group=self.callback_group)
        self.udc_set_params_client = self.create_client(
            SetParameters, '/unified_data_collector/set_parameters', callback_group=self.callback_group)
        self.map_set_params_client = self.create_client(
            SetParameters, '/raw_map_saver/set_parameters', callback_group=self.callback_group)
        self.gps_set_params_client = self.create_client(
            SetParameters, '/gps_driver/set_parameters', callback_group=self.callback_group)

        self.gnss_health_sub = self.create_subscription(
            String, '/gps/raw_health', self.gnss_health_callback, 10,
            callback_group=self.callback_group)

        # Cross-process state-reset broadcast. Other DC-aware nodes (today
        # just mpc_accel_autonomous_controller, but the channel is generic)
        # subscribe and drop any stale local DC flags. See
        # docs/instructions.md EC-001 for the bug this fixed (cancel-then-
        # replan left dc_active=True in the controller, blocking the next
        # /dc/start). Payload is a String so future cancel-like events
        # (e.g. 'mission_finalized') can reuse the topic.
        self.dc_state_reset_pub = self.create_publisher(
            String, '/dc/state_reset', 10)

        self.precapture_watchdog_timer = self.create_timer(
            30.0, self.precapture_watchdog_callback, callback_group=self.callback_group)
        if self.gnss_enabled:
            self.cleanup_stale_gnss_precaptures()
        
        self.get_logger().info('='*60)
        self.get_logger().info('DATA COLLECTION COORDINATOR STARTED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Base data directory: {self.base_data_dir}')
        self.get_logger().info(f'Scan mode: {self.scan_mode}')
        if self.gnss_enabled:
            self.get_logger().info('GNSS collection: enabled for outdoor mode')
        else:
            self.get_logger().info('GNSS collection: disabled for indoor mode')
        self.get_logger().info('')
        self.get_logger().info('Available services:')
        self.get_logger().info('  /dc/start          - Start all data collection (B->G->R)')
        self.get_logger().info('  /dc/start_gnss_precapture - Start temporary raw GNSS logging')
        self.get_logger().info('  /dc/pause          - Pause all data collection')
        self.get_logger().info('  /dc/resume         - Resume data collection')
        self.get_logger().info('  /dc/end_and_save   - End and save (data: 0=partial, 1=complete)')
        self.get_logger().info('  /dc/end_and_delete - End and delete all session data')
        self.get_logger().info('  /dc/finalize_mission - Stop continuous GNSS + finalize mission folder')
        self.get_logger().info('  /dc/cancel_scan      - Abort whole scan: stop DC + delete all section data + mission folder')
        self.get_logger().info('')
        self.get_logger().info('Section folders are created dynamically on /dc/start')
        self.get_logger().info(
            '  Layout: <base>/<day>/<building_slug>/Section_<n>_<HHMMSS>/')
        self.get_logger().info(
            '  Set ROS params building_name / operator_name / units_preference')
        self.get_logger().info(
            '  before /dc/start to override fallbacks (BDR_test / unknown_operator / metric)')
        self.get_logger().info('='*60)

    # ================================================================
    #  Helpers
    # ================================================================

    # ---- Session metadata (building / operator / units) -------------
    #
    # Pulled from ROS parameters every time a section or mission folder is
    # created so a re-prompt from the OCU mid-session is reflected on the
    # next /dc/start without a node restart. See `__init__` for context on
    # the empty-string fallbacks.

    _SLUG_DISALLOWED = re.compile(r'[^A-Za-z0-9._-]+')

    @staticmethod
    def slugify_building_name(raw: str) -> str:
        """Convert an arbitrary string into a path-safe slug.

        Mirrors `MissionMetadataDialog::slugify` in the OCU (C++).
        Replaces runs of disallowed characters with `_`, strips leading/
        trailing `_` and `.`, truncates to 64 chars. Empty input returns
        '' so the caller can fall back to 'BDR_test'.
        """
        if not raw:
            return ''
        trimmed = raw.strip()
        if not trimmed:
            return ''
        slug = DataCollectionCoordinator._SLUG_DISALLOWED.sub('_', trimmed)
        slug = slug.strip('_.')
        if len(slug) > 64:
            slug = slug[:64].rstrip('_.')
        return slug

    def current_building_name(self) -> str:
        """Operator-typed raw building name (post-strip). Fallback string
        when the OCU never pushed a value."""
        raw = str(self.get_parameter('building_name').value or '').strip()
        return raw if raw else 'BDR_test'

    def current_building_slug(self) -> str:
        """Path-safe building slug used as the directory name under <day>/.

        Returns 'BDR_test' when the param is empty or slugifies to nothing,
        which keeps the on-disk tree well-formed for `ros2 service call`-
        only workflows (no OCU in the loop).
        """
        slug = self.slugify_building_name(
            str(self.get_parameter('building_name').value or ''))
        return slug if slug else 'BDR_test'

    def current_operator_name(self) -> str:
        raw = str(self.get_parameter('operator_name').value or '').strip()
        return raw if raw else 'unknown_operator'

    def current_units_preference(self) -> str:
        raw = str(self.get_parameter('units_preference').value or '').strip().lower()
        return raw if raw in ('metric', 'ansi') else 'metric'

    def current_session_metadata(self) -> dict:
        """Bundle for embedding into session_config.json / gnss_session.json
        / mission_config.json. Always read fresh — never cached."""
        return {
            'building_name': self.current_building_name(),
            'building_slug': self.current_building_slug(),
            'operator_name': self.current_operator_name(),
            'units_preference': self.current_units_preference(),
        }

    def call_service_sync(self, client, request, service_name, timeout=5.0):
        """
        Call a service synchronously with proper timeout handling.
        Uses polling to avoid blocking the executor.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {service_name} not available')
            return False, f'Service {service_name} not available'
        
        future = client.call_async(request)
        
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'Service {service_name} timed out')
                return False, f'Service {service_name} timed out'
            time.sleep(0.05)
        
        try:
            result = future.result()
            if result is not None:
                return result.success, result.message
            else:
                return False, f'Service {service_name} returned None'
        except Exception as e:
            self.get_logger().error(f'Service {service_name} exception: {e}')
            return False, f'Service {service_name} exception: {e}'

    def reset_component_state(self):
        """Reset best-effort runtime state for coordinated collectors."""
        self.video_recording_active = False
        self.video_paused = False
        self.gpr_scan_active = False
        self.gpr_paused = False
        self.rosbag_recording_active = False
        self.rosbag_paused = False
        self.gnss_logging_active = False

    def _msg_contains(self, msg: str, *substrings: str) -> bool:
        lower = (msg or '').lower()
        return any(substr in lower for substr in substrings)

    def set_node_param(self, param_client, param_name, param_value, node_label):
        """Set a string parameter on a remote node. Returns (success, message)."""
        if not param_client.wait_for_service(timeout_sec=2.0):
            msg = f'{node_label}: parameter service not available'
            self.get_logger().warn(f'  {msg}')
            return False, msg
        
        param = Parameter()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = param_value
        
        req = SetParameters.Request()
        req.parameters = [param]
        
        future = param_client.call_async(req)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 3.0:
            time.sleep(0.05)
        
        if not future.done():
            msg = f'{node_label}: parameter update timed out'
            self.get_logger().warn(f'  {msg}')
            return False, msg

        try:
            result = future.result()
        except Exception as e:
            msg = f'{node_label}: parameter update exception: {e}'
            self.get_logger().warn(f'  {msg}')
            return False, msg

        if result is None or not hasattr(result, 'results') or not result.results:
            msg = f'{node_label}: parameter update returned no results'
            self.get_logger().warn(f'  {msg}')
            return False, msg

        for res in result.results:
            if not res.successful:
                reason = res.reason if res.reason else 'parameter rejected'
                msg = f'{node_label}: {param_name} rejected ({reason})'
                self.get_logger().warn(f'  {msg}')
                return False, msg

        return True, f'{node_label}: {param_name} updated'

    def update_json_file(self, path: Path, updates: dict):
        """Merge updates into a JSON file, creating it if needed."""
        try:
            data = {}
            if path.exists():
                with open(path, 'r') as f:
                    data = json.load(f)
            data.update(updates)
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to update JSON file {path}: {e}')

    def cleanup_stale_gnss_precaptures(self):
        """Delete stale temporary GNSS precapture files on startup."""
        temp_dir = Path(self.gnss_temp_directory)
        if not temp_dir.exists():
            return
        removed = 0
        for path in temp_dir.glob('gnss_precapture_*.ubx'):
            try:
                path.unlink()
                removed += 1
            except Exception as e:
                self.get_logger().warn(f'Failed to delete stale GNSS precapture {path}: {e}')
        if removed > 0:
            self.get_logger().info(f'Removed {removed} stale GNSS precapture file(s) from {temp_dir}')

    def append_unique_reason(self, reasons, reason: str):
        if reason and reason not in reasons:
            reasons.append(reason)

    def extract_gnss_health_fields(self, health: dict) -> dict:
        if not health:
            return {}
        return {
            'driver_boot_time': health.get('driver_boot_time'),
            'health_level': health.get('health_level'),
            'gnss_invalid': health.get('gnss_invalid'),
            'raw_log_active': health.get('raw_log_active'),
            'raw_file_path': health.get('raw_log_path'),
            'raw_log_bytes': health.get('raw_log_bytes'),
            'logging_failures': health.get('logging_failures'),
            'serial_reconnects': health.get('serial_reconnects'),
            'flush_count': health.get('flush_count'),
            'fsync_count': health.get('fsync_count'),
            'rawx_seen': health.get('rawx_seen'),
            'sfrbx_seen': health.get('sfrbx_seen'),
            'rawx_count': health.get('rawx_count'),
            'sfrbx_count': health.get('sfrbx_count'),
            'nav_sat_count': health.get('nav_sat_count'),
            'hpposllh_count': health.get('hpposllh_count'),
            'first_raw_obs_time': health.get('first_raw_obs_time'),
            'last_raw_obs_time': health.get('last_raw_obs_time'),
            'first_sfrbx_time': health.get('first_sfrbx_time'),
            'last_sfrbx_time': health.get('last_sfrbx_time'),
            'raw_log_elapsed_sec': health.get('raw_log_elapsed_sec'),
            'rawx_expected_epochs': health.get('rawx_expected_epochs'),
            'rawx_coverage_ratio': health.get('rawx_coverage_ratio'),
            'current_rawx_gap_sec': health.get('current_rawx_gap_sec'),
            'max_rawx_gap_sec': health.get('max_rawx_gap_sec'),
            'rawx_startup_failed': health.get('rawx_startup_failed'),
            'sfrbx_startup_warned': health.get('sfrbx_startup_warned'),
            'rawx_gap_warned': health.get('rawx_gap_warned'),
            'rawx_gap_invalid': health.get('rawx_gap_invalid'),
            'warning_reasons': health.get('warning_reasons', []),
            'invalid_reasons': health.get('invalid_reasons', []),
        }

    def sync_gnss_status_to_metadata(self):
        if not self.gnss_enabled:
            return
        updates = {
            'gnss_invalid': self.gnss_invalid,
            'gnss_invalid_reasons': self.gnss_invalid_reasons,
            'gnss_warning_reasons': self.gnss_warning_reasons,
            'driver_restarts': self.gnss_driver_restarts,
            'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
        }
        self.update_session_config(updates)
        self.update_gnss_session(updates)
        if self.latest_gnss_health:
            self.update_gnss_session(self.extract_gnss_health_fields(self.latest_gnss_health))

    def mark_gnss_warning(self, reason: str):
        self.append_unique_reason(self.gnss_warning_reasons, reason)
        self.sync_gnss_status_to_metadata()

    def mark_gnss_invalid(self, reason: str):
        was_invalid = self.gnss_invalid
        self.gnss_invalid = True
        self.append_unique_reason(self.gnss_invalid_reasons, reason)
        self.sync_gnss_status_to_metadata()
        if not was_invalid:
            self.get_logger().error(f'GNSS session marked invalid for PPK: {reason}')
        else:
            self.get_logger().error(f'Additional GNSS invalid reason: {reason}')

    def register_preserved_precapture_path(self, path_str: str):
        if path_str and path_str not in self.preserved_gnss_precapture_paths:
            self.preserved_gnss_precapture_paths.append(path_str)
            self.sync_gnss_status_to_metadata()

    def delete_preserved_precapture_paths(self):
        for path_str in list(self.preserved_gnss_precapture_paths):
            self.delete_file_if_exists(path_str)
        self.preserved_gnss_precapture_paths = []

    def gnss_health_callback(self, msg: String):
        """Track live GNSS health for metadata, restart detection, and operator warnings."""
        if not self.gnss_enabled:
            return
        try:
            health = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /gps/raw_health JSON: {e}')
            return

        boot_time = health.get('driver_boot_time')
        if boot_time:
            if self.latest_gps_driver_boot_time and boot_time != self.latest_gps_driver_boot_time:
                if self.gnss_precapture_active or self.collection_started:
                    self.gnss_driver_restarts += 1
                    self.mark_gnss_invalid('gps_driver restarted during GNSS logging session')
            self.latest_gps_driver_boot_time = boot_time

        self.latest_gnss_health = health

        for reason in health.get('warning_reasons', []):
            self.append_unique_reason(self.gnss_warning_reasons, reason)
        if health.get('gnss_invalid'):
            for reason in health.get('invalid_reasons', []):
                self.append_unique_reason(self.gnss_invalid_reasons, reason)
            self.gnss_invalid = True

        self.sync_gnss_status_to_metadata()

    def fetch_gnss_health_snapshot(self):
        """Fetch the latest GNSS raw-health snapshot from gps_driver."""
        if not self.gnss_enabled:
            return {}
        success, msg = self.call_service_sync(
            self.gps_raw_health_snapshot_client, Trigger.Request(), '/gps/raw_health_snapshot', timeout=5.0)
        if success:
            try:
                health = json.loads(msg)
                self.latest_gnss_health = health
                boot_time = health.get('driver_boot_time')
                if boot_time:
                    self.latest_gps_driver_boot_time = boot_time
                for reason in health.get('warning_reasons', []):
                    self.append_unique_reason(self.gnss_warning_reasons, reason)
                if health.get('gnss_invalid'):
                    self.gnss_invalid = True
                    for reason in health.get('invalid_reasons', []):
                        self.append_unique_reason(self.gnss_invalid_reasons, reason)
                self.sync_gnss_status_to_metadata()
                return health
            except Exception as e:
                self.get_logger().warn(f'Failed to decode GNSS health snapshot: {e}')
        elif msg:
            self.get_logger().warn(f'GNSS health snapshot unavailable: {msg}')
        return dict(self.latest_gnss_health) if self.latest_gnss_health else {}

    def cleanup_failed_start_section(self):
        """Remove a newly-created section folder after a hard /dc/start failure."""
        if self.section_folder and os.path.exists(self.section_folder):
            try:
                shutil.rmtree(self.section_folder)
                self.get_logger().info(f'Removed failed start section folder: {self.section_folder}')
            except Exception as e:
                self.get_logger().warn(f'Failed to remove failed start section folder: {e}')
        self.day_folder = ''
        self.building_folder = ''
        self.section_folder = ''
        self.section_timestamp = ''
        self.gnss_data_folder = ''
        self.gnss_session_file = ''
        self.gnss_raw_file = ''
        self.collection_started = False
        self.is_paused = False
        self.reset_component_state()

    def validate_gnss_session(self, health: dict):
        """Validate the GNSS session against the agreed PPK readiness rules."""
        if not self.gnss_enabled:
            return [], []
        warnings = []
        invalid_reasons = []

        if not health:
            invalid_reasons.append('GNSS health snapshot unavailable at end of session')
            return warnings, invalid_reasons

        if self.gnss_raw_file:
            if not os.path.exists(self.gnss_raw_file):
                invalid_reasons.append('GNSS raw log file missing at end of session')
            else:
                file_size = os.path.getsize(self.gnss_raw_file)
                if file_size < self.gnss_validation_min_file_size_bytes:
                    invalid_reasons.append(
                        f'GNSS raw log smaller than {self.gnss_validation_min_file_size_bytes} bytes')
        else:
            invalid_reasons.append('GNSS raw log file path was never assigned')

        if not health.get('rawx_seen', False):
            invalid_reasons.append('RAWX observations were never seen')

        if not health.get('sfrbx_seen', False):
            warnings.append('SFRBX navigation pages were never seen')

        if bool(health.get('rawx_startup_failed', False)):
            invalid_reasons.append('RAWX did not appear within the startup window')

        raw_duration = float(health.get('raw_log_elapsed_sec') or 0.0)
        if raw_duration < self.gnss_validation_min_duration_sec:
            invalid_reasons.append(
                f'GNSS raw logging duration below {self.gnss_validation_min_duration_sec:.1f} s')

        min_ratio = float(health.get('rawx_min_coverage_ratio') or 0.0)
        coverage_ratio = float(health.get('rawx_coverage_ratio') or 0.0)
        if min_ratio > 0.0 and coverage_ratio < min_ratio:
            invalid_reasons.append(
                f'RAWX coverage ratio {coverage_ratio:.3f} below {min_ratio:.3f}')

        if int(health.get('logging_failures') or 0) > 0:
            invalid_reasons.append('GNSS raw logging recorded write/flush/fsync failures')

        max_gap = float(health.get('max_rawx_gap_sec') or 0.0)
        gap_fail = float(health.get('rawx_gap_fail_sec') or 0.0)
        if gap_fail > 0.0 and max_gap > gap_fail:
            invalid_reasons.append(
                f'RAWX max gap {max_gap:.2f} s exceeded failure threshold {gap_fail:.2f} s')

        for reason in health.get('warning_reasons', []):
            self.append_unique_reason(warnings, reason)
        for reason in health.get('invalid_reasons', []):
            self.append_unique_reason(invalid_reasons, reason)

        return warnings, invalid_reasons

    def replace_path_prefix(self, value, old_prefix: str, new_prefix: str):
        """Replace a path prefix in a stored string value if it matches."""
        if isinstance(value, str) and value.startswith(old_prefix):
            return new_prefix + value[len(old_prefix):]
        return value

    def session_config_path(self) -> Path:
        return Path(self.section_folder) / 'session_config.json'

    def update_session_config(self, updates: dict):
        if self.section_folder:
            self.update_json_file(self.session_config_path(), updates)

    def update_gnss_session(self, updates: dict):
        if self.gnss_session_file:
            self.update_json_file(Path(self.gnss_session_file), updates)

    def set_gps_raw_log_target(self, path_value: str):
        return self.set_node_param(
            self.gps_set_params_client, 'raw_log_target_path', path_value, 'gps_driver')

    def start_gps_raw_log(self, target_path: str):
        """Set gps_driver raw log target path and start logging.

        Returns:
          (success, message, hard_failure)
        """
        success, msg = self.set_gps_raw_log_target(target_path)
        if not success:
            return False, msg, True
        req = SetBool.Request()
        req.data = True
        success, msg = self.call_service_sync(
            self.gps_raw_log_set_client, req, '/gps/raw_log_set', timeout=5.0)
        return success, msg, False

    def stop_gps_raw_log(self):
        """Stop gps_driver raw logging."""
        req = SetBool.Request()
        req.data = False
        return self.call_service_sync(
            self.gps_raw_log_set_client, req, '/gps/raw_log_set', timeout=5.0)

    def promote_gps_raw_log(self, target_path: str):
        """Promote the current raw log into the session folder and keep appending there.

        Returns:
          (success, message, hard_failure)
        """
        success, msg = self.set_gps_raw_log_target(target_path)
        if not success:
            return False, msg, True
        success, msg = self.call_service_sync(
            self.gps_raw_log_promote_client, Trigger.Request(), '/gps/raw_log_promote', timeout=5.0)
        return success, msg, False

    def build_gnss_precapture_path(self) -> str:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        return str(Path(self.gnss_temp_directory) / f'gnss_precapture_{stamp}.ubx')

    def build_session_gnss_path(self) -> str:
        if not self.gnss_enabled or not self.gnss_data_folder:
            return ''
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return str(Path(self.gnss_data_folder) / f'rover_{stamp}.ubx')

    def delete_file_if_exists(self, path_str: str):
        if not path_str:
            return
        path = Path(path_str)
        try:
            if path.exists():
                path.unlink()
        except Exception as e:
            self.get_logger().warn(f'Failed to delete file {path}: {e}')

    def reset_gnss_precapture_state(self, clear_path: bool = True):
        self.gnss_precapture_active = False
        self.gnss_precapture_started_at = None
        self.gnss_precapture_started_monotonic = None
        if clear_path:
            self.gnss_precapture_path = ''

    def reset_gnss_session_state(self):
        # Note: this only resets PER-SECTION gnss bookkeeping. Mission-scoped
        # GNSS state (mission_gnss_*, mission_section_history) intentionally
        # persists across /dc/end_and_save so the same UBX file keeps logging
        # for the next section. /dc/finalize_mission resets the mission state.
        self.gnss_data_folder = ''
        self.gnss_session_file = ''
        self.gnss_raw_file = ''
        self.section_timestamp = ''
        self.preserved_gnss_precapture_paths = []
        self.session_gps_driver_boot_time = self.latest_gps_driver_boot_time
        self.gnss_driver_restarts = 0
        self.gnss_invalid = False
        self.gnss_invalid_reasons = []
        self.gnss_warning_reasons = []
        self.reset_gnss_precapture_state()

    # ================================================================
    #  Mission-scoped GNSS (continuous UBX across multiple sections)
    # ================================================================

    def reset_mission_state(self):
        self.mission_folder = ''
        self.mission_timestamp = ''
        self.mission_gnss_folder = ''
        self.mission_gnss_path = ''
        self.mission_gnss_started_at = None
        self.mission_gnss_active = False
        self.mission_section_history = []
        self.mission_active_section_start = None

    def mission_config_path(self) -> Path:
        if not self.mission_folder:
            return Path('')
        return Path(self.mission_folder) / 'mission_config.json'

    def update_mission_config(self, updates: dict):
        path = self.mission_config_path()
        if path and str(path):
            self.update_json_file(path, updates)

    def build_mission_gnss_path(self, mission_folder: Path, timestamp: str) -> str:
        gnss_folder = mission_folder / 'GNSS_data'
        gnss_folder.mkdir(parents=True, exist_ok=True)
        self.mission_gnss_folder = str(gnss_folder)
        return str(gnss_folder / f'rover_{datetime.now().strftime("%Y%m%d")}_{timestamp}.ubx')

    def ensure_mission_session(self) -> bool:
        """Lazily create the mission folder + start continuous GNSS logging.

        Returns True on success (mission already active is also success).
        Returns False on hard GNSS failure when GNSS is enabled.
        """
        if self.mission_gnss_active and self.mission_gnss_path:
            return True

        try:
            base = Path(self.base_data_dir)
            base.mkdir(parents=True, exist_ok=True)
            now = datetime.now()
            day_path = base / now.strftime('%B_%d_%Y')
            day_path.mkdir(parents=True, exist_ok=True)
            # Per-client tree: insert the building slug between day and
            # mission/section. AWS sync keys off this directly so each
            # client's data sits in a self-contained subtree.
            building_slug = self.current_building_slug()
            building_path = day_path / building_slug
            building_path.mkdir(parents=True, exist_ok=True)
            timestamp = now.strftime('%H%M%S')
            mission_path = building_path / f'Mission_{timestamp}'
            # Highly unlikely collision; if it happens, append seconds-ms.
            if mission_path.exists():
                mission_path = building_path / f'Mission_{timestamp}_{now.strftime("%f")[:3]}'
            mission_path.mkdir(parents=True, exist_ok=True)

            self.mission_folder = str(mission_path)
            self.mission_timestamp = timestamp
            self.mission_section_history = []
        except Exception as e:
            self.get_logger().error(f'Failed to create mission folder: {e}')
            return False

        session_metadata = self.current_session_metadata()

        if not self.gnss_enabled:
            # Indoor mode: no UBX log, but we still create the mission folder
            # so the per-section session_config can reference it for grouping.
            self.mission_gnss_active = False
            self.mission_gnss_path = ''
            self.mission_gnss_started_at = None
            mission_config = {
                'mission_folder': self.mission_folder,
                'mission_timestamp': self.mission_timestamp,
                'gnss_enabled': False,
                'mission_started_at': now.isoformat(),
                'mission_finalized_at': None,
                'mission_gnss_path': '',
                'sections': [],
            }
            mission_config.update(session_metadata)
            self.update_mission_config(mission_config)
            self.get_logger().info(
                f'Mission folder created (indoor mode, no GNSS): {self.mission_folder}')
            return True

        try:
            target_path = self.build_mission_gnss_path(Path(self.mission_folder), timestamp)
        except Exception as e:
            self.get_logger().error(f'Failed to prepare mission GNSS folder: {e}')
            return False

        # Promote precapture if active so we keep continuity from boot;
        # otherwise start a fresh session.
        if self.gnss_precapture_active and self.gnss_precapture_path:
            success, msg, hard_failure = self.promote_gps_raw_log(target_path)
            if not success:
                if hard_failure:
                    self.get_logger().error(
                        f'Mission GNSS bootstrap (promote) hard-failed: {msg}')
                    return False
                self.get_logger().error(
                    f'Mission GNSS promote failed: {msg} — falling through to fresh start')
                self.register_preserved_precapture_path(self.gnss_precapture_path)
            else:
                self.get_logger().info(f'Mission GNSS promoted from precapture: {msg}')
                self.mission_gnss_path = target_path
                self.mission_gnss_active = True

        if not self.mission_gnss_active:
            success, msg, hard_failure = self.start_gps_raw_log(target_path)
            if not success:
                if hard_failure:
                    self.get_logger().error(
                        f'Mission GNSS bootstrap (start) hard-failed: {msg}')
                    return False
                self.get_logger().error(
                    f'Mission GNSS raw log start failed: {msg} — mission GNSS will be unavailable')
                self.mark_gnss_invalid('Mission GNSS raw logging failed to start')
                return False
            self.mission_gnss_path = target_path
            self.mission_gnss_active = True
            self.get_logger().info(f'Mission GNSS raw logging started: {msg}')

        self.reset_gnss_precapture_state()
        self.mission_gnss_started_at = datetime.now().isoformat()
        mission_config = {
            'mission_folder': self.mission_folder,
            'mission_timestamp': self.mission_timestamp,
            'gnss_enabled': True,
            'mission_started_at': self.mission_gnss_started_at,
            'mission_finalized_at': None,
            'mission_gnss_path': self.mission_gnss_path,
            'sections': [],
        }
        mission_config.update(session_metadata)
        self.update_mission_config(mission_config)
        self.get_logger().info(f'Mission folder created: {self.mission_folder}')
        return True

    def mission_gnss_bytes_written(self) -> int:
        if self.mission_gnss_path and os.path.exists(self.mission_gnss_path):
            try:
                return os.path.getsize(self.mission_gnss_path)
            except OSError:
                return 0
        return 0

    def record_section_start_in_mission(self, section_name: str):
        self.mission_active_section_start = {
            'section_name': section_name,
            'section_folder': self.section_folder,
            'start_time': datetime.now().isoformat(),
            'gnss_bytes_at_start': self.mission_gnss_bytes_written(),
        }

    def record_section_end_in_mission(self, completion_tag: str, deleted: bool = False):
        if not self.mission_active_section_start:
            return
        entry = dict(self.mission_active_section_start)
        entry['end_time'] = datetime.now().isoformat()
        entry['gnss_bytes_at_end'] = self.mission_gnss_bytes_written()
        entry['completion_tag'] = completion_tag
        entry['deleted'] = deleted
        # Refresh section_folder in case it was renamed (e.g. _complete suffix).
        if self.section_folder:
            entry['section_folder'] = self.section_folder
        self.mission_section_history.append(entry)
        self.mission_active_section_start = None
        if self.mission_folder:
            self.update_mission_config({'sections': self.mission_section_history})

    def finalize_mission_callback(self, request, response):
        """Stop continuous mission GNSS logging and finalize mission_config.json.

        Called by the OCU when the operator presses Complete Mission, before
        the launch tree is torn down. Idempotent: returns success if no mission
        is active.
        """
        del request
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('FINALIZING MISSION')
        self.get_logger().info('=' * 60)

        if not self.mission_folder and not self.mission_gnss_active:
            response.success = True
            response.message = 'No active mission to finalize'
            self.get_logger().info(response.message)
            return response

        # Defensively close any in-flight section so its slice is captured
        # even if the operator hit Complete Mission mid-section.
        if self.mission_active_section_start:
            self.get_logger().warn(
                'Active section still open at finalize — recording as partial')
            self.record_section_end_in_mission('partial', deleted=False)

        gnss_stop_msg = 'GNSS was not active'
        gnss_stop_ok = True
        if self.gnss_enabled and self.mission_gnss_active:
            self.get_logger().info('Stopping continuous mission GNSS raw logging...')
            success, msg = self.stop_gps_raw_log()
            gnss_stop_ok = success
            gnss_stop_msg = msg or ''
            if success:
                self.get_logger().info(f'  Mission GNSS stopped: {msg}')
            else:
                self.get_logger().error(f'  Mission GNSS stop failed: {msg}')

            # Brief settle so the driver flushes the final UBX block before we
            # report bytes_written. 1 s is enough — the 30 s legacy delay was
            # for ZED-F9P RAWX cool-down and is not needed here because we
            # never tore down the driver during the mission.
            time.sleep(1.0)

        bytes_written = self.mission_gnss_bytes_written()
        finalized_at = datetime.now().isoformat()

        if self.mission_folder:
            self.update_mission_config({
                'mission_finalized_at': finalized_at,
                'mission_gnss_path': self.mission_gnss_path,
                'mission_gnss_bytes_written': bytes_written,
                'mission_gnss_stop_ok': gnss_stop_ok,
                'mission_gnss_stop_message': gnss_stop_msg,
                'sections': self.mission_section_history,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'driver_restarts': self.gnss_driver_restarts,
            })

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'MISSION FINALIZED — sections={len(self.mission_section_history)} '
            f'gnss_bytes={bytes_written}')
        self.get_logger().info('=' * 60)

        # Reset mission state so the next launch session (or the next Complete
        # Mission cycle within the same launch) starts fresh.
        self.reset_mission_state()
        self.mission_gnss_active = False

        response.success = True
        response.message = (
            f'Mission finalized: gnss_bytes={bytes_written}, gnss_stop_ok={gnss_stop_ok}'
        )
        return response

    def precapture_watchdog_callback(self):
        """Stop and delete stale GNSS precapture logs if /dc/start never happens."""
        if not self.gnss_enabled:
            return
        if not self.gnss_precapture_active or self.collection_started:
            return
        if self.gnss_precapture_started_monotonic is None:
            return
        elapsed = time.time() - self.gnss_precapture_started_monotonic
        if elapsed < self.gnss_precapture_timeout_sec:
            return

        self.get_logger().warn(
            f'GNSS precapture timed out after {elapsed:.1f}s — stopping temporary raw log')
        self.stop_gps_raw_log()
        self.delete_file_if_exists(self.gnss_precapture_path)
        self.reset_gnss_precapture_state()

    # ================================================================
    #  Section folder management
    # ================================================================

    def find_next_section_number(self, parent_path):
        """Find the next available section number under the given folder.

        With the per-building tree this is called with the building dir
        (`/R_DATA/<day>/<building_slug>/`) so two buildings on the same
        day each start at Section_1.
        """
        if not parent_path.exists():
            return 1

        section_folders = [d for d in parent_path.iterdir()
                           if d.is_dir() and d.name.startswith("Section_")]
        if not section_folders:
            return 1
        
        numbers = []
        for folder in section_folders:
            try:
                parts = folder.name.split("_")
                if len(parts) >= 2:
                    numbers.append(int(parts[1]))
            except (ValueError, IndexError):
                continue
        return (max(numbers) + 1) if numbers else 1

    def create_new_section(self):
        """
        Create a brand-new section folder with the current timestamp.
        Updates self.day_folder and self.section_folder, then pushes
        the new paths to gpr_scan_controller, unified_data_collector,
        and raw_map_saver.

        Returns True on success.
        """
        section_path = None
        try:
            base = Path(self.base_data_dir)
            base.mkdir(parents=True, exist_ok=True)

            # Day folder:  e.g.  /R_DATA/February_10_2026
            now = datetime.now()
            day_name = now.strftime("%B_%d_%Y")
            day_path = base / day_name
            day_path.mkdir(parents=True, exist_ok=True)
            self.day_folder = str(day_path)

            # Building folder:  e.g.  /R_DATA/February_10_2026/Acme_HQ
            # Same building scanned twice on the same day shares a single
            # folder — sections accumulate underneath it. find_next_section_number
            # therefore scans the building dir, not the day dir, so two
            # different buildings on the same day each start at Section_1.
            session_metadata = self.current_session_metadata()
            building_slug = session_metadata['building_slug']
            building_path = day_path / building_slug
            building_path.mkdir(parents=True, exist_ok=True)
            self.building_folder = str(building_path)

            # Section folder:
            # e.g.  /R_DATA/February_10_2026/Acme_HQ/Section_7_143022
            section_num = self.find_next_section_number(building_path)
            timestamp = now.strftime("%H%M%S")
            section_name = f"Section_{section_num}_{timestamp}"
            section_path = building_path / section_name
            section_path.mkdir(parents=True, exist_ok=True)

            # Sub-folders
            visual_data_folder = section_path / "Visual_data"
            gpr_scan_folder = section_path / "GPR_scan_data"
            visual_data_folder.mkdir(exist_ok=True)
            gpr_scan_folder.mkdir(exist_ok=True)

            # NOTE: We no longer create a per-section GNSS_data folder. The
            # raw UBX is logged once into the mission folder
            # (/R_DATA/<day>/Mission_<HHMMSS>/GNSS_data/rover_*.ubx) for the
            # entire mission. Per-section GNSS metadata still lives in
            # <section>/gnss_session.json and references the mission file.
            self.section_folder = str(section_path)
            self.section_timestamp = timestamp
            self.gnss_data_folder = ''
            self.gnss_session_file = ''
            self.gnss_raw_file = ''
            if self.gnss_enabled:
                self.gnss_session_file = str(section_path / 'gnss_session.json')

            self.get_logger().info('')
            self.get_logger().info(f'Created new section: {section_name}')
            self.get_logger().info(f'  Path:        {section_path}')
            self.get_logger().info(f'  Visual data: {visual_data_folder}')
            self.get_logger().info(f'  GPR data:    {gpr_scan_folder}')
            if self.gnss_enabled:
                self.get_logger().info('  GNSS data:   continuous mission UBX (see mission_folder)')
            else:
                self.get_logger().info('  GNSS data:   disabled for indoor mode')

            # Persist a session_config.json for offline tooling.
            # building_name (raw) + building_slug (path-safe) + operator
            # + units_preference are embedded so AWS-side tooling can
            # present the pretty name while keying off the slug.
            config = {
                'base_directory': str(base),
                'day_folder': str(day_path),
                'building_folder': str(building_path),
                'section_folder': str(section_path),
                'visual_data_folder': str(visual_data_folder),
                'gpr_scan_folder': str(gpr_scan_folder),
                'section_number': section_num,
                'timestamp': timestamp,
                'day_name': day_name,
                'section_name': section_name,
                'scan_mode': self.scan_mode,
                'gnss_enabled': self.gnss_enabled,
            }
            config.update(session_metadata)
            if self.gnss_enabled:
                config.update({
                    'gnss_data_folder': '',
                    'gnss_session_file': self.gnss_session_file,
                    'gnss_raw_file': self.mission_gnss_path,
                    'mission_folder': self.mission_folder,
                    'mission_gnss_path': self.mission_gnss_path,
                    'gnss_continuous_mission_mode': True,
                })
            with open(section_path / 'session_config.json', 'w') as f:
                json.dump(config, f, indent=2)
            if self.gnss_enabled:
                gnss_metadata = {
                    'section_folder': str(section_path),
                    'section_name': section_name,
                    'gnss_data_folder': '',
                    'mission_folder': self.mission_folder,
                    'mission_gnss_path': self.mission_gnss_path,
                    'gnss_continuous_mission_mode': True,
                    'raw_file_path': self.mission_gnss_path,
                    'precapture_start_time': self.gnss_precapture_started_at,
                    'collection_start_time': None,
                    'collection_end_time': None,
                    'bytes_written': 0,
                    'precapture_used': False,
                    'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
                    'status': 'section_created',
                    'health_level': 'idle',
                    'scan_mode': self.scan_mode,
                    'gnss_enabled': True,
                    'gnss_invalid': self.gnss_invalid,
                    'gnss_invalid_reasons': self.gnss_invalid_reasons,
                    'gnss_warning_reasons': self.gnss_warning_reasons,
                    'driver_boot_time': self.latest_gps_driver_boot_time,
                    'driver_restarts': self.gnss_driver_restarts,
                    'rawx_seen': False,
                    'sfrbx_seen': False,
                    'first_raw_obs_time': None,
                    'last_raw_obs_time': None,
                    'logging_failures': 0,
                }
                gnss_metadata.update(session_metadata)
                with open(self.gnss_session_file, 'w') as f:
                    json.dump(gnss_metadata, f, indent=2)

            # ---- Push new paths to downstream nodes ----

            # GPR scan controller
            self.get_logger().info('Updating gpr_scan_controller directory...')
            success, msg = self.set_node_param(self.gpr_set_params_client,
                                               'gpr_scan_data_directory',
                                               str(gpr_scan_folder),
                                               'gpr_scan_controller')
            if not success:
                self.get_logger().error(f'  GPR directory update FAILED: {msg}')
                raise RuntimeError(msg)
            success, msg = self.call_service_sync(
                self.gpr_set_dir_client, Trigger.Request(), '/gpr_scan/set_directory')
            if not success:
                self.get_logger().error(f'  GPR set_directory FAILED: {msg}')
                raise RuntimeError(f'gpr_scan_controller set_directory failed: {msg}')
            self.get_logger().info(f'  GPR directory  -> {gpr_scan_folder}')

            # Unified data collector
            self.get_logger().info('Updating unified_data_collector directory...')
            success, msg = self.set_node_param(self.udc_set_params_client,
                                               'visual_data_directory',
                                               str(visual_data_folder),
                                               'unified_data_collector')
            if not success:
                self.get_logger().error(f'  UDC directory update FAILED: {msg}')
                raise RuntimeError(msg)
            success, msg = self.call_service_sync(
                self.udc_set_dir_client, Trigger.Request(), '/udc/set_directory')
            if not success:
                self.get_logger().error(f'  UDC set_directory FAILED: {msg}')
                raise RuntimeError(f'unified_data_collector set_directory failed: {msg}')
            self.get_logger().info(f'  UDC directory  -> {visual_data_folder}')

            # Raw map saver
            self.get_logger().info('Updating raw_map_saver directory...')
            success, msg = self.set_node_param(self.map_set_params_client,
                                               'save_directory',
                                               str(section_path),
                                               'raw_map_saver')
            if not success:
                self.get_logger().error(f'  Map directory update FAILED: {msg}')
                raise RuntimeError(msg)
            success, msg = self.call_service_sync(
                self.map_set_dir_client, Trigger.Request(), '/raw_map_saver/set_directory')
            if not success:
                self.get_logger().error(f'  raw_map_saver set_directory FAILED: {msg}')
                raise RuntimeError(f'raw_map_saver set_directory failed: {msg}')
            self.get_logger().info(f'  Map directory  -> {section_path}')

            self.sync_gnss_status_to_metadata()

            return True

        except Exception as e:
            self.get_logger().error(f'Failed to create new section: {e}')
            if section_path and section_path.exists():
                try:
                    shutil.rmtree(section_path)
                except Exception as cleanup_error:
                    self.get_logger().warn(
                        f'Failed to clean up incomplete section folder {section_path}: {cleanup_error}')
            self.section_folder = ''
            self.gnss_data_folder = ''
            self.gnss_session_file = ''
            self.gnss_raw_file = ''
            self.reset_component_state()
            return False

    # ================================================================
    #  /dc/start_gnss_precapture
    # ================================================================

    def start_gnss_precapture_callback(self, request, response):
        """Start a temporary GNSS raw log before /dc/start creates a section folder."""
        del request

        if not self.gnss_enabled:
            self.get_logger().info('Indoor mode: GNSS precapture skipped')
            response.success = True
            response.message = 'Indoor mode: GNSS precapture skipped'
            return response

        if self.collection_started:
            response.success = True
            response.message = 'Collection already started - GNSS session logging already active'
            return response

        if self.gnss_precapture_active:
            response.success = True
            response.message = f'GNSS precapture already active: {self.gnss_precapture_path}'
            return response

        precapture_path = self.build_gnss_precapture_path()
        success, msg, _hard_failure = self.start_gps_raw_log(precapture_path)
        if success:
            self.gnss_precapture_active = True
            self.gnss_precapture_path = precapture_path
            self.gnss_precapture_started_at = datetime.now().isoformat()
            self.gnss_precapture_started_monotonic = time.time()
            self.fetch_gnss_health_snapshot()
            self.get_logger().info(f'GNSS precapture started: {precapture_path}')
            response.success = True
            response.message = f'GNSS precapture active: {precapture_path}'
        else:
            self.get_logger().error(f'GNSS precapture failed: {msg}')
            response.success = False
            response.message = f'GNSS precapture failed: {msg}'

        return response

    # ================================================================
    #  /dc/start
    # ================================================================

    def start_callback(self, request, response):
        """
        Start all data collection in sequence:
        0. Create a new section folder (with the current timestamp)
        1. GNSS - Start or promote raw UBX logging
        2. B - Start rosbag recording
        3. G - Start GPR scan (includes linear actuator UP + GPR motor + logging)
        4. R - Start video recording (RGB + thermal)
        """
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING DATA COLLECTION SEQUENCE')
        self.get_logger().info('='*60)
        
        if self.collection_started:
            response.success = False
            response.message = 'Data collection already started — call /dc/end_and_save first'
            self.get_logger().warn('Data collection already started')
            return response

        self.reset_component_state()

        # Echo the session metadata at the start of every /dc/start so the
        # coordinator log makes it obvious which building/operator a section
        # belongs to without having to grep session_config.json.
        meta = self.current_session_metadata()
        self.get_logger().info('')
        self.get_logger().info(
            f"Session metadata: building='{meta['building_name']}' "
            f"slug='{meta['building_slug']}' operator='{meta['operator_name']}' "
            f"units={meta['units_preference']}")

        # --- Step 0: fresh section folder ---
        self.get_logger().info('')
        self.get_logger().info('Step 0: Creating new section folder...')
        if not self.create_new_section():
            response.success = False
            response.message = 'Failed to create new section folder'
            self.get_logger().error('Aborting — could not create section folder')
            return response
        
        errors = []
        delay = self.start_delay

        # --- Step 1: Bootstrap mission GNSS (continuous across all sections) ---
        self.get_logger().info('')
        if self.gnss_enabled:
            collection_start_time = datetime.now().isoformat()
            precapture_temp_path = self.gnss_precapture_path
            precapture_start_time = self.gnss_precapture_started_at

            if self.mission_gnss_active and self.mission_gnss_path:
                self.get_logger().info(
                    f'Step 1/4: Mission GNSS already streaming continuously — reusing {os.path.basename(self.mission_gnss_path)}')
            else:
                self.get_logger().info('Step 1/4: Bootstrapping continuous mission GNSS raw logging...')
                if not self.ensure_mission_session():
                    self.cleanup_failed_start_section()
                    response.success = False
                    response.message = 'Failed to start collection: mission GNSS bootstrap failed'
                    return response

            # Per-section bookkeeping: this section's metadata still references
            # the (mission-wide) raw file, plus its own start/end timestamps so
            # the PPK pipeline can slice the .ubx by section.
            self.gnss_raw_file = self.mission_gnss_path
            self.record_section_start_in_mission(os.path.basename(self.section_folder))
            health_snapshot = self.fetch_gnss_health_snapshot()
            if health_snapshot:
                self.session_gps_driver_boot_time = health_snapshot.get(
                    'driver_boot_time', self.latest_gps_driver_boot_time)
            self.update_session_config({
                'mission_folder': self.mission_folder,
                'mission_gnss_path': self.mission_gnss_path,
                'gnss_raw_file': self.gnss_raw_file,
                'gnss_continuous_mission_mode': True,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
            })
            self.update_gnss_session({
                'raw_file_path': self.gnss_raw_file,
                'mission_folder': self.mission_folder,
                'mission_gnss_path': self.mission_gnss_path,
                'precapture_start_time': precapture_start_time,
                'collection_start_time': collection_start_time,
                'gnss_continuous_mission_mode': True,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
                'driver_boot_time': self.session_gps_driver_boot_time,
                'driver_restarts': self.gnss_driver_restarts,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'status': 'recording' if self.mission_gnss_active else 'recording_with_gnss_error',
            })
            if health_snapshot:
                self.update_gnss_session(self.extract_gnss_health_fields(health_snapshot))
            if precapture_temp_path and not self.mission_gnss_active:
                self.get_logger().warn(
                    f'  Preserved temp precapture for manual inspection: {precapture_temp_path}')
        else:
            self.get_logger().info('Step 1/4: Indoor mode - GNSS disabled for this session')
            # Still create a mission folder so per-section configs can group
            # under a single mission, even without GNSS.
            if not self.mission_folder:
                self.ensure_mission_session()
            self.record_section_start_in_mission(os.path.basename(self.section_folder))
            self.gnss_raw_file = ''
            self.reset_gnss_precapture_state()
        self.gnss_logging_active = bool(self.mission_gnss_active)

        # --- Step 2: rosbag (B key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 2/4: Starting rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_start_client, Trigger.Request(), '/rosbag/start')
        if not success:
            errors.append(f'Rosbag start: {msg}')
            self.get_logger().error(f'  Rosbag start failed: {msg}')
            self.rosbag_recording_active = False
            self.rosbag_paused = False
        else:
            self.get_logger().info(f'  Rosbag recording started: {msg}')
            self.rosbag_recording_active = True
            self.rosbag_paused = False

        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)

        # --- Step 3: GPR scan (G key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 3/4: Starting GPR scan (actuator + motor + logging)...')
        success, msg = self.call_service_sync(
            self.gpr_scan_start_client, Trigger.Request(), '/gpr_scan/start')
        if not success:
            errors.append(f'GPR scan start: {msg}')
            self.get_logger().error(f'  GPR scan start failed: {msg}')
            self.gpr_scan_active = False
            self.gpr_paused = False
        else:
            self.get_logger().info(f'  GPR scan started: {msg}')
            self.gpr_scan_active = True
            self.gpr_paused = False

        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)

        # --- Step 4: video (R key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 4/4: Starting video recording (RGB + thermal)...')
        req = SetBool.Request()
        req.data = True
        success, msg = self.call_service_sync(
            self.video_record_client, req, '/video_record_set')
        if not success:
            errors.append(f'Video start: {msg}')
            self.get_logger().error(f'  Video recording start failed: {msg}')
            self.video_recording_active = False
            self.video_paused = False
        else:
            self.get_logger().info(f'  Video recording started: {msg}')
            self.video_recording_active = True
            self.video_paused = False
        
        self.collection_started = True
        self.collection_active = True
        self.is_paused = False
        
        self.get_logger().info('')
        if errors:
            response.success = True
            response.message = f'Started with warnings: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            if self.gnss_enabled:
                response.message = f'All data collection started — {os.path.basename(self.section_folder)}'
            else:
                response.message = (
                    f'All data collection started — {os.path.basename(self.section_folder)} '
                    '(indoor mode, GNSS disabled)'
                )
            self.get_logger().info('='*60)
            self.get_logger().info('DATA COLLECTION STARTED')
            self.get_logger().info(f'  Section:  {os.path.basename(self.section_folder)}')
            if self.gnss_enabled:
                self.get_logger().info('  GNSS:     Enabled (outdoor mode)')
            else:
                self.get_logger().info('  GNSS:     Disabled (indoor mode)')
            self.get_logger().info(f'  Rosbag:   Recording')
            self.get_logger().info(f'  GPR:      Scanning (actuator UP, motor ON)')
            self.get_logger().info(f'  Video:    Recording (RGB + thermal)')
            self.get_logger().info('='*60)
        
        return response

    # ================================================================
    #  /dc/pause & /dc/resume
    # ================================================================

    def pause_callback(self, request, response):
        """Pause all data collection"""
        del request
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('PAUSING DATA COLLECTION')
        self.get_logger().info('='*60)
        
        if self.is_paused:
            response.success = True
            response.message = 'Already paused'
            return response
        
        errors = []
        if self.video_recording_active or self.collection_started:
            self.get_logger().info('Pausing unified_data_collector...')
            success, msg = self.call_service_sync(
                self.udc_pause_client, Trigger.Request(), '/udc/pause')
            if success or self._msg_contains(msg, 'already paused'):
                self.video_recording_active = True
                self.video_paused = True
                self.get_logger().info('  unified_data_collector paused')
            elif self._msg_contains(msg, 'no active recording', 'already stopped'):
                self.video_recording_active = False
                self.video_paused = False
                self.get_logger().warn('  unified_data_collector was not actively recording')
            else:
                errors.append(f'UDC pause: {msg}')
        else:
            self.get_logger().info('Skipping unified_data_collector pause (not recording)')

        if self.gpr_scan_active or self.collection_started:
            self.get_logger().info('Pausing gpr_scan_controller...')
            success, msg = self.call_service_sync(
                self.gpr_pause_client, Trigger.Request(), '/gpr_scan/pause')
            if success or self._msg_contains(msg, 'already paused'):
                self.gpr_scan_active = True
                self.gpr_paused = True
                self.get_logger().info('  gpr_scan_controller paused')
            elif self._msg_contains(msg, 'gpr scan not active'):
                self.gpr_scan_active = False
                self.gpr_paused = False
                self.get_logger().warn('  gpr_scan_controller was not actively scanning')
            else:
                errors.append(f'GPR pause: {msg}')
        else:
            self.get_logger().info('Skipping gpr_scan_controller pause (not scanning)')

        if self.rosbag_recording_active or self.collection_started:
            self.get_logger().info('Pausing rosbag recording...')
            success, msg = self.call_service_sync(
                self.rosbag_pause_client, Trigger.Request(), '/rosbag/pause')
            if success or self._msg_contains(msg, 'already paused'):
                self.rosbag_recording_active = True
                self.rosbag_paused = True
                self.get_logger().info('  rosbag recording paused')
            elif self._msg_contains(msg, 'no rosbag recording active'):
                self.rosbag_recording_active = False
                self.rosbag_paused = False
                self.get_logger().warn('  rosbag recording was not active')
            else:
                errors.append(f'Rosbag pause: {msg}')
        else:
            self.get_logger().info('Skipping rosbag pause (not recording)')

        self.is_paused = not errors
        
        if errors:
            response.success = False
            response.message = f'Paused with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = 'All data collection paused'
            self.get_logger().info('All data collection PAUSED')
        
        self.get_logger().info('='*60)
        return response

    def resume_callback(self, request, response):
        """Resume all data collection"""
        del request
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('RESUMING DATA COLLECTION')
        self.get_logger().info('='*60)
        
        if not self.is_paused:
            response.success = True
            response.message = 'Not paused'
            return response
        
        errors = []
        if self.video_recording_active and self.video_paused:
            self.get_logger().info('Resuming unified_data_collector...')
            success, msg = self.call_service_sync(
                self.udc_resume_client, Trigger.Request(), '/udc/resume')
            if success or self._msg_contains(msg, 'not paused'):
                self.video_paused = False
                self.get_logger().info('  unified_data_collector resumed')
            elif self._msg_contains(msg, 'no active recording', 'already stopped'):
                self.video_recording_active = False
                self.video_paused = False
                self.get_logger().warn('  unified_data_collector was not actively recording')
            else:
                errors.append(f'UDC resume: {msg}')
        else:
            self.get_logger().info('Skipping unified_data_collector resume (not paused)')

        if self.gpr_scan_active and self.gpr_paused:
            self.get_logger().info('Resuming gpr_scan_controller...')
            success, msg = self.call_service_sync(
                self.gpr_resume_client, Trigger.Request(), '/gpr_scan/resume')
            if success or self._msg_contains(msg, 'not paused'):
                self.gpr_paused = False
                self.get_logger().info('  gpr_scan_controller resumed')
            elif self._msg_contains(msg, 'gpr scan not active'):
                self.gpr_scan_active = False
                self.gpr_paused = False
                self.get_logger().warn('  gpr_scan_controller was not actively scanning')
            else:
                errors.append(f'GPR resume: {msg}')
        else:
            self.get_logger().info('Skipping gpr_scan_controller resume (not paused)')

        if self.rosbag_recording_active and self.rosbag_paused:
            self.get_logger().info('Resuming rosbag recording...')
            success, msg = self.call_service_sync(
                self.rosbag_resume_client, Trigger.Request(), '/rosbag/resume')
            if success or self._msg_contains(msg, 'not paused'):
                self.rosbag_paused = False
                self.get_logger().info('  rosbag recording resumed')
            elif self._msg_contains(msg, 'no rosbag recording active'):
                self.rosbag_recording_active = False
                self.rosbag_paused = False
                self.get_logger().warn('  rosbag recording was not active')
            else:
                errors.append(f'Rosbag resume: {msg}')
        else:
            self.get_logger().info('Skipping rosbag resume (not paused)')

        self.is_paused = bool(errors)
        
        if errors:
            response.success = False
            response.message = f'Resumed with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = 'All data collection resumed'
            self.get_logger().info('All data collection RESUMED')
        
        self.get_logger().info('='*60)
        return response

    # ================================================================
    #  /dc/end_and_save
    # ================================================================

    def end_and_save_callback(self, request, response):
        """
        End data collection and save with tag.
        Sequence mirrors the stop keys: T -> G -> B -> M

        request.data: False (0) = partial, True (1) = complete
        """
        tag = 'complete' if request.data else 'partial'
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'ENDING DATA COLLECTION — TAG: {tag.upper()}')
        self.get_logger().info('='*60)
        
        errors = []
        final_tag = tag
        
        # Step 1: Stop video recording (T key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 1/4: Stopping video recording + CSV logging...')
        if self.video_recording_active or self.collection_started:
            req = SetBool.Request()
            req.data = False
            success, msg = self.call_service_sync(
                self.video_record_client, req, '/video_record_set')
            if success or self._msg_contains(msg, 'already stopped', 'no active recording'):
                self.video_recording_active = False
                self.video_paused = False
                self.get_logger().info('  Video recording + CSV logging stopped')
            else:
                errors.append(f'Video stop: {msg}')
                self.get_logger().error(f'  Video stop failed: {msg}')
        else:
            self.get_logger().info('  Video recording was already stopped')
        
        # Step 2: Stop GPR scan (G key toggle equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 2/4: Stopping GPR scan (motor + logging + actuator)...')
        if self.gpr_scan_active or self.gpr_paused or self.collection_started:
            success, msg = self.call_service_sync(
                self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
            if success or self._msg_contains(msg, 'gpr scan not active'):
                self.gpr_scan_active = False
                self.gpr_paused = False
                self.get_logger().info('  GPR scan stopped (actuator lowered)')
            else:
                errors.append(f'GPR stop: {msg}')
                self.get_logger().error(f'  GPR stop failed: {msg}')
        else:
            self.get_logger().info('  GPR scan was already stopped')
        
        # Step 3: Stop rosbag recording (B key toggle equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 3/4: Stopping rosbag recording...')
        if self.rosbag_recording_active or self.rosbag_paused or self.collection_started:
            success, msg = self.call_service_sync(
                self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
            if success or self._msg_contains(msg, 'no rosbag recording active'):
                self.rosbag_recording_active = False
                self.rosbag_paused = False
                self.get_logger().info('  Rosbag recording stopped')
            else:
                errors.append(f'Rosbag stop: {msg}')
                self.get_logger().error(f'  Rosbag stop failed: {msg}')
        else:
            self.get_logger().info('  Rosbag recording was already stopped')
        
        # Step 4: GNSS keeps streaming continuously across sections — we no
        # longer stop the raw UBX log here. The mission-wide log is closed
        # only by /dc/finalize_mission (or launch shutdown). We still snapshot
        # bytes written so per-section metadata records the slice boundary.
        self.get_logger().info('')
        if self.gnss_enabled:
            self.get_logger().info(
                'Step 4/5: GNSS continuous mode — keeping mission raw log active across section boundary')
            collection_end_time = datetime.now().isoformat()
            gnss_bytes_written = self.mission_gnss_bytes_written()
            health_snapshot = self.fetch_gnss_health_snapshot()
            warnings, invalid_reasons = self.validate_gnss_session(health_snapshot)
            for warning in warnings:
                self.append_unique_reason(self.gnss_warning_reasons, warning)
            for invalid_reason in invalid_reasons:
                self.append_unique_reason(self.gnss_invalid_reasons, invalid_reason)
            if invalid_reasons:
                self.gnss_invalid = True

            final_tag = f'{tag}_gnss_invalid' if self.gnss_invalid else tag

            self.update_session_config({
                'mission_folder': self.mission_folder,
                'mission_gnss_path': self.mission_gnss_path,
                'gnss_raw_file': self.gnss_raw_file,
                'gnss_bytes_written': gnss_bytes_written,
                'gnss_continuous_mission_mode': True,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'driver_restarts': self.gnss_driver_restarts,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
            })
            self.update_gnss_session({
                'raw_file_path': self.gnss_raw_file,
                'mission_folder': self.mission_folder,
                'mission_gnss_path': self.mission_gnss_path,
                'collection_end_time': collection_end_time,
                'bytes_written': gnss_bytes_written,
                'gnss_continuous_mission_mode': True,
                'status': 'saved' if not errors else 'saved_with_warnings',
                'completion_tag': final_tag,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'driver_restarts': self.gnss_driver_restarts,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
            })
            if health_snapshot:
                self.update_gnss_session(self.extract_gnss_health_fields(health_snapshot))
        else:
            self.get_logger().info('Step 4/5: Indoor mode - skipping GNSS shutdown and validation')

        # Step 5: Save map (M key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 5/5: Saving point cloud map...')
        success, msg = self.call_service_sync(
            self.save_map_client, Trigger.Request(), '/save_raw_map', timeout=15.0)
        if not success:
            errors.append(f'Map save: {msg}')
            self.get_logger().error(f'  Map save failed: {msg}')
        else:
            self.get_logger().info(f'  Point cloud map saved: {msg}')
        
        # Step 6: Rename section folder with tag
        self.get_logger().info('')
        self.get_logger().info('Renaming section folder...')
        new_folder_path = self._rename_section_folder(final_tag)
        if new_folder_path:
            self.get_logger().info(f'  Section folder renamed to: {os.path.basename(new_folder_path)}')
        else:
            errors.append('Failed to rename section folder')
            self.get_logger().error('  Failed to rename section folder')
        self.update_gnss_session({
            'status': 'saved' if not errors else 'saved_with_warnings',
            'completion_tag': final_tag,
            'gnss_invalid': self.gnss_invalid,
            'gnss_invalid_reasons': self.gnss_invalid_reasons,
            'gnss_warning_reasons': self.gnss_warning_reasons,
        })

        # Append the (renamed) section to the mission history so PPK can
        # later slice the continuous UBX into per-section windows.
        self.record_section_end_in_mission(final_tag, deleted=False)

        # Reset state — ready for next /dc/start
        self.collection_started = False
        self.is_paused = False
        self.reset_component_state()
        self.reset_gnss_session_state()
        
        self.get_logger().info('')
        if errors:
            response.success = True
            response.message = f'Ended with warnings: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            if self.gnss_enabled:
                response.message = f'Data collection ended and saved as {final_tag}'
            else:
                response.message = (
                    f'Data collection ended and saved as {final_tag} '
                    '(indoor mode, GNSS disabled)'
                )
            self.get_logger().info('='*60)
            if self.gnss_enabled:
                self.get_logger().info(f'DATA COLLECTION ENDED AND SAVED AS {final_tag.upper()}')
            else:
                self.get_logger().info(
                    f'DATA COLLECTION ENDED AND SAVED AS {final_tag.upper()} (INDOOR MODE)')
            self.get_logger().info('='*60)
        
        self.get_logger().info('Ready for new section (call /dc/start)')
        
        return response

    # ================================================================
    #  /dc/end_and_delete
    # ================================================================

    def end_and_delete_callback(self, request, response):
        """
        End data collection and delete all session data.
        Stops all processes and removes the entire Section folder.
        """
        del request
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('ENDING DATA COLLECTION — DELETING ALL DATA')
        self.get_logger().info('='*60)
        
        # Step 1: Stop video recording + CSV logging
        self.get_logger().info('')
        self.get_logger().info('Step 1/5: Stopping video recording + CSV logging...')
        if self.video_recording_active or self.collection_started:
            req = SetBool.Request()
            req.data = False
            self.call_service_sync(self.video_record_client, req, '/video_record_set')
            self.video_recording_active = False
            self.video_paused = False
            self.get_logger().info('  Video recording + CSV logging stopped')
        else:
            self.get_logger().info('  Video recording was already stopped')
        
        # Step 2: Stop GPR scan controller
        self.get_logger().info('')
        self.get_logger().info('Step 2/5: Stopping GPR scan (motor + logging + actuator)...')
        if self.gpr_scan_active or self.gpr_paused or self.collection_started:
            self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
            self.gpr_scan_active = False
            self.gpr_paused = False
            self.get_logger().info('  gpr_scan_controller stopped (actuator lowered)')
        else:
            self.get_logger().info('  gpr_scan_controller was already stopped')
        
        # Step 3: Stop rosbag recording
        self.get_logger().info('')
        self.get_logger().info('Step 3/5: Stopping rosbag recording...')
        if self.rosbag_recording_active or self.rosbag_paused or self.collection_started:
            self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
            self.rosbag_recording_active = False
            self.rosbag_paused = False
            self.get_logger().info('  rosbag recording stopped')
        else:
            self.get_logger().info('  rosbag recording was already stopped')
        
        # Step 4: Mission GNSS keeps streaming continuously — discarding a
        # single section never stops the mission UBX log. We only clean up
        # any leftover precapture temp files (which only exist before the
        # first /dc/start of a launch session).
        self.get_logger().info('')
        deleted_precapture_artifacts = False
        if self.gnss_enabled:
            self.get_logger().info(
                'Step 4/5: GNSS continuous mode — mission raw log unaffected by section discard')
            if self.gnss_precapture_path:
                self.delete_file_if_exists(self.gnss_precapture_path)
                self.get_logger().info('  GNSS precapture temp file deleted')
                deleted_precapture_artifacts = True
            if self.preserved_gnss_precapture_paths:
                self.delete_preserved_precapture_paths()
                self.get_logger().info('  Preserved GNSS precapture temp files deleted')
                deleted_precapture_artifacts = True
        else:
            self.get_logger().info('Step 4/5: Indoor mode - no GNSS data to stop or delete')
        
        # Step 5: Delete the entire section folder
        self.get_logger().info('')
        self.get_logger().info('Step 5/5: Deleting session data...')
        if self.section_folder and os.path.exists(self.section_folder):
            try:
                self.get_logger().info(f'  Deleting: {self.section_folder}')
                shutil.rmtree(self.section_folder)
                self.get_logger().info('  Section folder deleted')
                response.success = True
                response.message = f'Data collection ended and deleted: {self.section_folder}'
            except Exception as e:
                self.get_logger().error(f'  Failed to delete folder: {e}')
                response.success = False
                response.message = f'Failed to delete folder: {e}'
        else:
            if deleted_precapture_artifacts:
                response.success = True
                response.message = 'GNSS precapture artifacts deleted - no section folder existed yet'
            else:
                self.get_logger().warn(f'  Section folder not found: {self.section_folder}')
                response.success = False
                response.message = f'Section folder not found: {self.section_folder}'
        
        # Record this section as deleted in the mission history (if a mission
        # is active) so PPK can mark the slice as discarded.
        self.record_section_end_in_mission('deleted', deleted=True)

        # Reset state
        self.collection_started = False
        self.is_paused = False
        self.section_folder = ''
        self.reset_component_state()
        self.reset_gnss_session_state()

        self.get_logger().info('')
        if response.success:
            self.get_logger().info('='*60)
            self.get_logger().info('DATA COLLECTION ENDED AND DELETED')
            self.get_logger().info('='*60)
            self.get_logger().info('Ready for new section (call /dc/start)')
        
        return response

    def cancel_scan_callback(self, request, response):
        """
        Abort the entire scan: stop active data collection, then delete every
        section folder produced this mission (including the in-flight section)
        and the mission folder itself (continuous UBX log + mission_config.json).

        Pipeline processes (unified_data_collector, gpr_scan_controller,
        rosbag2, gps_driver) stay alive — only their data on disk is removed
        and their session state is reset. The OCU is expected to navigate the
        operator back to Map Processing after this call returns.
        """
        del request
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('CANCELLING SCAN — deleting ALL collected mission data')
        self.get_logger().info('=' * 60)

        # Snapshot the mission folder + history BEFORE we tear anything down,
        # because reset_mission_state() below clears them.
        mission_folder_snapshot = self.mission_folder
        section_history_snapshot = list(self.mission_section_history)
        active_section_folder = self.section_folder

        errors = []

        # Step 1: Stop video recording + CSV logging
        self.get_logger().info('')
        self.get_logger().info('Step 1/6: Stopping video recording + CSV logging...')
        if self.video_recording_active or self.collection_started:
            req = SetBool.Request()
            req.data = False
            self.call_service_sync(self.video_record_client, req, '/video_record_set')
            self.video_recording_active = False
            self.video_paused = False
            self.get_logger().info('  Video recording + CSV logging stopped')
        else:
            self.get_logger().info('  Video recording was already stopped')

        # Step 2: Stop GPR scan controller (lowers actuator)
        self.get_logger().info('')
        self.get_logger().info('Step 2/6: Stopping GPR scan (motor + logging + actuator)...')
        if self.gpr_scan_active or self.gpr_paused or self.collection_started:
            self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
            self.gpr_scan_active = False
            self.gpr_paused = False
            self.get_logger().info('  gpr_scan_controller stopped (actuator lowered)')
        else:
            self.get_logger().info('  gpr_scan_controller was already stopped')

        # Step 3: Stop rosbag recording
        self.get_logger().info('')
        self.get_logger().info('Step 3/6: Stopping rosbag recording...')
        if self.rosbag_recording_active or self.rosbag_paused or self.collection_started:
            self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
            self.rosbag_recording_active = False
            self.rosbag_paused = False
            self.get_logger().info('  rosbag recording stopped')
        else:
            self.get_logger().info('  rosbag recording was already stopped')

        # Step 4: Stop continuous mission GNSS raw logging (entire mission is
        # being thrown away — no need to preserve any UBX bytes).
        self.get_logger().info('')
        if self.gnss_enabled and self.mission_gnss_active:
            self.get_logger().info('Step 4/6: Stopping continuous mission GNSS raw logging...')
            success, msg = self.stop_gps_raw_log()
            if success:
                self.get_logger().info(f'  Mission GNSS stopped: {msg}')
            else:
                err = f'GNSS stop failed: {msg}'
                self.get_logger().error(f'  {err}')
                errors.append(err)
            # No 30 s drain — we're deleting the file anyway.
            time.sleep(0.5)
        else:
            self.get_logger().info('Step 4/6: GNSS not active — nothing to stop')

        # Also clean any leftover precapture artifacts that might still be on
        # disk (only relevant if cancel fired before the first /dc/start).
        if self.gnss_enabled:
            if self.gnss_precapture_path:
                self.delete_file_if_exists(self.gnss_precapture_path)
            if self.preserved_gnss_precapture_paths:
                self.delete_preserved_precapture_paths()

        # Step 5: Delete the in-flight section folder (if one was started this
        # mission and still exists on disk under its original or _partial name).
        self.get_logger().info('')
        self.get_logger().info('Step 5/6: Deleting current section folder...')
        if active_section_folder and os.path.exists(active_section_folder):
            try:
                self.get_logger().info(f'  Deleting: {active_section_folder}')
                shutil.rmtree(active_section_folder)
                self.get_logger().info('  Current section folder deleted')
            except Exception as e:
                err = f'Failed to delete current section: {e}'
                self.get_logger().error(f'  {err}')
                errors.append(err)
        else:
            self.get_logger().info('  No current section folder to delete')

        # Step 6: Delete every prior section folder recorded in the mission
        # history, then delete the mission folder itself (UBX + config + any
        # leftover children). This is best-effort: missing/already-deleted
        # paths are silently skipped, but real failures are reported.
        self.get_logger().info('')
        self.get_logger().info(
            f'Step 6/6: Deleting {len(section_history_snapshot)} prior section folder(s) '
            f'+ mission folder...')

        deleted_sections = 0
        for entry in section_history_snapshot:
            section_path = entry.get('section_folder')
            if not section_path:
                continue
            if section_path == active_section_folder:
                continue  # already handled in Step 5
            if not os.path.exists(section_path):
                continue
            try:
                self.get_logger().info(f'  Deleting prior section: {section_path}')
                shutil.rmtree(section_path)
                deleted_sections += 1
            except Exception as e:
                err = f'Failed to delete {section_path}: {e}'
                self.get_logger().error(f'  {err}')
                errors.append(err)

        if mission_folder_snapshot and os.path.exists(mission_folder_snapshot):
            try:
                self.get_logger().info(f'  Deleting mission folder: {mission_folder_snapshot}')
                shutil.rmtree(mission_folder_snapshot)
                self.get_logger().info('  Mission folder deleted (GNSS + config + leftovers)')
            except Exception as e:
                err = f'Failed to delete mission folder {mission_folder_snapshot}: {e}'
                self.get_logger().error(f'  {err}')
                errors.append(err)
        else:
            self.get_logger().info('  No mission folder on disk to delete')

        # Reset all per-mission and per-section state so the next /dc/start
        # bootstraps a fresh mission from scratch.
        self.collection_started = False
        self.collection_active = True
        self.is_paused = False
        self.section_folder = ''
        self.day_folder = ''
        self.building_folder = ''
        self.section_number = 0
        self.reset_component_state()
        self.reset_gnss_session_state()
        self.reset_mission_state()
        self.mission_gnss_active = False

        # Tell every other DC-aware node to drop its local state. Critical:
        # mpc_accel_autonomous_controller's dc_active / pause_reasons /
        # waypoint nav state are process-local and would otherwise stay stale
        # across the cancel boundary, blocking the next /dc/start (EC-001 in
        # docs/instructions.md). We publish even if errors occurred above —
        # the coordinator's own state has already been wiped, so the
        # controller MUST follow or the next scan will desync.
        try:
            reset_msg = String()
            reset_msg.data = 'cancelled'
            self.dc_state_reset_pub.publish(reset_msg)
            self.get_logger().info('  Published /dc/state_reset = "cancelled"')
        except Exception as e:
            self.get_logger().error(f'  Failed to publish /dc/state_reset: {e}')

        if errors:
            response.success = False
            response.message = (
                f'Scan cancelled with errors: deleted {deleted_sections} prior section(s); '
                f'errors: {"; ".join(errors)}')
            self.get_logger().error(response.message)
        else:
            response.success = True
            response.message = (
                f'Scan cancelled — deleted current section + {deleted_sections} prior section(s) '
                f'+ mission folder')

        self.get_logger().info('=' * 60)
        self.get_logger().info('SCAN CANCELLED — pipeline still running, ready for next mission')
        self.get_logger().info('=' * 60)
        return response

    # ================================================================
    #  Internal helpers
    # ================================================================

    def _rename_section_folder(self, tag):
        """
        Rename section folder to include tag (partial/complete).
        Section_1_123456 -> Section_1_123456_complete
        """
        if not self.section_folder or not os.path.exists(self.section_folder):
            self.get_logger().error(f'Section folder not found: {self.section_folder}')
            return None
        
        folder_path = Path(self.section_folder)
        current_name = folder_path.name
        
        # Already tagged?
        tagged_suffixes = (
            '_partial',
            '_complete',
            '_partial_gnss_invalid',
            '_complete_gnss_invalid',
        )
        if current_name.endswith(tagged_suffixes):
            self.get_logger().info(f'Folder already tagged: {current_name}')
            return str(folder_path)
        
        new_name = f'{current_name}_{tag}'
        new_path = folder_path.parent / new_name
        
        try:
            old_path_str = str(folder_path)
            folder_path.rename(new_path)
            self.section_folder = str(new_path)
            self.gnss_data_folder = self.replace_path_prefix(
                self.gnss_data_folder, old_path_str, str(new_path))
            self.gnss_session_file = self.replace_path_prefix(
                self.gnss_session_file, old_path_str, str(new_path))
            self.gnss_raw_file = self.replace_path_prefix(
                self.gnss_raw_file, old_path_str, str(new_path))
            
            # Update session_config.json
            config_file = new_path / 'session_config.json'
            if config_file.exists():
                config_updates = {
                    'section_folder': str(new_path),
                    'section_name': new_name,
                    'completion_tag': tag,
                    'scan_mode': self.scan_mode,
                    'gnss_enabled': self.gnss_enabled,
                }
                if self.gnss_enabled:
                    config_updates.update({
                        'gnss_data_folder': self.gnss_data_folder,
                        'gnss_session_file': self.gnss_session_file,
                        'gnss_raw_file': self.gnss_raw_file,
                        'gnss_invalid': self.gnss_invalid,
                        'gnss_invalid_reasons': self.gnss_invalid_reasons,
                        'gnss_warning_reasons': self.gnss_warning_reasons,
                        'driver_restarts': self.gnss_driver_restarts,
                        'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
                    })
                self.update_json_file(config_file, config_updates)

            gnss_file = Path(self.gnss_session_file) if self.gnss_session_file else None
            if self.gnss_enabled and gnss_file and gnss_file.exists():
                self.update_json_file(gnss_file, {
                    'section_folder': str(new_path),
                    'section_name': new_name,
                    'completion_tag': tag,
                    'gnss_data_folder': self.gnss_data_folder,
                    'raw_file_path': self.gnss_raw_file,
                    'gnss_invalid': self.gnss_invalid,
                    'gnss_invalid_reasons': self.gnss_invalid_reasons,
                    'gnss_warning_reasons': self.gnss_warning_reasons,
                    'driver_restarts': self.gnss_driver_restarts,
                    'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
                })
            
            return str(new_path)
        except Exception as e:
            self.get_logger().error(f'Failed to rename folder: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionCoordinator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
