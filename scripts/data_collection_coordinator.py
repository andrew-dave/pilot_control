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
        self.section_folder = ''
        self.section_timestamp = ''
        
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
        self.get_logger().info('')
        self.get_logger().info('Section folders are created dynamically on /dc/start')
        self.get_logger().info('='*60)

    # ================================================================
    #  Helpers
    # ================================================================

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
        self.section_folder = ''
        self.section_timestamp = ''
        self.gnss_data_folder = ''
        self.gnss_session_file = ''
        self.gnss_raw_file = ''

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

    def find_next_section_number(self, day_path):
        """Find the next available section number in the given day folder."""
        if not day_path.exists():
            return 1
        
        section_folders = [d for d in day_path.iterdir()
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

            # Section folder:  e.g.  /R_DATA/February_10_2026/Section_7_143022
            section_num = self.find_next_section_number(day_path)
            timestamp = now.strftime("%H%M%S")
            section_name = f"Section_{section_num}_{timestamp}"
            section_path = day_path / section_name
            section_path.mkdir(parents=True, exist_ok=True)

            # Sub-folders
            visual_data_folder = section_path / "Visual_data"
            gpr_scan_folder = section_path / "GPR_scan_data"
            visual_data_folder.mkdir(exist_ok=True)
            gpr_scan_folder.mkdir(exist_ok=True)

            gnss_data_folder = None
            self.section_folder = str(section_path)
            self.section_timestamp = timestamp
            self.gnss_data_folder = ''
            self.gnss_session_file = ''
            self.gnss_raw_file = ''
            if self.gnss_enabled:
                gnss_data_folder = section_path / "GNSS_data"
                gnss_data_folder.mkdir(exist_ok=True)
                self.gnss_data_folder = str(gnss_data_folder)
                self.gnss_session_file = str(section_path / 'gnss_session.json')

            self.get_logger().info('')
            self.get_logger().info(f'Created new section: {section_name}')
            self.get_logger().info(f'  Path:        {section_path}')
            self.get_logger().info(f'  Visual data: {visual_data_folder}')
            self.get_logger().info(f'  GPR data:    {gpr_scan_folder}')
            if self.gnss_enabled:
                self.get_logger().info(f'  GNSS data:   {gnss_data_folder}')
            else:
                self.get_logger().info('  GNSS data:   disabled for indoor mode')

            # Persist a session_config.json for offline tooling
            config = {
                'base_directory': str(base),
                'day_folder': str(day_path),
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
            if self.gnss_enabled:
                config.update({
                    'gnss_data_folder': str(gnss_data_folder),
                    'gnss_session_file': self.gnss_session_file,
                    'gnss_raw_file': '',
                })
            with open(section_path / 'session_config.json', 'w') as f:
                json.dump(config, f, indent=2)
            if self.gnss_enabled:
                gnss_metadata = {
                    'section_folder': str(section_path),
                    'section_name': section_name,
                    'gnss_data_folder': str(gnss_data_folder),
                    'raw_file_path': '',
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

        # --- Step 1: GNSS raw logging ---
        self.get_logger().info('')
        if self.gnss_enabled:
            self.get_logger().info('Step 1/4: Starting/promoting GNSS raw logging...')
            session_gnss_path = self.build_session_gnss_path()
            collection_start_time = datetime.now().isoformat()
            precapture_start_time = self.gnss_precapture_started_at
            precapture_temp_path = self.gnss_precapture_path
            precapture_used = False
            gnss_ok = False
            preserved_precapture = False

            if self.gnss_precapture_active and self.gnss_precapture_path:
                success, msg, hard_failure = self.promote_gps_raw_log(session_gnss_path)
                if success:
                    precapture_used = True
                    gnss_ok = True
                    self.get_logger().info(f'  GNSS precapture promoted: {msg}')
                else:
                    if hard_failure:
                        self.get_logger().error(f'  Hard failure during GNSS precapture promotion: {msg}')
                        self.cleanup_failed_start_section()
                        response.success = False
                        response.message = f'Failed to start collection: {msg}'
                        return response
                    errors.append(f'GNSS promote: {msg}')
                    self.mark_gnss_invalid('GNSS precapture promotion failed; pre-motion GNSS continuity lost')
                    self.register_preserved_precapture_path(precapture_temp_path)
                    preserved_precapture = True
                    self.get_logger().error('  !!! GNSS RAW PROMOTE FAILED - PPK PRECAPTURE MAY BE LOST !!!')
                    self.get_logger().error(f'  GNSS promote failed and temp file preserved: {msg}')

            if not gnss_ok:
                success, msg, hard_failure = self.start_gps_raw_log(session_gnss_path)
                if success:
                    gnss_ok = True
                    self.get_logger().info(f'  GNSS raw logging started: {msg}')
                else:
                    if hard_failure:
                        self.get_logger().error(f'  Hard failure during GNSS raw log start: {msg}')
                        self.cleanup_failed_start_section()
                        response.success = False
                        response.message = f'Failed to start collection: {msg}'
                        return response
                    errors.append(f'GNSS start: {msg}')
                    self.mark_gnss_invalid('GNSS raw logging failed to start')
                    self.get_logger().error('  !!! GNSS RAW LOGGING FAILED - PPK DATA WILL BE UNAVAILABLE !!!')
                    self.get_logger().error(f'  GNSS raw log start failed: {msg}')

            self.reset_gnss_precapture_state()

            self.gnss_raw_file = session_gnss_path if gnss_ok else ''
            health_snapshot = self.fetch_gnss_health_snapshot()
            if health_snapshot:
                self.session_gps_driver_boot_time = health_snapshot.get(
                    'driver_boot_time', self.latest_gps_driver_boot_time)
            self.update_session_config({
                'gnss_raw_file': self.gnss_raw_file,
                'gnss_precapture_used': precapture_used,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
            })
            self.update_gnss_session({
                'raw_file_path': self.gnss_raw_file,
                'precapture_start_time': precapture_start_time,
                'collection_start_time': collection_start_time,
                'precapture_used': precapture_used,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
                'driver_boot_time': self.session_gps_driver_boot_time,
                'driver_restarts': self.gnss_driver_restarts,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'status': 'recording' if gnss_ok else 'recording_with_gnss_error',
            })
            if health_snapshot:
                self.update_gnss_session(self.extract_gnss_health_fields(health_snapshot))
            if preserved_precapture:
                self.get_logger().warn(
                    f'  Preserved temp precapture for manual inspection: {precapture_temp_path}')
        else:
            self.get_logger().info('Step 1/4: Indoor mode - GNSS disabled for this session')
            self.gnss_raw_file = ''
            self.reset_gnss_precapture_state()

        # --- Step 2: rosbag (B key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 2/4: Starting rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_start_client, Trigger.Request(), '/rosbag/start')
        if not success:
            errors.append(f'Rosbag start: {msg}')
            self.get_logger().error(f'  Rosbag start failed: {msg}')
        else:
            self.get_logger().info(f'  Rosbag recording started: {msg}')

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
        else:
            self.get_logger().info(f'  GPR scan started: {msg}')

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
        else:
            self.get_logger().info(f'  Video recording started: {msg}')
        
        self.collection_started = True
        self.collection_active = True
        self.is_paused = False
        
        self.get_logger().info('')
        if errors:
            response.success = False
            response.message = f'Started with errors: {"; ".join(errors)}'
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
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('PAUSING DATA COLLECTION')
        self.get_logger().info('='*60)
        
        if self.is_paused:
            response.success = True
            response.message = 'Already paused'
            return response
        
        errors = []
        
        self.get_logger().info('Pausing unified_data_collector...')
        success, msg = self.call_service_sync(
            self.udc_pause_client, Trigger.Request(), '/udc/pause')
        if not success:
            errors.append(f'UDC pause: {msg}')
        else:
            self.get_logger().info('  unified_data_collector paused')
        
        self.get_logger().info('Pausing gpr_scan_controller...')
        success, msg = self.call_service_sync(
            self.gpr_pause_client, Trigger.Request(), '/gpr_scan/pause')
        if not success:
            errors.append(f'GPR pause: {msg}')
        else:
            self.get_logger().info('  gpr_scan_controller paused')
        
        self.get_logger().info('Pausing rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_pause_client, Trigger.Request(), '/rosbag/pause')
        if not success:
            errors.append(f'Rosbag pause: {msg}')
        else:
            self.get_logger().info('  rosbag recording paused')
        
        self.is_paused = True
        
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
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('RESUMING DATA COLLECTION')
        self.get_logger().info('='*60)
        
        if not self.is_paused:
            response.success = True
            response.message = 'Not paused'
            return response
        
        errors = []
        
        self.get_logger().info('Resuming unified_data_collector...')
        success, msg = self.call_service_sync(
            self.udc_resume_client, Trigger.Request(), '/udc/resume')
        if not success:
            errors.append(f'UDC resume: {msg}')
        else:
            self.get_logger().info('  unified_data_collector resumed')
        
        self.get_logger().info('Resuming gpr_scan_controller...')
        success, msg = self.call_service_sync(
            self.gpr_resume_client, Trigger.Request(), '/gpr_scan/resume')
        if not success:
            errors.append(f'GPR resume: {msg}')
        else:
            self.get_logger().info('  gpr_scan_controller resumed')
        
        self.get_logger().info('Resuming rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_resume_client, Trigger.Request(), '/rosbag/resume')
        if not success:
            errors.append(f'Rosbag resume: {msg}')
        else:
            self.get_logger().info('  rosbag recording resumed')
        
        self.is_paused = False
        
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
        req = SetBool.Request()
        req.data = False
        success, msg = self.call_service_sync(
            self.video_record_client, req, '/video_record_set')
        if not success:
            errors.append(f'Video stop: {msg}')
            self.get_logger().error(f'  Video stop failed: {msg}')
        else:
            self.get_logger().info('  Video recording + CSV logging stopped')
        
        # Step 2: Stop GPR scan (G key toggle equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 2/4: Stopping GPR scan (motor + logging + actuator)...')
        success, msg = self.call_service_sync(
            self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        if not success:
            errors.append(f'GPR stop: {msg}')
            self.get_logger().error(f'  GPR stop failed: {msg}')
        else:
            self.get_logger().info('  GPR scan stopped (actuator lowered)')
        
        # Step 3: Stop rosbag recording (B key toggle equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 3/4: Stopping rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        if not success:
            errors.append(f'Rosbag stop: {msg}')
            self.get_logger().error(f'  Rosbag stop failed: {msg}')
        else:
            self.get_logger().info('  Rosbag recording stopped')
        
        # Step 4: Stop GNSS raw logging after a short tail window
        self.get_logger().info('')
        if self.gnss_enabled:
            if self.gnss_raw_file:
                self.get_logger().info(
                    f'Step 4/5: Keeping GNSS raw logging active for {self.gnss_post_stop_delay_sec:.1f}s...')
                time.sleep(self.gnss_post_stop_delay_sec)
                success, msg = self.stop_gps_raw_log()
                if not success:
                    errors.append(f'GNSS stop: {msg}')
                    self.get_logger().error(f'  GNSS raw log stop failed: {msg}')
                else:
                    self.get_logger().info(f'  GNSS raw logging stopped: {msg}')
            else:
                self.get_logger().info('Step 4/5: GNSS raw logging was not active for this session')

            collection_end_time = datetime.now().isoformat()
            gnss_bytes_written = 0
            if self.gnss_raw_file and os.path.exists(self.gnss_raw_file):
                gnss_bytes_written = os.path.getsize(self.gnss_raw_file)
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
                'gnss_raw_file': self.gnss_raw_file,
                'gnss_bytes_written': gnss_bytes_written,
                'gnss_invalid': self.gnss_invalid,
                'gnss_invalid_reasons': self.gnss_invalid_reasons,
                'gnss_warning_reasons': self.gnss_warning_reasons,
                'driver_restarts': self.gnss_driver_restarts,
                'preserved_precapture_paths': self.preserved_gnss_precapture_paths,
            })
            self.update_gnss_session({
                'raw_file_path': self.gnss_raw_file,
                'collection_end_time': collection_end_time,
                'bytes_written': gnss_bytes_written,
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
        
        # Reset state — ready for next /dc/start
        self.collection_started = False
        self.is_paused = False
        self.reset_gnss_session_state()
        
        self.get_logger().info('')
        if errors:
            response.success = False
            response.message = f'Ended with errors: {"; ".join(errors)}'
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
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('ENDING DATA COLLECTION — DELETING ALL DATA')
        self.get_logger().info('='*60)
        
        # Step 1: Stop video recording + CSV logging
        self.get_logger().info('')
        self.get_logger().info('Step 1/5: Stopping video recording + CSV logging...')
        req = SetBool.Request()
        req.data = False
        self.call_service_sync(self.video_record_client, req, '/video_record_set')
        self.get_logger().info('  Video recording + CSV logging stopped')
        
        # Step 2: Stop GPR scan controller
        self.get_logger().info('')
        self.get_logger().info('Step 2/5: Stopping GPR scan (motor + logging + actuator)...')
        self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        self.get_logger().info('  gpr_scan_controller stopped (actuator lowered)')
        
        # Step 3: Stop rosbag recording
        self.get_logger().info('')
        self.get_logger().info('Step 3/5: Stopping rosbag recording...')
        self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        self.get_logger().info('  rosbag recording stopped')
        
        # Step 4: Stop GNSS raw logging and delete temporary capture, if any
        self.get_logger().info('')
        deleted_precapture_artifacts = False
        if self.gnss_enabled:
            self.get_logger().info('Step 4/5: Stopping GNSS raw logging...')
            self.stop_gps_raw_log()
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
        
        # Reset state
        self.collection_started = False
        self.is_paused = False
        self.section_folder = ''
        self.reset_gnss_session_state()
        
        self.get_logger().info('')
        if response.success:
            self.get_logger().info('='*60)
            self.get_logger().info('DATA COLLECTION ENDED AND DELETED')
            self.get_logger().info('='*60)
            self.get_logger().info('Ready for new section (call /dc/start)')
        
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
