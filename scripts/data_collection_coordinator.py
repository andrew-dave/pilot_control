#!/usr/bin/env python3
"""
Data Collection Coordinator Node
================================
Central coordinator for managing data collection across multiple nodes.

Provides unified services for:
- start_dc: Start all data collection (creates new section, then B -> G -> R)
- pause_dc: Pause all data collection (visual, GPR, rosbag)
- resume_dc: Resume all data collection
- end_and_save_dc: End collection and save with partial/complete tag
- end_and_delete_dc: End collection and delete all session data

This node coordinates:
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
        self.declare_parameter('start_sequence_delay', 1.0)  # Delay between start steps
        
        # Get parameters
        self.base_data_dir = self.get_parameter('base_data_directory').value
        self.start_delay = self.get_parameter('start_sequence_delay').value
        
        # Dynamic state — set on each /dc/start
        self.day_folder = ''
        self.section_folder = ''
        
        # State tracking
        self.is_paused = False
        self.collection_active = True   # Assume active — stop services should always work
        self.collection_started = False
        
        # ==================== Service Servers ====================
        self.start_srv = self.create_service(
            Trigger, '/dc/start', self.start_callback,
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
        
        self.get_logger().info('='*60)
        self.get_logger().info('DATA COLLECTION COORDINATOR STARTED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Base data directory: {self.base_data_dir}')
        self.get_logger().info('')
        self.get_logger().info('Available services:')
        self.get_logger().info('  /dc/start          - Start all data collection (B->G->R)')
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
        """Set a string parameter on a remote node. Returns True on success."""
        if not param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'  {node_label}: parameter service not available')
            return False
        
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
            self.get_logger().warn(f'  {node_label}: parameter update timed out')
            return False
        return True

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
            gpr_scan_folder   = section_path / "GPR_scan_data"
            visual_data_folder.mkdir(exist_ok=True)
            gpr_scan_folder.mkdir(exist_ok=True)

            self.section_folder = str(section_path)

            self.get_logger().info('')
            self.get_logger().info(f'Created new section: {section_name}')
            self.get_logger().info(f'  Path:        {section_path}')
            self.get_logger().info(f'  Visual data: {visual_data_folder}')
            self.get_logger().info(f'  GPR data:    {gpr_scan_folder}')

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
            }
            with open(section_path / 'session_config.json', 'w') as f:
                json.dump(config, f, indent=2)

            # ---- Push new paths to downstream nodes ----

            # GPR scan controller
            self.get_logger().info('Updating gpr_scan_controller directory...')
            if self.set_node_param(self.gpr_set_params_client,
                                   'gpr_scan_data_directory',
                                   str(gpr_scan_folder),
                                   'gpr_scan_controller'):
                self.call_service_sync(
                    self.gpr_set_dir_client, Trigger.Request(), '/gpr_scan/set_directory')
                self.get_logger().info(f'  GPR directory  -> {gpr_scan_folder}')
            else:
                self.get_logger().warn('  GPR directory update FAILED')

            # Unified data collector
            self.get_logger().info('Updating unified_data_collector directory...')
            if self.set_node_param(self.udc_set_params_client,
                                   'visual_data_directory',
                                   str(visual_data_folder),
                                   'unified_data_collector'):
                self.call_service_sync(
                    self.udc_set_dir_client, Trigger.Request(), '/udc/set_directory')
                self.get_logger().info(f'  UDC directory  -> {visual_data_folder}')
            else:
                self.get_logger().warn('  UDC directory update FAILED')

            # Raw map saver
            self.get_logger().info('Updating raw_map_saver directory...')
            if self.set_node_param(self.map_set_params_client,
                                   'save_directory',
                                   str(section_path),
                                   'raw_map_saver'):
                self.call_service_sync(
                    self.map_set_dir_client, Trigger.Request(), '/raw_map_saver/set_directory')
                self.get_logger().info(f'  Map directory  -> {section_path}')
            else:
                self.get_logger().warn('  Map directory update FAILED')

            return True

        except Exception as e:
            self.get_logger().error(f'Failed to create new section: {e}')
            return False

    # ================================================================
    #  /dc/start
    # ================================================================

    def start_callback(self, request, response):
        """
        Start all data collection in sequence:
        0. Create a new section folder (with the current timestamp)
        1. B - Start rosbag recording
        2. G - Start GPR scan (includes linear actuator UP + GPR motor + logging)
        3. R - Start video recording (RGB + thermal)
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
        
        # --- Step 1: rosbag (B key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 1/3: Starting rosbag recording...')
        success, msg = self.call_service_sync(
            self.rosbag_start_client, Trigger.Request(), '/rosbag/start')
        if not success:
            errors.append(f'Rosbag start: {msg}')
            self.get_logger().error(f'  Rosbag start failed: {msg}')
        else:
            self.get_logger().info(f'  Rosbag recording started: {msg}')
        
        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)
        
        # --- Step 2: GPR scan (G key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 2/3: Starting GPR scan (actuator + motor + logging)...')
        success, msg = self.call_service_sync(
            self.gpr_scan_start_client, Trigger.Request(), '/gpr_scan/start')
        if not success:
            errors.append(f'GPR scan start: {msg}')
            self.get_logger().error(f'  GPR scan start failed: {msg}')
        else:
            self.get_logger().info(f'  GPR scan started: {msg}')
        
        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)
        
        # --- Step 3: video (R key) ---
        self.get_logger().info('')
        self.get_logger().info('Step 3/3: Starting video recording (RGB + thermal)...')
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
            response.message = f'All data collection started — {os.path.basename(self.section_folder)}'
            self.get_logger().info('='*60)
            self.get_logger().info('DATA COLLECTION STARTED')
            self.get_logger().info(f'  Section:  {os.path.basename(self.section_folder)}')
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
        
        # Step 4: Save map (M key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 4/4: Saving point cloud map...')
        success, msg = self.call_service_sync(
            self.save_map_client, Trigger.Request(), '/save_raw_map', timeout=15.0)
        if not success:
            errors.append(f'Map save: {msg}')
            self.get_logger().error(f'  Map save failed: {msg}')
        else:
            self.get_logger().info(f'  Point cloud map saved: {msg}')
        
        # Step 5: Rename section folder with tag
        self.get_logger().info('')
        self.get_logger().info('Renaming section folder...')
        new_folder_path = self._rename_section_folder(tag)
        if new_folder_path:
            self.get_logger().info(f'  Section folder renamed to: {os.path.basename(new_folder_path)}')
        else:
            errors.append('Failed to rename section folder')
            self.get_logger().error('  Failed to rename section folder')
        
        # Reset state — ready for next /dc/start
        self.collection_started = False
        self.is_paused = False
        
        self.get_logger().info('')
        if errors:
            response.success = False
            response.message = f'Ended with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = f'Data collection ended and saved as {tag}'
            self.get_logger().info('='*60)
            self.get_logger().info(f'DATA COLLECTION ENDED AND SAVED AS {tag.upper()}')
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
        self.get_logger().info('Step 1/4: Stopping video recording + CSV logging...')
        req = SetBool.Request()
        req.data = False
        self.call_service_sync(self.video_record_client, req, '/video_record_set')
        self.get_logger().info('  Video recording + CSV logging stopped')
        
        # Step 2: Stop GPR scan controller
        self.get_logger().info('')
        self.get_logger().info('Step 2/4: Stopping GPR scan (motor + logging + actuator)...')
        self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        self.get_logger().info('  gpr_scan_controller stopped (actuator lowered)')
        
        # Step 3: Stop rosbag recording
        self.get_logger().info('')
        self.get_logger().info('Step 3/4: Stopping rosbag recording...')
        self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        self.get_logger().info('  rosbag recording stopped')
        
        # Step 4: Delete the entire section folder
        self.get_logger().info('')
        self.get_logger().info('Step 4/4: Deleting session data...')
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
            self.get_logger().warn(f'  Section folder not found: {self.section_folder}')
            response.success = False
            response.message = f'Section folder not found: {self.section_folder}'
        
        # Reset state
        self.collection_started = False
        self.is_paused = False
        self.section_folder = ''
        
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
        if current_name.endswith('_partial') or current_name.endswith('_complete'):
            self.get_logger().info(f'Folder already tagged: {current_name}')
            return str(folder_path)
        
        new_name = f'{current_name}_{tag}'
        new_path = folder_path.parent / new_name
        
        try:
            folder_path.rename(new_path)
            self.section_folder = str(new_path)
            
            # Update session_config.json
            config_file = new_path / 'session_config.json'
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = json.load(f)
                config['section_folder'] = str(new_path)
                config['section_name'] = new_name
                config['completion_tag'] = tag
                with open(config_file, 'w') as f:
                    json.dump(config, f, indent=2)
            
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
