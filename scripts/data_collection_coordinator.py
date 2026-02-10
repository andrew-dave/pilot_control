#!/usr/bin/env python3
"""
Data Collection Coordinator Node
================================
Central coordinator for managing data collection across multiple nodes.

Provides unified services for:
- start_dc: Start all data collection (B -> G -> R sequence with 1s gaps)
- pause_dc: Pause all data collection (visual, GPR, rosbag)
- resume_dc: Resume all data collection
- end_and_save_dc: End collection and save with partial/complete tag
- end_and_delete_dc: End collection and delete all session data

This node coordinates:
- unified_data_collector (thermal + cameras + odometry)
- gpr_scan_controller (GPR + rosbag)
- raw_map_saver (point cloud maps)
- gpr_serial_bridge (linear actuator control)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import os
import shutil
import json
import time
from pathlib import Path


class DataCollectionCoordinator(Node):
    def __init__(self):
        super().__init__('data_collection_coordinator')
        
        # Declare parameters
        self.declare_parameter('section_folder', '')
        self.declare_parameter('day_folder', '')
        self.declare_parameter('start_sequence_delay', 1.0)  # Delay between start steps
        
        # Get parameters
        self.section_folder = self.get_parameter('section_folder').value
        self.day_folder = self.get_parameter('day_folder').value
        self.start_delay = self.get_parameter('start_sequence_delay').value
        
        # State tracking
        self.is_paused = False
        self.collection_active = False  # Start as inactive until start_dc is called
        self.collection_started = False
        
        # ==================== Service Servers ====================
        # Main coordination services
        self.start_srv = self.create_service(
            Trigger, '/dc/start', self.start_callback)
        self.pause_srv = self.create_service(
            Trigger, '/dc/pause', self.pause_callback)
        self.resume_srv = self.create_service(
            Trigger, '/dc/resume', self.resume_callback)
        self.end_save_srv = self.create_service(
            SetBool, '/dc/end_and_save', self.end_and_save_callback)
        self.end_delete_srv = self.create_service(
            Trigger, '/dc/end_and_delete', self.end_and_delete_callback)
        
        # ==================== Service Clients ====================
        # unified_data_collector services (video/thermal recording)
        # /video_record_set (SetBool) is used for start (data=true) and stop (data=false)
        self.video_record_client = self.create_client(SetBool, '/video_record_set')
        self.udc_pause_client = self.create_client(Trigger, '/udc/pause')
        self.udc_resume_client = self.create_client(Trigger, '/udc/resume')
        self.udc_stop_client = self.create_client(Trigger, '/udc/stop')
        
        # gpr_scan_controller services
        self.gpr_scan_start_client = self.create_client(Trigger, '/gpr_scan/start')
        self.gpr_scan_toggle_client = self.create_client(Trigger, '/gpr_scan/toggle')
        self.gpr_pause_client = self.create_client(Trigger, '/gpr_scan/pause')
        self.gpr_resume_client = self.create_client(Trigger, '/gpr_scan/resume')
        self.gpr_stop_client = self.create_client(Trigger, '/gpr_scan/stop')
        
        # rosbag control (via gpr_scan_controller)
        self.rosbag_start_client = self.create_client(Trigger, '/rosbag/start')
        self.rosbag_toggle_client = self.create_client(Trigger, '/rosbag/toggle')
        self.rosbag_pause_client = self.create_client(Trigger, '/rosbag/pause')
        self.rosbag_resume_client = self.create_client(Trigger, '/rosbag/resume')
        self.rosbag_stop_client = self.create_client(Trigger, '/rosbag/stop')
        
        # Linear actuator control (Arduino via gpr_serial_bridge)
        self.line_start_client = self.create_client(Trigger, '/gpr_line_start')
        self.line_stop_client = self.create_client(Trigger, '/gpr_line_stop')
        
        # raw_map_saver service
        self.save_map_client = self.create_client(Trigger, '/save_raw_map')
        
        self.get_logger().info('='*60)
        self.get_logger().info('DATA COLLECTION COORDINATOR STARTED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Section folder: {self.section_folder}')
        self.get_logger().info('')
        self.get_logger().info('Available services:')
        self.get_logger().info('  /dc/start          - Start all data collection (B->G->R)')
        self.get_logger().info('  /dc/pause          - Pause all data collection')
        self.get_logger().info('  /dc/resume         - Resume data collection')
        self.get_logger().info('  /dc/end_and_save   - End and save (data: 0=partial, 1=complete)')
        self.get_logger().info('  /dc/end_and_delete - End and delete all session data')
        self.get_logger().info('='*60)

    def call_service_async(self, client, request, service_name, timeout=2.0):
        """Helper to call a service asynchronously with timeout"""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {service_name} not available')
            return False, f'Service {service_name} not available'
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        else:
            return False, f'Service {service_name} call failed'

    def start_callback(self, request, response):
        """
        Start all data collection in sequence:
        1. B - Start rosbag recording
        2. G - Start GPR scan (includes linear actuator UP + GPR motor + logging)
        3. R - Start video recording (RGB + thermal)
        
        Each step has a 1 second delay between them (configurable via start_sequence_delay param).
        """
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING DATA COLLECTION SEQUENCE')
        self.get_logger().info('='*60)
        
        if self.collection_started:
            response.success = False
            response.message = 'Data collection already started'
            self.get_logger().warn('Data collection already started')
            return response
        
        errors = []
        delay = self.start_delay
        
        # Step 1: Start rosbag recording (B key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 1/3: Starting rosbag recording...')
        success, msg = self.call_service_async(
            self.rosbag_start_client, Trigger.Request(), '/rosbag/start')
        if not success:
            errors.append(f'Rosbag start: {msg}')
            self.get_logger().error(f'  ✗ Rosbag start failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Rosbag recording started: {msg}')
        
        # Wait before next step
        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)
        
        # Step 2: Start GPR scan (G key equivalent)
        # This includes: linear actuator UP + GPR motor enable + GPR logging
        self.get_logger().info('')
        self.get_logger().info('Step 2/3: Starting GPR scan (actuator + motor + logging)...')
        success, msg = self.call_service_async(
            self.gpr_scan_start_client, Trigger.Request(), '/gpr_scan/start')
        if not success:
            errors.append(f'GPR scan start: {msg}')
            self.get_logger().error(f'  ✗ GPR scan start failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ GPR scan started: {msg}')
        
        # Wait before next step
        self.get_logger().info(f'  Waiting {delay}s before next step...')
        time.sleep(delay)
        
        # Step 3: Start video recording (R key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 3/3: Starting video recording (RGB + thermal)...')
        req = SetBool.Request()
        req.data = True
        success, msg = self.call_service_async(
            self.video_record_client, req, '/video_record_set')
        if not success:
            errors.append(f'Video start: {msg}')
            self.get_logger().error(f'  ✗ Video recording start failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Video recording started: {msg}')
        
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
            response.message = 'All data collection started successfully'
            self.get_logger().info('='*60)
            self.get_logger().info('✓ ALL DATA COLLECTION STARTED')
            self.get_logger().info('  - Rosbag: Recording')
            self.get_logger().info('  - GPR: Scanning (actuator UP, motor ON)')
            self.get_logger().info('  - Video: Recording (RGB + thermal)')
            self.get_logger().info('='*60)
        
        return response

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
        
        # 1. Pause unified_data_collector
        self.get_logger().info('Pausing unified_data_collector...')
        success, msg = self.call_service_async(
            self.udc_pause_client, Trigger.Request(), '/udc/pause')
        if not success:
            errors.append(f'UDC pause: {msg}')
        else:
            self.get_logger().info('  ✓ unified_data_collector paused')
        
        # 2. Pause GPR scan controller
        self.get_logger().info('Pausing gpr_scan_controller...')
        success, msg = self.call_service_async(
            self.gpr_pause_client, Trigger.Request(), '/gpr_scan/pause')
        if not success:
            errors.append(f'GPR pause: {msg}')
        else:
            self.get_logger().info('  ✓ gpr_scan_controller paused')
        
        # 3. Pause rosbag recording
        self.get_logger().info('Pausing rosbag recording...')
        success, msg = self.call_service_async(
            self.rosbag_pause_client, Trigger.Request(), '/rosbag/pause')
        if not success:
            errors.append(f'Rosbag pause: {msg}')
        else:
            self.get_logger().info('  ✓ rosbag recording paused')
        
        self.is_paused = True
        
        if errors:
            response.success = False
            response.message = f'Paused with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = 'All data collection paused'
            self.get_logger().info('✓ All data collection PAUSED')
        
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
        
        # 1. Resume unified_data_collector
        self.get_logger().info('Resuming unified_data_collector...')
        success, msg = self.call_service_async(
            self.udc_resume_client, Trigger.Request(), '/udc/resume')
        if not success:
            errors.append(f'UDC resume: {msg}')
        else:
            self.get_logger().info('  ✓ unified_data_collector resumed')
        
        # 2. Resume GPR scan controller
        self.get_logger().info('Resuming gpr_scan_controller...')
        success, msg = self.call_service_async(
            self.gpr_resume_client, Trigger.Request(), '/gpr_scan/resume')
        if not success:
            errors.append(f'GPR resume: {msg}')
        else:
            self.get_logger().info('  ✓ gpr_scan_controller resumed')
        
        # 3. Resume rosbag recording
        self.get_logger().info('Resuming rosbag recording...')
        success, msg = self.call_service_async(
            self.rosbag_resume_client, Trigger.Request(), '/rosbag/resume')
        if not success:
            errors.append(f'Rosbag resume: {msg}')
        else:
            self.get_logger().info('  ✓ rosbag recording resumed')
        
        self.is_paused = False
        
        if errors:
            response.success = False
            response.message = f'Resumed with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = 'All data collection resumed'
            self.get_logger().info('✓ All data collection RESUMED')
        
        self.get_logger().info('='*60)
        return response

    def end_and_save_callback(self, request, response):
        """
        End data collection and save with tag.
        Sequence mirrors the stop keys: T -> G -> B -> M
        
        request.data: False (0) = partial, True (1) = complete
        """
        tag = 'complete' if request.data else 'partial'
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'ENDING DATA COLLECTION - TAG: {tag.upper()}')
        self.get_logger().info('='*60)
        
        if not self.collection_active:
            response.success = False
            response.message = 'Collection already ended'
            return response
        
        errors = []
        
        # Step 1: Stop video recording (T key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 1/5: Stopping video recording...')
        req = SetBool.Request()
        req.data = False
        success, msg = self.call_service_async(
            self.video_record_client, req, '/video_record_set')
        if not success:
            errors.append(f'Video stop: {msg}')
            self.get_logger().error(f'  ✗ Video stop failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Video recording stopped')
        
        # Also stop unified_data_collector CSV logging
        self.get_logger().info('  Stopping unified_data_collector CSV logging...')
        success, msg = self.call_service_async(
            self.udc_stop_client, Trigger.Request(), '/udc/stop')
        if not success:
            errors.append(f'UDC stop: {msg}')
        else:
            self.get_logger().info('  ✓ unified_data_collector stopped')
        
        # Step 2: Stop GPR scan (G key toggle equivalent - stops motor, logging, actuator DOWN)
        self.get_logger().info('')
        self.get_logger().info('Step 2/5: Stopping GPR scan (motor + logging + actuator DOWN)...')
        success, msg = self.call_service_async(
            self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        if not success:
            errors.append(f'GPR stop: {msg}')
            self.get_logger().error(f'  ✗ GPR stop failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ GPR scan stopped')
        
        # Step 3: Lower linear actuator explicitly (K key equivalent - line DOWN)
        self.get_logger().info('')
        self.get_logger().info('Step 3/5: Lowering linear actuator...')
        success, msg = self.call_service_async(
            self.line_stop_client, Trigger.Request(), '/gpr_line_stop')
        if not success:
            errors.append(f'Actuator down: {msg}')
            self.get_logger().error(f'  ✗ Actuator down failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Linear actuator lowered')
        
        # Step 4: Stop rosbag recording (B key toggle equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 4/5: Stopping rosbag recording...')
        success, msg = self.call_service_async(
            self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        if not success:
            errors.append(f'Rosbag stop: {msg}')
            self.get_logger().error(f'  ✗ Rosbag stop failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Rosbag recording stopped')
        
        # Step 5: Save map (M key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 5/5: Saving point cloud map...')
        success, msg = self.call_service_async(
            self.save_map_client, Trigger.Request(), '/save_raw_map', timeout=10.0)
        if not success:
            errors.append(f'Map save: {msg}')
            self.get_logger().error(f'  ✗ Map save failed: {msg}')
        else:
            self.get_logger().info(f'  ✓ Point cloud map saved: {msg}')
        
        # Step 6: Rename section folder with tag
        self.get_logger().info('')
        self.get_logger().info('Renaming section folder...')
        new_folder_path = self.rename_section_folder(tag)
        if new_folder_path:
            self.get_logger().info(f'  ✓ Section folder renamed to: {os.path.basename(new_folder_path)}')
        else:
            errors.append('Failed to rename section folder')
            self.get_logger().error('  ✗ Failed to rename section folder')
        
        self.collection_active = False
        self.collection_started = False
        
        self.get_logger().info('')
        if errors:
            response.success = False
            response.message = f'Ended with errors: {"; ".join(errors)}'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = f'Data collection ended and saved as {tag}'
            self.get_logger().info('='*60)
            self.get_logger().info(f'✓ DATA COLLECTION ENDED AND SAVED AS {tag.upper()}')
            self.get_logger().info('='*60)
        
        return response

    def end_and_delete_callback(self, request, response):
        """
        End data collection and delete all session data.
        Stops all processes and removes the entire Section folder.
        """
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('ENDING DATA COLLECTION - DELETING ALL DATA')
        self.get_logger().info('='*60)
        
        if not self.collection_active:
            response.success = False
            response.message = 'Collection already ended'
            return response
        
        # Step 1: Stop video recording
        self.get_logger().info('')
        self.get_logger().info('Step 1/5: Stopping video recording...')
        req = SetBool.Request()
        req.data = False
        self.call_service_async(
            self.video_record_client, req, '/video_record_set')
        self.get_logger().info('  ✓ Video recording stopped')
        
        # Also stop unified_data_collector
        self.get_logger().info('  Stopping unified_data_collector...')
        self.call_service_async(
            self.udc_stop_client, Trigger.Request(), '/udc/stop')
        self.get_logger().info('  ✓ unified_data_collector stopped')
        
        # Step 2: Stop GPR scan controller
        self.get_logger().info('')
        self.get_logger().info('Step 2/5: Stopping GPR scan...')
        self.call_service_async(
            self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        self.get_logger().info('  ✓ gpr_scan_controller stopped')
        
        # Step 3: Lower linear actuator (K key equivalent)
        self.get_logger().info('')
        self.get_logger().info('Step 3/5: Lowering linear actuator...')
        self.call_service_async(
            self.line_stop_client, Trigger.Request(), '/gpr_line_stop')
        self.get_logger().info('  ✓ Linear actuator lowered')
        
        # Step 4: Stop rosbag recording (will be deleted with folder)
        self.get_logger().info('')
        self.get_logger().info('Step 4/5: Stopping rosbag recording...')
        self.call_service_async(
            self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        self.get_logger().info('  ✓ rosbag recording stopped')
        
        # Step 5: Delete the entire section folder
        self.get_logger().info('')
        self.get_logger().info('Step 5/5: Deleting session data...')
        if self.section_folder and os.path.exists(self.section_folder):
            try:
                self.get_logger().info(f'  Deleting: {self.section_folder}')
                shutil.rmtree(self.section_folder)
                self.get_logger().info('  ✓ Section folder deleted')
                response.success = True
                response.message = f'Data collection ended and deleted: {self.section_folder}'
            except Exception as e:
                self.get_logger().error(f'  ✗ Failed to delete folder: {e}')
                response.success = False
                response.message = f'Failed to delete folder: {e}'
        else:
            self.get_logger().warn(f'  Section folder not found: {self.section_folder}')
            response.success = False
            response.message = f'Section folder not found: {self.section_folder}'
        
        self.collection_active = False
        self.collection_started = False
        
        self.get_logger().info('')
        if response.success:
            self.get_logger().info('='*60)
            self.get_logger().info('✓ DATA COLLECTION ENDED AND DELETED')
            self.get_logger().info('='*60)
        
        return response

    def rename_section_folder(self, tag):
        """
        Rename section folder to include tag (partial/complete).
        Section_1_123456 -> Section_1_123456_complete
        """
        if not self.section_folder or not os.path.exists(self.section_folder):
            self.get_logger().error(f'Section folder not found: {self.section_folder}')
            return None
        
        folder_path = Path(self.section_folder)
        parent = folder_path.parent
        current_name = folder_path.name
        
        # Check if already tagged
        if current_name.endswith('_partial') or current_name.endswith('_complete'):
            self.get_logger().info(f'Folder already tagged: {current_name}')
            return str(folder_path)
        
        # Create new name with tag
        new_name = f'{current_name}_{tag}'
        new_path = parent / new_name
        
        try:
            folder_path.rename(new_path)
            self.section_folder = str(new_path)
            
            # Update session_config.json if it exists
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
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
