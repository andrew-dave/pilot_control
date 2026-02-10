#!/usr/bin/env python3
"""
Data Collection Coordinator Node
================================
Central coordinator for data collection. Section folders are created dynamically
when /dc/start is called (using current time), not at launch. Other nodes
(unified_data_collector, gpr_scan_controller, raw_map_saver) get their session
directory from this node via set_parameters + set_directory.

Services:
- /dc/start: Create new section folder (current time), update all nodes, then start (B->G->R).
- /dc/pause, /dc/resume: Pause/resume all collection.
- /dc/end_and_save: End and save with partial/complete tag; rename section folder.
- /dc/end_and_delete: End and delete section data.

Linear actuator control is handled internally by gpr_scan_controller.
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

# Re-use section numbering from setup_data_folders
try:
    from setup_data_folders import find_next_section_number
except ImportError:
    def find_next_section_number(day_folder_path):
        if not Path(day_folder_path).exists():
            return 1
        day_path = Path(day_folder_path)
        section_folders = [d for d in day_path.iterdir()
                          if d.is_dir() and d.name.startswith("Section_")]
        if not section_folders:
            return 1
        section_numbers = []
        for folder in section_folders:
            try:
                parts = folder.name.split("_")
                if len(parts) >= 2:
                    section_numbers.append(int(parts[1]))
            except (ValueError, IndexError):
                continue
        return max(section_numbers) + 1 if section_numbers else 1


class DataCollectionCoordinator(Node):
    def __init__(self):
        super().__init__('data_collection_coordinator')
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('section_folder', '')
        self.declare_parameter('day_folder', '')
        self.declare_parameter('start_sequence_delay', 1.0)

        self.section_folder = self.get_parameter('section_folder').value
        self.day_folder = self.get_parameter('day_folder').value
        self.start_delay = self.get_parameter('start_sequence_delay').value

        self.is_paused = False
        self.collection_active = True
        self.collection_started = False

        # Service servers
        self.create_service(Trigger, '/dc/start', self.start_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/dc/pause', self.pause_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/dc/resume', self.resume_callback, callback_group=self.callback_group)
        self.create_service(SetBool, '/dc/end_and_save', self.end_and_save_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/dc/end_and_delete', self.end_and_delete_callback, callback_group=self.callback_group)

        # Service clients
        self.video_record_client = self.create_client(SetBool, '/video_record_set', callback_group=self.callback_group)
        self.udc_pause_client = self.create_client(Trigger, '/udc/pause', callback_group=self.callback_group)
        self.udc_resume_client = self.create_client(Trigger, '/udc/resume', callback_group=self.callback_group)
        self.gpr_scan_start_client = self.create_client(Trigger, '/gpr_scan/start', callback_group=self.callback_group)
        self.gpr_pause_client = self.create_client(Trigger, '/gpr_scan/pause', callback_group=self.callback_group)
        self.gpr_resume_client = self.create_client(Trigger, '/gpr_scan/resume', callback_group=self.callback_group)
        self.gpr_stop_client = self.create_client(Trigger, '/gpr_scan/stop', callback_group=self.callback_group)
        self.rosbag_start_client = self.create_client(Trigger, '/rosbag/start', callback_group=self.callback_group)
        self.rosbag_pause_client = self.create_client(Trigger, '/rosbag/pause', callback_group=self.callback_group)
        self.rosbag_resume_client = self.create_client(Trigger, '/rosbag/resume', callback_group=self.callback_group)
        self.rosbag_stop_client = self.create_client(Trigger, '/rosbag/stop', callback_group=self.callback_group)
        self.save_map_client = self.create_client(Trigger, '/save_raw_map', callback_group=self.callback_group)
        self.gpr_set_dir_client = self.create_client(Trigger, '/gpr_scan/set_directory', callback_group=self.callback_group)
        self.udc_set_dir_client = self.create_client(Trigger, '/udc/set_directory', callback_group=self.callback_group)
        self.map_set_dir_client = self.create_client(Trigger, '/raw_map_saver/set_directory', callback_group=self.callback_group)
        self.gpr_set_params_client = self.create_client(SetParameters, '/gpr_scan_controller/set_parameters', callback_group=self.callback_group)
        self.udc_set_params_client = self.create_client(SetParameters, '/unified_data_collector/set_parameters', callback_group=self.callback_group)
        self.map_set_params_client = self.create_client(SetParameters, '/raw_map_saver/set_parameters', callback_group=self.callback_group)

        self.get_logger().info('='*60)
        self.get_logger().info('DATA COLLECTION COORDINATOR STARTED')
        self.get_logger().info('Section folders are created on /dc/start (current time)')
        self.get_logger().info(f'Day folder: {self.day_folder}')
        self.get_logger().info('  /dc/start          - Create section + start all collection')
        self.get_logger().info('  /dc/pause, /dc/resume, /dc/end_and_save, /dc/end_and_delete')
        self.get_logger().info('='*60)

    def call_service_sync(self, client, request, service_name, timeout=5.0):
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
            return False, f'Service {service_name} returned None'
        except Exception as e:
            self.get_logger().error(f'Service {service_name} exception: {e}')
            return False, f'Service {service_name} exception: {e}'

    def _set_param_and_directory(self, set_params_client, set_dir_client, param_name, value, node_label, set_dir_srv_name):
        if not set_params_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'  {node_label}: set_parameters not available')
            return False
        param = Parameter()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = value
        req = SetParameters.Request()
        req.parameters = [param]
        future = set_params_client.call_async(req)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 3.0:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(f'  {node_label}: parameter update timed out')
            return False
        success, _ = self.call_service_sync(set_dir_client, Trigger.Request(), set_dir_srv_name)
        if success:
            self.get_logger().info(f'  ✓ {node_label} directory updated')
        return success

    def create_new_section(self):
        """Create a new section folder with current time and update all data collection nodes."""
        if not self.day_folder or not os.path.exists(self.day_folder):
            self.get_logger().error('Day folder not set or missing - cannot create section')
            return False
        try:
            day_path = Path(self.day_folder)
            section_num = find_next_section_number(day_path)
            timestamp = datetime.now().strftime("%H%M%S")
            section_folder_name = f"Section_{section_num}_{timestamp}"
            section_folder = day_path / section_folder_name
            section_folder.mkdir(parents=True, exist_ok=True)
            visual_data_folder = section_folder / "Visual_data"
            gpr_scan_folder = section_folder / "GPR_scan_data"
            visual_data_folder.mkdir(exist_ok=True)
            gpr_scan_folder.mkdir(exist_ok=True)

            self.section_folder = str(section_folder)
            self.get_logger().info('')
            self.get_logger().info(f'✓ Created section: {section_folder_name} (current time)')
            self.get_logger().info(f'  Visual: {visual_data_folder}')
            self.get_logger().info(f'  GPR:    {gpr_scan_folder}')

            config_file = section_folder / 'session_config.json'
            config = {
                'day_folder': self.day_folder,
                'section_folder': self.section_folder,
                'visual_data_folder': str(visual_data_folder),
                'gpr_scan_folder': str(gpr_scan_folder),
                'section_number': section_num,
                'section_name': section_folder_name,
            }
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)

            self.get_logger().info('Updating node directories...')
            self._set_param_and_directory(
                self.gpr_set_params_client, self.gpr_set_dir_client,
                'gpr_scan_data_directory', str(gpr_scan_folder), 'GPR scan', '/gpr_scan/set_directory')
            self._set_param_and_directory(
                self.udc_set_params_client, self.udc_set_dir_client,
                'visual_data_directory', str(visual_data_folder), 'Unified data collector', '/udc/set_directory')
            self._set_param_and_directory(
                self.map_set_params_client, self.map_set_dir_client,
                'save_directory', self.section_folder, 'Raw map saver', '/raw_map_saver/set_directory')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to create section: {e}')
            return False

    def start_callback(self, request, response):
        """Create a new section (current time), update nodes, then start B->G->R."""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING DATA COLLECTION SEQUENCE')
        self.get_logger().info('='*60)
        if self.collection_started:
            response.success = False
            response.message = 'Data collection already started'
            return response

        # Always create a new section folder (current time) and push to nodes
        self.get_logger().info('Creating new section folder (current time)...')
        if not self.create_new_section():
            response.success = False
            response.message = 'Failed to create section folder'
            return response

        errors = []
        delay = self.start_delay

        self.get_logger().info('')
        self.get_logger().info('Step 1/3: Starting rosbag recording...')
        success, msg = self.call_service_sync(self.rosbag_start_client, Trigger.Request(), '/rosbag/start')
        if not success:
            errors.append(f'Rosbag start: {msg}')
        else:
            self.get_logger().info(f'  ✓ Rosbag started: {msg}')
        time.sleep(delay)

        self.get_logger().info('')
        self.get_logger().info('Step 2/3: Starting GPR scan (actuator + motor + logging)...')
        success, msg = self.call_service_sync(self.gpr_scan_start_client, Trigger.Request(), '/gpr_scan/start')
        if not success:
            errors.append(f'GPR scan start: {msg}')
        else:
            self.get_logger().info(f'  ✓ GPR scan started')
        time.sleep(delay)

        self.get_logger().info('')
        self.get_logger().info('Step 3/3: Starting video recording (RGB + thermal)...')
        req = SetBool.Request()
        req.data = True
        success, msg = self.call_service_sync(self.video_record_client, req, '/video_record_set')
        if not success:
            errors.append(f'Video start: {msg}')
        else:
            self.get_logger().info(f'  ✓ Video recording started')

        self.collection_started = True
        self.collection_active = True
        self.is_paused = False
        response.success = len(errors) == 0
        response.message = 'All started' if not errors else f'Started with errors: {"; ".join(errors)}'
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('✓ DATA COLLECTION STARTED' if not errors else f'Started with errors: {response.message}')
        self.get_logger().info('='*60)
        return response

    def pause_callback(self, request, response):
        errors = []
        for label, client, srv in [
            ('UDC', self.udc_pause_client, '/udc/pause'),
            ('GPR', self.gpr_pause_client, '/gpr_scan/pause'),
            ('Rosbag', self.rosbag_pause_client, '/rosbag/pause'),
        ]:
            success, _ = self.call_service_sync(client, Trigger.Request(), srv)
            if not success:
                errors.append(label)
        self.is_paused = True
        response.success = len(errors) == 0
        response.message = 'Paused with errors: ' + ', '.join(errors) if errors else 'All paused'
        return response

    def resume_callback(self, request, response):
        errors = []
        for label, client, srv in [
            ('UDC', self.udc_resume_client, '/udc/resume'),
            ('GPR', self.gpr_resume_client, '/gpr_scan/resume'),
            ('Rosbag', self.rosbag_resume_client, '/rosbag/resume'),
        ]:
            success, _ = self.call_service_sync(client, Trigger.Request(), srv)
            if not success:
                errors.append(label)
        self.is_paused = False
        response.success = len(errors) == 0
        response.message = 'Resumed with errors: ' + ', '.join(errors) if errors else 'All resumed'
        return response

    def end_and_save_callback(self, request, response):
        tag = 'complete' if request.data else 'partial'
        self.get_logger().info('')
        self.get_logger().info(f'ENDING DATA COLLECTION - TAG: {tag.upper()}')

        errors = []
        req_false = SetBool.Request()
        req_false.data = False

        self.get_logger().info('Step 1/4: Stopping video + CSV...')
        success, msg = self.call_service_sync(self.video_record_client, req_false, '/video_record_set')
        if not success:
            errors.append(f'Video: {msg}')
        self.get_logger().info('Step 2/4: Stopping GPR scan (actuator lowered)...')
        success, msg = self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        if not success:
            errors.append(f'GPR: {msg}')
        self.get_logger().info('Step 3/4: Stopping rosbag...')
        success, msg = self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        if not success:
            errors.append(f'Rosbag: {msg}')
        self.get_logger().info('Step 4/4: Saving map...')
        success, msg = self.call_service_sync(self.save_map_client, Trigger.Request(), '/save_raw_map', timeout=15.0)
        if not success:
            errors.append(f'Map: {msg}')

        new_path = self.rename_section_folder(tag)
        if new_path:
            self.get_logger().info(f'  ✓ Section renamed to: {os.path.basename(new_path)}')
        elif self.section_folder:
            errors.append('Rename section folder')

        self.collection_started = False
        self.is_paused = False
        response.success = len(errors) == 0
        response.message = f'Ended and saved as {tag}' if not errors else '; '.join(errors)
        self.get_logger().info('✓ Ready for new section (call /dc/start)')
        return response

    def end_and_delete_callback(self, request, response):
        self.get_logger().info('ENDING DATA COLLECTION - DELETING')
        req_false = SetBool.Request()
        req_false.data = False
        self.call_service_sync(self.video_record_client, req_false, '/video_record_set')
        self.call_service_sync(self.gpr_stop_client, Trigger.Request(), '/gpr_scan/stop')
        self.call_service_sync(self.rosbag_stop_client, Trigger.Request(), '/rosbag/stop')
        if self.section_folder and os.path.exists(self.section_folder):
            try:
                shutil.rmtree(self.section_folder)
                self.get_logger().info(f'  ✓ Deleted: {self.section_folder}')
                response.success = True
                response.message = f'Deleted {self.section_folder}'
            except Exception as e:
                self.get_logger().error(f'  ✗ Delete failed: {e}')
                response.success = False
                response.message = str(e)
        else:
            response.success = True
            response.message = 'No section folder to delete'
        self.section_folder = ''
        self.collection_started = False
        self.is_paused = False
        self.get_logger().info('✓ Ready for new section (call /dc/start)')
        return response

    def rename_section_folder(self, tag):
        if not self.section_folder or not os.path.exists(self.section_folder):
            return None
        folder_path = Path(self.section_folder)
        parent = folder_path.parent
        current_name = folder_path.name
        if current_name.endswith('_partial') or current_name.endswith('_complete'):
            return str(folder_path)
        new_name = f'{current_name}_{tag}'
        new_path = parent / new_name
        try:
            folder_path.rename(new_path)
            self.section_folder = str(new_path)
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
            self.get_logger().error(f'Rename failed: {e}')
            return None


def main():
    rclpy.init()
    node = DataCollectionCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
