from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Import the folder setup script
# Get the package installation directory and add scripts to path
package_dir = Path(get_package_share_directory('pilot_control'))
# Try source directory first (for development)
source_script_dir = Path(__file__).parent.parent / 'scripts'
if source_script_dir.exists():
    sys.path.insert(0, str(source_script_dir))
else:
    # Fall back to installed location
    # Scripts are installed to lib/pilot_control, navigate there
    install_script_dir = package_dir.parent.parent / 'lib' / 'pilot_control'
    sys.path.insert(0, str(install_script_dir))

from setup_data_folders import setup_data_folders

def generate_launch_description():
    # Declare launch arguments
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.072',
        description='Radius of the wheels in meters.'
    )
    declare_wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.32',
        description='Distance between the two wheels in meters.'
    )
    declare_can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='The CAN interface name.'
    )
    declare_can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='250000',
        description='CAN interface bitrate.'
    )
    declare_velocity_multiplier_arg = DeclareLaunchArgument(
        'velocity_multiplier',
        default_value='1.0',
        description='A multiplier to tune the robot speed.'
    )
    declare_turn_speed_multiplier_arg = DeclareLaunchArgument(
        'turn_speed_multiplier',
        default_value='1.0',
        description='A multiplier to tune the robot turning speed.'
    )
    
    declare_base_data_directory_arg = DeclareLaunchArgument(
        'base_data_directory',
        default_value='/R_DATA',
        description='Base directory for all robot data collection.'
    )

    # Setup folder structure for this session
    base_data_dir = '/R_DATA'
    folder_paths = setup_data_folders(base_data_dir)
    
    print("\n" + "="*70)
    print("DATA COLLECTION SESSION SETUP")
    print("="*70)
    print(f"Day Folder:     {folder_paths['day_name']}")
    print(f"Section:        {folder_paths['section_name']}")
    print(f"Visual Data:    {folder_paths['visual_data_folder']}")
    print(f"GPR Scan Data:  {folder_paths['gpr_scan_folder']}")
    print("="*70 + "\n")
    print("Raj is a baaaaad boy")
    # Unified Data Collector (Thermal + Dual Cameras + Odometry sync)
    unified_data_collector_node = Node(
        package='pilot_control',
        executable='unified_data_collector',
        name='unified_data_collector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            # Odometry and thermal camera settings (using tilt-corrected odometry)
            'fastlio_odom_topic': '/Odometry_tilt_corrected_diff',
            'log_directory': '/R_DATA/unified_scans',  # Fallback
            'visual_data_directory': folder_paths['visual_data_folder'],  # Session-specific
            'use_seekvision_mode': True,
            'save_color_png': True,
            'csv_flush_every_rows': 50,
            'log_frequency_hz': 10.0,
            
            # Camera device paths (See3CAM)
            'left_device': '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0',
            'right_device': '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0',
            
            # Streaming settings
            'stream_host': '192.168.168.101', # TODO: change to 172.16.14.195 (R) / 172.16.15.61 (A)
            'stream_port': 5600, # TODO: change to 5602 (R) / 5600 (A)
            'stream_bitrate_kbps': 800,
            'rtp_mtu': 1200,
            
            # Video capture settings
            'use_mjpeg_pipeline': True,
            'cap_w': 1920,
            'cap_h': 1080,
            'cap_fps': 30,
            'raw_format': 'UYVY',
            'raw_w': 1280,
            'raw_h': 720,
            'raw_fps': 60,
        }]
    )

    # CAN interface setup command
    can_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', LaunchConfiguration('can_interface'), 
             'up', 'type', 'can', 'bitrate', LaunchConfiguration('can_bitrate')],
        output='screen'
    )

    # ODrive CAN nodes for motor control
    left_odrive_node = Node(
        package='odrive_can',
        executable='odrive_can_node',
        namespace='left',
        name='odrive_can_left',
        parameters=[{
            'node_id': 0,
            'interface': LaunchConfiguration('can_interface')
        }],
        output='screen'
    )

    right_odrive_node = Node(
        package='odrive_can',
        executable='odrive_can_node',
        namespace='right',
        name='odrive_can_right',
        parameters=[{
            'node_id': 1,
            'interface': LaunchConfiguration('can_interface')
        }],
        output='screen'
    )

    # ODrive CAN node for GPR/micro (node_id 2)
    gpr_odrive_node = Node(
        package='odrive_can',
        executable='odrive_can_node',
        namespace='gpr',
        name='odrive_can_gpr',
        parameters=[{
            'node_id': 2,
            'interface': LaunchConfiguration('can_interface')
        }],
        output='screen'
    )

    # Differential Drive Controller
    # NOTE: GPR (third motor) control is DISABLED here - handled by gpr_scan_controller instead
    diff_drive_controller = Node(
        package='pilot_control',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'can_interface': LaunchConfiguration('can_interface'),
            'velocity_multiplier': LaunchConfiguration('velocity_multiplier'),
            'turn_speed_multiplier': LaunchConfiguration('turn_speed_multiplier'),
            # GPR CONTROL DISABLED - controlled by gpr_scan_controller instead
            # 'invert_third': True  # COMMENTED OUT - no GPR control from diff_drive_controller
        }]
    )

    # Foxglove Bridge - Maximum buffer settings
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'max_q_size': 5000,  # Maximum buffer size
            'num_threads': 16,    # Maximum thread count
            'send_buffer_limit': 100000000,  # Increase send buffer to 100MB
            'capabilities': ['clientPublish', 'connectionGraph', 'parameters', 'parametersSubscribe', 'services', 'subscribe', 'advertise', 'unadvertise', 'getParameters', 'setParameters', 'subscribeParameterUpdates', 'messageDefinitions', 'fetchAsset', 'unsubscribe', 'subscribeConnectionGraph']
        }]
    )

    # Livox ROS Driver (MID360) - Reduced frequency
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        prefix='taskset -c 5',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,  # Reduced from 10Hz to 3Hz to reduce data load
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/robot/livox_test.lvx',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'
            ])
        }]
    )

    # Fast-LIO2 Node (exact copy from original launch file)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        prefix='taskset -c 5',
        parameters=[PathJoinSubstitution([
            FindPackageShare('pilot_control'), 'config', 'fastlio_mid360.yaml'
        ]), {
            'use_sim_time': False
        }],
        output='screen'
    )

    # Static Transform Publishers for Tilted LiDAR Setup
    # Transform from body to foot (corrects the 15-degree tilt)
    body_to_foot_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_foot_transform',
        arguments=['0.0', '0.0', '0.0', '0.0', '-0.2618', '0.0', 'body', 'foot'],
        output='screen'
    )
    
    # Transform from camera_init to foot_init (corrects the global frame for visualization)
    camera_init_to_foot_init_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_foot_init_transform',
        arguments=['0.0','0.0','0.0','0.0','-0.2618','0.0', 'camera_init', 'foot_init'],
        output='screen'
    )

    # Raw Map Saver - Saves unprocessed maps from Fast-LIO2
    raw_map_saver = Node(
        package='pilot_control',
        executable='raw_map_saver',
        name='raw_map_saver',
        output='screen',
        parameters=[{
            'input_topic': '/Laser_map',
            'save_directory': '/tmp/robot_maps'
        }]
    )

    # Shutdown Service Node (for remote shutdown)
    shutdown_service_node = Node(
        package='pilot_control',
        executable='shutdown_service',
        name='shutdown_service',
        output='screen'
    )

    # GPR Serial Bridge (Arduino connector)
    gpr_serial_bridge_node = Node(
        package='pilot_control',
        executable='gpr_serial_bridge.py',
        name='gpr_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM1',  # adjust if different on LattePanda
            'baud_rate': 9600
        }]
    )

    # GPR Scan Controller - EXCLUSIVE GPR motor control + synchronized scan + logging + rosbag recording
    # This is the ONLY node that controls the GPR motor (diff_drive_controller GPR control is disabled)
    # Also handles rosbag recording to save resources (no separate Python process needed)
    gpr_scan_controller_node = Node(
        package='pilot_control',
        executable='gpr_scan_controller.py',
        name='gpr_scan_controller',
        output='screen',
        parameters=[{
            'gpr_wheel_radius': 0.03,           # 60mm diameter virtual wheel
            'gpr_gear_ratio': 1.0,               # Direct drive
            'velocity_multiplier': LaunchConfiguration('velocity_multiplier'),
            'gpr_scan_velocity_mps': 0.5,        # 0.5 m/s scanning speed
            'invert_third': True,                # GPR motor direction inversion
            'fastlio_odom_topic': '/Odometry_tilt_corrected_diff',  # Use tilt-corrected odometry
            'log_frequency_hz': 50.0,            # 50 Hz logging
            'log_directory': '/R_DATA/gpr_scans',  # Fallback
            'gpr_scan_data_directory': folder_paths['section_folder'],  # Session-specific (for both GPR logs and rosbags)
            'rosbag_topics': [
                '/Odometry',  # Raw Fast-LIO odometry
                '/Odometry_tilt_corrected_diff',  # Tilt-corrected odometry from diff_drive_controller
                '/cmd_vel',
                '/left/controller_status',
                '/right/controller_status',
                '/gpr/controller_status',
                '/Laser_map',  # Point cloud - recorded without compression to reduce CPU load
                '/tf',
                '/tf_static',
            ]
        }]
    )

    # Laser Map Rotator - tilt correction for occupancy grid
    laser_map_rotator_node = Node(
        package='pilot_control',
        executable='laser_map_rotator',
        name='laser_map_rotator',
        output='screen',
        parameters=[{
            'input_topic': '/Laser_map',
            'output_topic': '/Laser_map_rotated',
            # 'pitch_rad': 0.2617993878,  # 15 degrees
            'output_frame': 'foot_init'
        }]
    )

    # OctoMap Server – builds 3-D occupancy map from rotated cloud
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        remappings=[('cloud_in', '/Laser_map_rotated')],
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('pilot_control'), 'config', 'octo_occu.yaml'
            ])
        ]
    )

    # Auto-start data collection after unified_data_collector is ready
    # start_recording = ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/video_record_set', 'std_srvs/srv/SetBool', '{data: true}'],
    #     output='screen'
    # )

    return LaunchDescription([
        # Launch arguments
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        declare_velocity_multiplier_arg,
        declare_turn_speed_multiplier_arg,
        declare_base_data_directory_arg,
        
        
        # CAN setup (with delay to ensure it's ready)
        TimerAction(
            period=1.0,
            actions=[can_setup]
        ),
        
        # Robot and mapping components (with delay after CAN setup)
        TimerAction(
            period=3.0,
            actions=[
                left_odrive_node, # taskset -c 5
                right_odrive_node, # taskset -c 5
                gpr_odrive_node, # taskset -c 5
                diff_drive_controller, # taskset -c 5
                livox_driver, # taskset -c 5
                fast_lio_node, # taskset -c 5
                laser_map_rotator_node, 
                body_to_foot_transform,
                camera_init_to_foot_init_transform,
                #raw_map_saver,
                #octomap_server_node,
                shutdown_service_node,
                unified_data_collector_node, # taskset -c 4,6
                
                gpr_serial_bridge_node,
                gpr_scan_controller_node, # taskset -c 4,6 (includes rosbag recording)
            ]
        ),
        
        # Auto-start data collection (with delay to ensure unified_data_collector is ready)
        # TimerAction(
        #     period=8.0,
        #     actions=[start_recording]
        # ),
    ]) 