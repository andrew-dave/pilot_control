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
    
    # Odometry Tilt Corrector (Node launch for timing consistency)
    odom_tilt_corrector_node = Node(
        package='pilot_control',
        executable='odom_tilt_corrector',
        name='odom_tilt_corrector',
        output='screen',
        parameters=[{
            'odometry_topic': '/Odometry',
            'accel_topic': '/livox/imu',
            'accel_samples': 10,
            'corrected_odometry_topic': '/Odometry_tilt_corrected_diff',
            'save_directory': folder_paths['section_folder']  # Save transformation to session folder
        }]
    )
    
    print("\n" + "="*70)
    print("DATA COLLECTION SESSION SETUP")
    print("="*70)
    print(f"Day Folder:     {folder_paths['day_name']}")
    print(f"Section:        {folder_paths['section_name']}")
    print(f"Section Path:   {folder_paths['section_folder']}")
    print(f"Visual Data:    {folder_paths['visual_data_folder']}")
    print(f"GPR Scan Data:  {folder_paths['gpr_scan_folder']}")
    print("Note: Tilt correction transformation will be saved to Section folder")
    print("="*70 + "\n")
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
            'stream_host': '192.168.168.100', # TODO: change to 172.16.14.195 (R) / 172.16.15.61 (A)
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
            # Avoid publishing on the same corrected topic as the Python tilt corrector
            'corrected_odom_topic': '/Odometry_tilt_corrected_diff_cpp'
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
        prefix='taskset -c 1',  # Dedicated core for LiDAR driver
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

    # Fast-LIO2 Node (single-threaded SLAM, needs one fast core)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        prefix='taskset -c 0',  # Dedicated core 0 for Fast-LIO2 (single-threaded)
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

    # Raw Map Saver - Save final accumulated map on-demand (press M key in teleop)
    raw_map_saver = Node(
        package='pilot_control',
        executable='raw_map_saver',
        name='raw_map_saver',
        output='screen',
        parameters=[{
            'input_topic': '/Laser_map',
            'save_directory': folder_paths['section_folder'],  # Save to session folder
            'auto_save_enabled': False,  # Disabled - save only when M key pressed
            'auto_save_interval_sec': 30.0,
            'apply_tilt_correction': True,  # Apply IMU-based tilt correction when saving
            'save_raw_backup': False,  # Set to True to keep uncorrected map as backup
            'save_format': 'compressed'  # 'compressed' (50% smaller, best for wireless transfer), 'binary' (fast), 'ascii' (debug)
        }]
    )

    # PCD Processor - REMOVED (not needed - using raw maps only)

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
            'gpr_scan_data_directory': folder_paths['gpr_scan_folder'],  # Session-specific GPR CSV folder
            'rosbag_topics': [
                '/Odometry',  # Raw Fast-LIO odometry
                '/Odometry_tilt_corrected_diff',  # Tilt-corrected odometry from diff_drive_controller
                '/cmd_vel',
                '/left/controller_status',
                '/right/controller_status',
                '/gpr/controller_status',
                '/path',
                # '/projected_map',  # 2D occupancy grid from octomap
                '/tf',
                '/tf_static',
                '/livox/imu'
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

    # MPC Autonomous Controller - MPC-based autonomous waypoint navigation
    # Uses tilt-corrected odometry and ODrive CAN wheel encoders.
    mpc_controller_node = Node(
        package='pilot_control',
        executable='mpc_autonomous_controller.py',
        name='mpc_autonomous_controller',
        output='screen',
        parameters=[{
            # Robot kinematics
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'gear_ratio': 1.0,
            'invert_left': False,
            'invert_right': True,

            # Control parameters
            'control_frequency': 10.0,          # Hz
            'max_linear_velocity': 0.5,         # m/s
            'max_angular_velocity': 4.0,        # rad/s

            # MPC parameters
            'mpc_horizon': 50,
            'mpc_dt': 0.1,
            'mpc_Q_xe': 15.0,
            'mpc_Q_ye': 20.0,
            'mpc_Q_yaw': 5.0,
            'mpc_R_delta': 0.00010,
            'mpc_weight_increase_xe': 0.0,
            'mpc_weight_increase_ye': 0.1,
            'mpc_weight_increase_yaw': 0.0,

            # Slip estimation parameters
            'slip_history_length': 100,
            'slip_estimation_window': 1.0,

            # Waypoint / stopping parameters
            'lookahead_distance': 0.5,
            'target_reached_threshold': 0.05,

            # Topic names (match existing robot wiring)
            'odometry_topic': '/Odometry_tilt_corrected_diff',
            'left_control_topic': '/left/control_message',
            'right_control_topic': '/right/control_message',
            'left_encoder_topic': '/left/controller_status',
            'right_encoder_topic': '/right/controller_status',

            # Solver debug
            'solver_debug_enabled': False,
        }]
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
                left_odrive_node,
                right_odrive_node,
                gpr_odrive_node,
                diff_drive_controller,
                mpc_controller_node,
                livox_driver,  # Core 1 (dedicated)
                fast_lio_node,  # Core 0 (dedicated, single-threaded SLAM)
                # laser_map_rotator_node,
                body_to_foot_transform,
                camera_init_to_foot_init_transform,
                odom_tilt_corrector_node,
                raw_map_saver,  # Enabled - saves final accumulated map (press M)
                # pcd_processor_node - REMOVED (not needed - using raw maps only)
                # octomap_server_node,  # Enabled - generates 2D projected map
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