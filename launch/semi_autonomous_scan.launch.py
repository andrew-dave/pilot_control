from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

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

    # Pose controller parameters
    declare_enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='true',
        description='Start pose controller enabled'
    )
    declare_control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control loop frequency (Hz)'
    )
    declare_max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.3',
        description='Max linear velocity (m/s)'
    )
    declare_max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='20.0',
        description='Max angular velocity (rad/s)'
    )
    declare_roll_rad_arg = DeclareLaunchArgument(
        'roll_rad',
        default_value='0.0',
        description='Roll correction angle (rad)'
    )
    declare_pitch_rad_arg = DeclareLaunchArgument(
        'pitch_rad',
        default_value='-0.2617993878',
        description='Pitch correction angle (rad)'
    )
    declare_yaw_rad_arg = DeclareLaunchArgument(
        'yaw_rad',
        default_value='0.0',
        description='Yaw correction angle (rad)'
    )
    
    # Error computation parameters
    declare_lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.4',
        description='Lookahead distance for waypoint generation (m)'
    )
    declare_r_close_arg = DeclareLaunchArgument(
        'r_close',
        default_value='0.03',
        description='Close distance threshold for yaw correction (m)'
    )
    declare_r_far_arg = DeclareLaunchArgument(
        'r_far',
        default_value='0.05',
        description='Far distance threshold for lateral control (m)'
    )
    declare_K_lat_arg = DeclareLaunchArgument(
        'K_lat',
        default_value='1.0',
        description='Lateral correction gain'
    )
    declare_K_yaw_arg = DeclareLaunchArgument(
        'K_yaw',
        default_value='1.0',
        description='Yaw correction gain'
    )
    declare_yaw_tol_arg = DeclareLaunchArgument(
        'yaw_tol',
        default_value='0.1',
        description='Yaw tolerance for target achievement (rad)'
    )
    declare_imu_flip_x_arg = DeclareLaunchArgument(
        'imu_flip_x',
        default_value='true',
        description='Flip IMU X-axis'
    )
    declare_imu_flip_y_arg = DeclareLaunchArgument(
        'imu_flip_y',
        default_value='false',
        description='Flip IMU Y-axis'
    )
    declare_imu_flip_z_arg = DeclareLaunchArgument(
        'imu_flip_z',
        default_value='true',
        description='Flip IMU Z-axis'
    )

    # Unified Data Collector (Thermal + Dual Cameras + Odometry sync)
    unified_data_collector_node = Node(
        package='pilot_control',
        executable='unified_data_collector',
        name='unified_data_collector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            # Odometry and thermal camera settings
            'fastlio_odom_topic': '/Odometry',
            'log_directory': os.path.join(os.path.expanduser('~'), 'unified_scans'),
            'use_seekvision_mode': True,
            'save_color_png': True,
            'csv_flush_every_rows': 50,
            'log_frequency_hz': 10.0,
            
            # Camera device paths (See3CAM)
            'left_device': '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_3728140416020900-video-index0',
            'right_device': '/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_0F12140416020900-video-index0',
            
            # Streaming settings
            'stream_host': '172.16.15.61', # TODO: change to 172.16.14.195 (R) / 172.16.15.61 (A)
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

    # Pose Controller - Autonomous navigation to target poses
    pose_controller_node = Node(
        package='pilot_control',
        executable='pose_controller.py',
        name='pose_controller',
        output='screen',
        parameters=[{
            'odometry_topic': '/Odometry',
            'left_control_topic': '/left/control_message',
            'right_control_topic': '/right/control_message',
            'enable_controller': LaunchConfiguration('enable_controller'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'roll_rad': LaunchConfiguration('roll_rad'),
            'pitch_rad': LaunchConfiguration('pitch_rad'),
            'yaw_rad': LaunchConfiguration('yaw_rad'),
            'imu_flip_x': LaunchConfiguration('imu_flip_x'),
            'imu_flip_y': LaunchConfiguration('imu_flip_y'),
            'imu_flip_z': LaunchConfiguration('imu_flip_z'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'velocity_multiplier': LaunchConfiguration('velocity_multiplier'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'r_close': LaunchConfiguration('r_close'),
            'r_far': LaunchConfiguration('r_far'),
            'K_lat': LaunchConfiguration('K_lat'),
            'K_yaw': LaunchConfiguration('K_yaw'),
            'yaw_tol': LaunchConfiguration('yaw_tol'),
        }],
    )

    # NOTE: diff_drive_controller is EXCLUDED from this launch file
    # Pose controller provides autonomous navigation instead

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
        parameters=[PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'mid360.yaml'
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

    # GPR Scan Controller - EXCLUSIVE GPR motor control + synchronized scan + logging
    # This is the ONLY node that controls the GPR motor (diff_drive_controller GPR control is disabled)
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
            'fastlio_odom_topic': '/Odometry',
            'log_frequency_hz': 50.0,            # 50 Hz logging
            'log_directory': os.path.join(os.path.expanduser('~'), 'gpr_scans')
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
            'pitch_rad': 0.2617993878,  # 15 degrees
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

    return LaunchDescription([
        # Launch arguments
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        declare_velocity_multiplier_arg,
        declare_turn_speed_multiplier_arg,
        
        # Pose controller arguments
        declare_enable_controller_arg,
        declare_control_frequency_arg,
        declare_max_linear_velocity_arg,
        declare_max_angular_velocity_arg,
        declare_roll_rad_arg,
        declare_pitch_rad_arg,
        declare_yaw_rad_arg,
        declare_lookahead_distance_arg,
        declare_r_close_arg,
        declare_r_far_arg,
        declare_K_lat_arg,
        declare_K_yaw_arg,
        declare_yaw_tol_arg,
        declare_imu_flip_x_arg,
        declare_imu_flip_y_arg,
        declare_imu_flip_z_arg,
        
        
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
                pose_controller_node,  # Pose controller for autonomous navigation
                # diff_drive_controller EXCLUDED - pose controller provides navigation instead
                #foxglove_bridge,
                livox_driver,
                fast_lio_node,
                #laser_map_rotator_node,
                #body_to_foot_transform,
                #camera_init_to_foot_init_transform,
                #raw_map_saver,
                #octomap_server_node,
                shutdown_service_node,
                #unified_data_collector_node,
                
                #gpr_serial_bridge_node
                #gpr_scan_controller_node,
            ]
        ),
    ])
