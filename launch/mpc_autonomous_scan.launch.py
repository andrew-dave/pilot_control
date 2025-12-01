from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path

def generate_launch_description():
    # Declare launch arguments
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.09',
        description='Radius of the wheels in meters.'
    )
    declare_wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.32',
        description='Distance between the two wheels in meters.'
    )
    declare_gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='Gear ratio for motor control.'
    )
    declare_invert_left_arg = DeclareLaunchArgument(
        'invert_left',
        default_value='false',
        description='Invert left motor direction.'
    )
    declare_invert_right_arg = DeclareLaunchArgument(
        'invert_right',
        default_value='true',
        description='Invert right motor direction.'
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

    # MPC Controller parameters
    declare_control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control loop frequency (Hz)'
    )
    declare_max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Max linear velocity (m/s)'
    )
    declare_max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='4.0',
        description='Max angular velocity (rad/s)'
    )
    
    # MPC optimization parameters
    declare_mpc_horizon_arg = DeclareLaunchArgument(
        'mpc_horizon',
        default_value='50',
        description='MPC prediction horizon (number of steps)'
    )
    declare_mpc_dt_arg = DeclareLaunchArgument(
        'mpc_dt',
        default_value='0.1',
        description='MPC time step (seconds)'
    )
    declare_mpc_Q_xe_arg = DeclareLaunchArgument(
        'mpc_Q_xe',
        default_value='5.0',
        description='MPC cost weight for position error x'
    )
    declare_mpc_Q_ye_arg = DeclareLaunchArgument(
        'mpc_Q_ye',
        default_value='10.0',
        description='MPC cost weight for position error y (higher for lateral correction)'
    )
    declare_mpc_Q_yaw_arg = DeclareLaunchArgument(
        'mpc_Q_yaw',
        default_value='10.0',
        description='MPC cost weight for yaw error (higher to keep robot facing along the path and avoid backwards tracking)'
    )
    declare_mpc_R_delta_arg = DeclareLaunchArgument(
        'mpc_R_delta',
        default_value='0.00007',
        description='MPC cost weight for control input change (delta u)'
    )
    declare_mpc_weight_increase_xe_arg = DeclareLaunchArgument(
        'mpc_weight_increase_xe',
        default_value='0.0',
        description='Linear weight increase factor per time step for xe error. Weight at step k = base_weight * (1 + weight_increase_xe * k). Example: 0.1 means 10% increase per step.'
    )
    declare_mpc_weight_increase_ye_arg = DeclareLaunchArgument(
        'mpc_weight_increase_ye',
        default_value='0.5',
        description='Linear weight increase factor per time step for ye error. Weight at step k = base_weight * (1 + weight_increase_ye * k). Example: 0.1 means 10% increase per step.'
    )
    declare_mpc_weight_increase_yaw_arg = DeclareLaunchArgument(
        'mpc_weight_increase_yaw',
        default_value='0.0',
        description='Linear weight increase factor per time step for yaw error. Weight at step k = base_weight * (1 + weight_increase_yaw * k). Example: 0.1 means 10% increase per step.'
    )
    
    # Slip estimation parameters
    declare_slip_history_length_arg = DeclareLaunchArgument(
        'slip_history_length',
        default_value='100',
        description='History buffer size for slip estimation'
    )
    declare_slip_estimation_window_arg = DeclareLaunchArgument(
        'slip_estimation_window',
        default_value='1.0',
        description='Time window for slip estimation (seconds)'
    )
    
    # Waypoint parameters
    declare_lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Lookahead distance for waypoint generation (m)'
    )
    
    # Topic names
    declare_odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/Odometry_tilt_corrected_diff',
        description='Odometry topic name'
    )
    declare_left_control_topic_arg = DeclareLaunchArgument(
        'left_control_topic',
        default_value='/left/control_message',
        description='Left wheel control topic'
    )
    declare_right_control_topic_arg = DeclareLaunchArgument(
        'right_control_topic',
        default_value='/right/control_message',
        description='Right wheel control topic'
    )
    declare_left_encoder_topic_arg = DeclareLaunchArgument(
        'left_encoder_topic',
        default_value='/left/controller_status',
        description='Left encoder topic'
    )
    declare_right_encoder_topic_arg = DeclareLaunchArgument(
        'right_encoder_topic',
        default_value='/right/controller_status',
        description='Right encoder topic'
    )

    # Odometry Tilt Corrector (provides tilt-corrected odometry for MPC controller)
    launch_file_path = Path(__file__).resolve()
    
    # Detect if we're in source (src/) or install (install/) directory
    path_str = str(launch_file_path)
    if '/src/' in path_str:
        # Source location: src/pilot_controol/scripts/odom_tilt_corrector.py
        src_base = Path(path_str[:path_str.index('/src/') + 4])  # Include '/src'
        source_script_dir = src_base / 'pilot_control' / 'scripts'
        odom_tilt_corrector_path = str(source_script_dir / 'odom_tilt_corrector.py')
    elif '/install/' in path_str:
        # Install location: install/pilot_control/lib/pilot_control/odom_tilt_corrector.py
        install_base = Path(path_str[:path_str.index('/install/') + 8])  # Include '/install'
        install_script_dir = install_base / 'pilot_control' / 'lib' / 'pilot_control'
        odom_tilt_corrector_path = str(install_script_dir / 'odom_tilt_corrector.py')
    else:
        # Fallback: try relative to launch file (for source)
        source_script_dir = launch_file_path.parent.parent / 'scripts'
        odom_tilt_corrector_path = str(source_script_dir / 'odom_tilt_corrector.py')
    
    odom_tilt_corrector_proc = ExecuteProcess(
        cmd=[
            'python3', odom_tilt_corrector_path,
            '--ros-args',
            '-p', 'odometry_topic:=/Odometry',
            '-p', 'accel_topic:=/livox/imu',
            '-p', 'accel_samples:=10',
            '-p', 'corrected_odometry_topic:=/Odometry_tilt_corrected_diff'
        ],
        output='screen'
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
        prefix='taskset -c 5',
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
        prefix='taskset -c 5',
        namespace='right',
        name='odrive_can_right',
        parameters=[{
            'node_id': 1,
            'interface': LaunchConfiguration('can_interface')
        }],
        output='screen'
    )

    # MPC Autonomous Controller - MPC-based autonomous navigation
    mpc_controller_node = Node(
        package='pilot_control',
        executable='mpc_autonomous_controller.py',
        prefix='taskset -c 5',
        name='mpc_autonomous_controller',
        output='screen',
        parameters=[{
            # Robot kinematics
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'invert_left': LaunchConfiguration('invert_left'),
            'invert_right': LaunchConfiguration('invert_right'),
            
            # Control parameters
            'control_frequency': LaunchConfiguration('control_frequency'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            
            # MPC parameters
            'mpc_horizon': LaunchConfiguration('mpc_horizon'),
            'mpc_dt': LaunchConfiguration('mpc_dt'),
            'mpc_Q_xe': LaunchConfiguration('mpc_Q_xe'),
            'mpc_Q_ye': LaunchConfiguration('mpc_Q_ye'),
            'mpc_Q_yaw': LaunchConfiguration('mpc_Q_yaw'),
            'mpc_R_delta': LaunchConfiguration('mpc_R_delta'),
            'mpc_weight_increase_xe': LaunchConfiguration('mpc_weight_increase_xe'),
            'mpc_weight_increase_ye': LaunchConfiguration('mpc_weight_increase_ye'),
            'mpc_weight_increase_yaw': LaunchConfiguration('mpc_weight_increase_yaw'),
            
            # Solver debug
            'solver_debug_enabled': True,
            
            # Slip estimation parameters
            'slip_history_length': LaunchConfiguration('slip_history_length'),
            'slip_estimation_window': LaunchConfiguration('slip_estimation_window'),
            
            # Waypoint parameters
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            
            # Topic names
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'left_control_topic': LaunchConfiguration('left_control_topic'),
            'right_control_topic': LaunchConfiguration('right_control_topic'),
            'left_encoder_topic': LaunchConfiguration('left_encoder_topic'),
            'right_encoder_topic': LaunchConfiguration('right_encoder_topic'),
        }],
    )

    # Livox ROS Driver (MID360)
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
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/robot/livox_test.lvx',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'
            ])
        }]
    )

    # Fast-LIO2 Node
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
    body_to_foot_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_foot_transform',
        arguments=['0.0', '0.0', '0.0', '0.0', '-0.2618', '0.0', 'body', 'foot'],
        output='screen'
    )
    
    camera_init_to_foot_init_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_foot_init_transform',
        arguments=['0.0','0.0','0.0','0.0','-0.2618','0.0', 'camera_init', 'foot_init'],
        output='screen'
    )

    # Shutdown Service Node (for remote shutdown)
    shutdown_service_node = Node(
        package='pilot_control',
        executable='shutdown_service',
        name='shutdown_service',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments - Robot kinematics
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_gear_ratio_arg,
        declare_invert_left_arg,
        declare_invert_right_arg,
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        
        # MPC Controller arguments
        declare_control_frequency_arg,
        declare_max_linear_velocity_arg,
        declare_max_angular_velocity_arg,
        declare_mpc_horizon_arg,
        declare_mpc_dt_arg,
        declare_mpc_Q_xe_arg,
        declare_mpc_Q_ye_arg,
        declare_mpc_Q_yaw_arg,
        declare_mpc_R_delta_arg,
        declare_mpc_weight_increase_xe_arg,
        declare_mpc_weight_increase_ye_arg,
        declare_mpc_weight_increase_yaw_arg,
        declare_slip_history_length_arg,
        declare_slip_estimation_window_arg,
        declare_lookahead_distance_arg,
        
        # Topic name arguments
        declare_odometry_topic_arg,
        declare_left_control_topic_arg,
        declare_right_control_topic_arg,
        declare_left_encoder_topic_arg,
        declare_right_encoder_topic_arg,
        
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
                livox_driver,
                fast_lio_node,
                odom_tilt_corrector_proc,  # Tilt correction (must start before MPC controller)
                mpc_controller_node,  # MPC controller for autonomous navigation
                body_to_foot_transform,
                camera_init_to_foot_init_transform,
                shutdown_service_node,
            ]
        ),
    ])

