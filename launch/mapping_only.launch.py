from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.063',
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
        default_value='0.8',
        description='A multiplier to tune the robot speed.'
    )
    declare_turn_speed_multiplier_arg = DeclareLaunchArgument(
        'turn_speed_multiplier',
        default_value='0.4',
        description='A multiplier to tune the robot turning speed.'
    )

    # CAN interface setup command
    can_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', LaunchConfiguration('can_interface'), 
             'up', 'type', 'can', 'bitrate', LaunchConfiguration('can_bitrate')],
        output='screen'
    )

    # Include the robot launch file (basic robot control only)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pilot_control'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments=[
            ('wheel_radius', LaunchConfiguration('wheel_radius')),
            ('wheel_base', LaunchConfiguration('wheel_base')),
            ('can_interface', LaunchConfiguration('can_interface')),
            ('velocity_multiplier', LaunchConfiguration('velocity_multiplier')),
            ('turn_speed_multiplier', LaunchConfiguration('turn_speed_multiplier'))
        ]
    )

    # Livox ROS Driver (MID360) - Full frequency for optimal Fast-LIO2 performance
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,  # Full frequency for best Fast-LIO2 performance
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/robot/livox_test.lvx',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'
            ])
        }]
    )

    # Fast-LIO2 Node with optimized settings for performance
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'mid360.yaml'
        ]), {
            'use_sim_time': False,
            'publish/path_en': True,
            'publish/map_en': True,
            'publish/scan_publish_en': True,
            'publish/dense_publish_en': False,  # Disable dense publishing for performance
            'publish/scan_bodyframe_pub_en': False,  # Reduce TF publishing
            'pcd_save/pcd_save_en': False  # Disable PCD saving during mapping
        }],
        output='screen'
    )

    # Static Transform Publishers for Tilted LiDAR Setup
    body_to_foot_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_foot_transform',
        arguments=['0', '0', '0', '0', '-0.5230', '0', 'body', 'foot'],
        output='screen'
    )
    
    camera_init_to_foot_init_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_foot_init_transform',
        arguments=['0', '0', '0', '0', '-0.5230', '0', 'camera_init', 'foot_init'],
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
            'save_directory': '/home/robot/maps'
        }]
    )

    # Shutdown Service Node (for remote shutdown)
    shutdown_service_node = Node(
        package='pilot_control',
        executable='shutdown_service',
        name='shutdown_service',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        declare_velocity_multiplier_arg,
        declare_turn_speed_multiplier_arg,
        
        # CAN setup (with delay to ensure it's ready)
        TimerAction(
            period=1.0,
            actions=[can_setup]
        ),
        
        # Robot and mapping components (with delay after CAN setup)
        TimerAction(
            period=3.0,
            actions=[
                robot_launch,
                livox_driver,
                fast_lio_node,
                body_to_foot_transform,
                camera_init_to_foot_init_transform,
                raw_map_saver,
                shutdown_service_node,
            ]
        ),
    ]) 