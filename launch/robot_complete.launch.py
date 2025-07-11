from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

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
    declare_enable_pcd_saver_arg = DeclareLaunchArgument(
        'enable_pcd_saver',
        default_value='true',
        description='Enable PCD saving for mapping data.'
    )
    declare_save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='/home/robot/maps',
        description='Directory to save PCD files.'
    )

    # CAN interface setup command
    can_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', LaunchConfiguration('can_interface'), 
             'up', 'type', 'can', 'bitrate', LaunchConfiguration('can_bitrate')],
        output='screen'
    )

    # Include the robot launch file
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

    # Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0'
        }]
    )

    # Livox ROS Driver (MID360)
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/robot/livox_test.lvx'
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

    # Map Relay Node (commented out - Fast-LIO2 doesn't publish occupancy grids)
    # map_relay_node = Node(
    #     package='pilot_control',
    #     executable='map_relay',
    #     name='map_relay',
    #     output='screen'
    # )

    # PCD Saver Node
    pcd_saver_node = Node(
        package='pilot_control',
        executable='pcd_saver',
        name='pcd_saver',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'save_interval': 60,
            'max_height': 2.0,
            'min_height': 0.1,
            'remove_outliers': True,
            'outlier_std_dev': 1.0,
            'voxel_size': 0.05,
            'auto_save': True,
            'save_on_laptop': True  # Save maps on laptop instead of robot
        }]
    )

    return LaunchDescription([
        # Launch arguments
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        declare_velocity_multiplier_arg,
        declare_turn_speed_multiplier_arg,
        declare_enable_pcd_saver_arg,
        declare_save_directory_arg,
        
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
                foxglove_bridge,
                livox_driver,
                fast_lio_node,
                pcd_saver_node,
            ]
        ),
    ]) 