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
        default_value='1.0',
        description='A multiplier to tune the robot speed.'
    )
    declare_turn_speed_multiplier_arg = DeclareLaunchArgument(
        'turn_speed_multiplier',
        default_value='1.0',
        description='A multiplier to tune the robot turning speed.'
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

    # Differential Drive Controller
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
            'turn_speed_multiplier': LaunchConfiguration('turn_speed_multiplier')
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
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 3.0,  # Reduced from 10Hz to 3Hz to reduce data load
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
    # Transform from body to foot (corrects the 30-degree tilt)
    body_to_foot_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_foot_transform',
        arguments=['-0.17','0','0','0','-0.5230','0', 'body', 'foot'],
        output='screen'
    )
    
    # Transform from camera_init to foot_init (corrects the global frame for visualization)
    camera_init_to_foot_init_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_foot_init_transform',
        arguments=['-0.17','0','0','0','-0.5230','0', 'camera_init', 'foot_init'],
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
                left_odrive_node,
                right_odrive_node,
                diff_drive_controller,
                foxglove_bridge,
                livox_driver,
                fast_lio_node,
                body_to_foot_transform,
                camera_init_to_foot_init_transform,
                raw_map_saver,
                shutdown_service_node,
                gpr_serial_bridge_node,
            ]
        ),
    ]) 