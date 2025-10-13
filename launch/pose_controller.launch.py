from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments to allow non-conflicting topics/params if desired
    namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='ROS namespace for the node'
    )
    odom_topic = DeclareLaunchArgument(
        'odometry_topic', default_value='/Odometry', description='Odometry topic name'
    )
    left_topic = DeclareLaunchArgument(
        'left_control_topic', default_value='/left/control_message', description='Left wheel control topic'
    )
    right_topic = DeclareLaunchArgument(
        'right_control_topic', default_value='/right/control_message', description='Right wheel control topic'
    )

    # Core controller parameters (exposed for convenience)
    enable_controller = DeclareLaunchArgument(
        'enable_controller', default_value='true', description='Start controller enabled'
    )
    control_frequency = DeclareLaunchArgument(
        'control_frequency', default_value='10.0', description='Control loop frequency (Hz)'
    )
    max_linear_velocity = DeclareLaunchArgument(
        'max_linear_velocity', default_value='0.3', description='Max linear velocity (m/s)'
    )
    max_angular_velocity = DeclareLaunchArgument(
        'max_angular_velocity', default_value='1.5', description='Max angular velocity (rad/s)'
    )

    # CAN interface args
    can_interface = DeclareLaunchArgument(
        'can_interface', default_value='can0', description='CAN interface name'
    )
    can_bitrate = DeclareLaunchArgument(
        'can_bitrate', default_value='250000', description='CAN bitrate'
    )

    # Core controller parameters (exposed for convenience)
    enable_controller = DeclareLaunchArgument(
        'enable_controller', default_value='true', description='Start controller enabled'
    )
    control_frequency = DeclareLaunchArgument(
        'control_frequency', default_value='10.0', description='Control loop frequency (Hz)'
    )
    max_linear_velocity = DeclareLaunchArgument(
        'max_linear_velocity', default_value='0.3', description='Max linear velocity (m/s)'
    )
    max_angular_velocity = DeclareLaunchArgument(
        'max_angular_velocity', default_value='1.5', description='Max angular velocity (rad/s)'
    )

    # Pose controller node
    pose_controller_node = Node(
        package='pilot_control',
        executable='pose_controller.py',
        name='pose_controller',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'left_control_topic': LaunchConfiguration('left_control_topic'),
            'right_control_topic': LaunchConfiguration('right_control_topic'),
            'enable_controller': LaunchConfiguration('enable_controller'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }],
    )

    # Bring up CAN interface before ODrive nodes
    can_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', LaunchConfiguration('can_interface'),
             'up', 'type', 'can', 'bitrate', LaunchConfiguration('can_bitrate')],
        output='screen'
    )

    # ODrive CAN nodes (left/right) — subscribe to /left|/right/control_message
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

    # Livox driver (provides point cloud for Fast-LIO)
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
            'lvx_file_path': '/home/robot/livox_test.lvx',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'
            ])
        }]
    )

    # Fast-LIO mapper (publishes /Odometry)
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

    return LaunchDescription([
        # Launch args
        namespace,
        odom_topic,
        left_topic,
        right_topic,
        can_interface,
        can_bitrate,
        enable_controller,
        control_frequency,
        max_linear_velocity,
        max_angular_velocity,

        # Bring up CAN, then start hardware-related nodes and controller
        TimerAction(period=1.0, actions=[can_setup]),
        TimerAction(period=3.0, actions=[left_odrive_node, right_odrive_node]),

        # Odometry pipeline
        livox_driver,
        fast_lio_node,

        # Pose controller last
        pose_controller_node,
    ])


