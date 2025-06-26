from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments for robot-specific configuration
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.063',
        description='Radius of the wheels in meters.'
    )
    declare_wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.315',
        description='Distance between the two wheels in meters.'
    )
    declare_can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='The CAN interface name.'
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

    # Node for the left motor
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

    # Node for the right motor
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

    # Node for the differential drive controller
    diff_drive_controller_node = Node(
        package='pilot_control',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'velocity_multiplier': LaunchConfiguration('velocity_multiplier'),
            'turn_speed_multiplier': LaunchConfiguration('turn_speed_multiplier')
        }],
        output='screen'
    )

    # Node for teleoperation
    host_teleop_node = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        prefix='xterm -e',
        output='screen'
    )

    return LaunchDescription([
        declare_wheel_radius_arg,
        declare_wheel_base_arg,
        declare_can_interface_arg,
        declare_velocity_multiplier_arg,
        declare_turn_speed_multiplier_arg,
        left_odrive_node,
        right_odrive_node,
        diff_drive_controller_node,
        host_teleop_node
    ]) 