from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for teleoperation (runs on laptop)
    host_teleop_node = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        prefix='xterm -e',
        output='screen'
    )

    return LaunchDescription([
        host_teleop_node,
    ]) 