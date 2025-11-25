from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # No streaming viewer; recording-only system
    # Node for teleoperation (runs on laptop)
    host_teleop_node = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        prefix='xterm -e',
        output='screen'
    )

    # PCD Processor Node - REMOVED (not needed - using raw maps only)

    # gpr_serial_bridge_node = Node(
    #     package='pilot_control',
    #     executable='gpr_serial_bridge.py',
    #     name='gpr_serial_bridge',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',  # Arduino is on robot, not laptop
    #         'baud_rate': 9600
    #     }]
    # )

    # Streaming viewer removed

    return LaunchDescription([
        host_teleop_node,
        # pcd_processor_node - REMOVED (not needed - using raw maps only)
        # gpr_serial_bridge_node,  # runs on robot instead
    ]) 