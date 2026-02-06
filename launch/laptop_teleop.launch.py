from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Start Zenoh bridge DDS daemon for Microhard communication
    # This must be running before ROS nodes launch for cross-network DDS communication
    zenoh_bridge = ExecuteProcess(
        cmd=['zenohd', '-c', os.path.expanduser('~/zenohd_laptop.json5')],
        output='screen',
        name='zenoh_bridge_dds'
    )
    
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
        # Zenoh bridge DDS (must start first for Microhard communication)
        zenoh_bridge,
        
        # Teleop node (with delay to ensure Zenoh bridge is ready)
        TimerAction(
            period=2.0,
            actions=[host_teleop_node]
        ),
        # pcd_processor_node - REMOVED (not needed - using raw maps only)
        # gpr_serial_bridge_node,  # runs on robot instead
    ]) 