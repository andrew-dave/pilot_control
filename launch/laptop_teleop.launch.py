from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _make_zenoh_bridge(context, *args, **kwargs):
    """
    Generate a laptop Zenoh config with the selected robot endpoint.
    This avoids hard-coding a single robot IP in the repo config.
    """
    robot_ip = LaunchConfiguration('robot_ip').perform(context)

    template_path = os.path.join(
        get_package_share_directory('pilot_control'),
        'config', 'zenoh', 'zenohd_laptop.json5'
    )
    with open(template_path, 'r', encoding='utf-8') as f:
        cfg = f.read()

    cfg = cfg.replace('__ROBOT_IP__', robot_ip)
    out_path = os.path.join('/tmp', f'zenohd_laptop_{robot_ip}.json5')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(cfg)

    return [
        ExecuteProcess(
            cmd=['zenohd', '-c', out_path],
            output='screen',
            name='zenoh_bridge_dds'
        )
    ]

def generate_launch_description():
    declare_robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.168.101',
        description='Robot Microhard IP for Zenoh (tcp/<robot_ip>:7447).'
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
        declare_robot_ip_arg,
        OpaqueFunction(function=_make_zenoh_bridge),
        
        # Teleop node (with delay to ensure Zenoh bridge is ready)
        TimerAction(
            period=2.0,
            actions=[host_teleop_node]
        ),
        # pcd_processor_node - REMOVED (not needed - using raw maps only)
        # gpr_serial_bridge_node,  # runs on robot instead
    ]) 