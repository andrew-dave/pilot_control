from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


DEFAULT_LAPTOP_CONFIG_FILE = os.environ.get(
    'PILOT_LAPTOP_CONFIG',
    os.path.expanduser('~/pilot_config/laptop.yaml')
)


def load_laptop_config(config_path=DEFAULT_LAPTOP_CONFIG_FILE):
    """Load machine-local laptop config from outside the git repo."""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}
        if not isinstance(config, dict):
            print(f"[Laptop Config] Ignoring {config_path}: top-level YAML must be a mapping")
            return {}
        print(f"[Laptop Config] Loaded external config: {config_path}")
        return config
    except FileNotFoundError:
        print(f"[Laptop Config] No external config at {config_path}; using in-repo defaults")
    except Exception as exc:
        print(f"[Laptop Config] Failed to load {config_path}: {exc}; using in-repo defaults")
    return {}


def config_value(config, dotted_key, default):
    current = config
    for part in dotted_key.split('.'):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def as_launch_bool(value, default):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return 'true'
        if normalized in ('0', 'false', 'no', 'off'):
            return 'false'
    return 'true' if default else 'false'


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
    laptop_config = load_laptop_config()

    declare_robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=str(config_value(laptop_config, 'teleop.robot_ip', '192.168.168.101')),
        description='Robot Microhard IP for Zenoh (tcp/<robot_ip>:7447).'
    )
    declare_use_xterm_arg = DeclareLaunchArgument(
        'use_xterm',
        default_value=as_launch_bool(config_value(laptop_config, 'teleop.use_xterm', True), True),
        description='Launch host_teleop in xterm window when true.'
    )
    declare_interactive_sdl_arg = DeclareLaunchArgument(
        'interactive_sdl',
        default_value=as_launch_bool(config_value(laptop_config, 'teleop.interactive_sdl', True), True),
        description='Enable SDL keyboard window in host_teleop.'
    )
    declare_cmd_vel_enabled_arg = DeclareLaunchArgument(
        'cmd_vel_enabled',
        default_value=as_launch_bool(config_value(laptop_config, 'teleop.cmd_vel_enabled', True), True),
        description='Enable cmd_vel publishing from host_teleop.'
    )
    
    # No streaming viewer; recording-only system
    # Node for teleoperation (runs on laptop)
    host_teleop_node_xterm = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        prefix='xterm -e',
        parameters=[{
            'interactive_sdl': LaunchConfiguration('interactive_sdl'),
            'cmd_vel_enabled': LaunchConfiguration('cmd_vel_enabled'),
        }],
        condition=IfCondition(LaunchConfiguration('use_xterm')),
        output='screen'
    )
    host_teleop_node_no_xterm = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        parameters=[{
            'interactive_sdl': LaunchConfiguration('interactive_sdl'),
            'cmd_vel_enabled': LaunchConfiguration('cmd_vel_enabled'),
        }],
        condition=UnlessCondition(LaunchConfiguration('use_xterm')),
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
        declare_use_xterm_arg,
        declare_interactive_sdl_arg,
        declare_cmd_vel_enabled_arg,
        OpaqueFunction(function=_make_zenoh_bridge),
        
        # Teleop node (with delay to ensure Zenoh bridge is ready)
        TimerAction(
            period=2.0,
            actions=[host_teleop_node_xterm, host_teleop_node_no_xterm]
        ),
        # pcd_processor_node - REMOVED (not needed - using raw maps only)
        # gpr_serial_bridge_node,  # runs on robot instead
    ]) 