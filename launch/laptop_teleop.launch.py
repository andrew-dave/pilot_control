from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Video viewer args
    declare_view_enabled = DeclareLaunchArgument('view_stream', default_value='true')
    declare_view_port = DeclareLaunchArgument('view_stream_port', default_value='5600')
    # Node for teleoperation (runs on laptop)
    host_teleop_node = Node(
        package='pilot_control',
        executable='host_teleop',
        name='host_teleop',
        prefix='xterm -e',
        output='screen'
    )

    # PCD Processor Node (runs on laptop, processes saved raw maps)
    pcd_processor_node = Node(
        package='pilot_control',
        executable='pcd_processor',
        name='pcd_processor',
        output='screen',
        parameters=[{
            'raw_map_directory': '/home/avenblake/robot_maps',  # Read raw maps from laptop (copied from robot)
            'processed_map_directory': '/home/avenblake/robot_maps',  # Save processed maps on laptop
            'processing_mode': 'high_quality',  # Options: minimal, fast, high_quality
            'voxel_size': 0.05,
            'remove_outliers': True,
            'outlier_std_dev': 2.0,
            'apply_rotation_correction': True,
            'rotation_angle': -2.6179,  # -150 degrees in radians
            'save_format': 'pcd',
            'auto_shutdown': True,  # Keep running after processing
            'max_height': 2.0,
            'min_height': 0.1,
            'find_latest_raw_map': True  # Automatically find the latest raw map
        }]
    )

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

    # UDP H.264 viewer (runs on laptop)
    gst_viewer = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('view_stream')),
        cmd=[
            'gst-launch-1.0', '-v',
            'udpsrc',
            PythonExpression(["'port=' + str(", LaunchConfiguration('view_stream_port'), ")"]),
            'caps=application/x-rtp,media=video,encoding-name=H264,payload=96',
            '!', 'rtph264depay', '!', 'decodebin', '!', 'autovideosink', 'sync=false'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_view_enabled,
        declare_view_port,
        host_teleop_node,
        pcd_processor_node,
        # gpr_serial_bridge_node,  # runs on robot instead
        gst_viewer,
    ]) 