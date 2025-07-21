from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
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
            'auto_shutdown': False,  # Keep running after processing
            'max_height': 2.0,
            'min_height': 0.1,
            'find_latest_raw_map': True  # Automatically find the latest raw map
        }]
    )

    # GPR Serial Bridge Node (runs on laptop, communicates with Arduino)
    # gpr_serial_bridge_node = Node(
    #     package='pilot_control',
    #     executable='gpr_serial_bridge.py',
    #     name='gpr_serial_bridge',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',  # Change this to match your Arduino port
    #         'baud_rate': 9600
    #     }]
    # )

    return LaunchDescription([
        host_teleop_node,
        pcd_processor_node,
        # gpr_serial_bridge_node,  # Temporarily disabled
    ]) 