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

    # PCD Saver Node (runs on laptop, saves maps locally)
    pcd_saver_node = Node(
        package='pilot_control',
        executable='pcd_saver',
        name='pcd_saver',
        output='screen',
        parameters=[{
            'save_directory': '/home/avenblake/robot_maps',  # Save on laptop
            'save_interval': 60,
            'max_height': 2.0,
            'min_height': 0.0,
            'remove_outliers': True,
            'outlier_std_dev': 1.0,
            'voxel_size': 0.1,  # Increased to avoid PCL overflow
            'auto_save': False,  # Disable auto-save, only manual save via 'M' key
            'save_on_laptop': True,  # This ensures it saves on laptop
            'apply_rotation_correction': True,  # Re-enable rotation
            'rotation_angle': 0.5230  # Try positive 30 degrees instead
        }]
    )

    return LaunchDescription([
        host_teleop_node,
        pcd_saver_node,
    ]) 