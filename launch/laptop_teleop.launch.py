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
            'min_height': 0.1,
            'remove_outliers': True,
            'outlier_std_dev': 1.0,
            'voxel_size': 0.05,
            'auto_save': True,
            'save_on_laptop': True  # This ensures it saves on laptop
        }]
    )

    return LaunchDescription([
        host_teleop_node,
        pcd_saver_node,
    ]) 