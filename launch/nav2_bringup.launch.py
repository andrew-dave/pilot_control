from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to your parameter file
    nav2_params_file = os.path.join(
        get_package_share_directory('pilot_control'),
        'config',
        'nav2_params.yaml'
    )
    
    return LaunchDescription([
        # Launch Nav2 bringup using the launch file directly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'params_file': nav2_params_file,
                'use_sim_time': 'false'
            }.items()
        ),
    ]) 