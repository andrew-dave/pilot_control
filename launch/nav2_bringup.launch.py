from launch import LaunchDescription
from launch_ros.actions import Node
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
        # Launch TF bridge to connect camera_init and base_link
        Node(
            package='pilot_control',
            executable='tf_bridge',
            output='screen',
            name='tf_bridge'
        ),
        
        # Launch individual Nav2 components instead of full bringup
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('/map', '/Laser_map')
            ]
        ),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('/map', '/Laser_map')
            ]
        ),
        
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[nav2_params_file],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ]) 