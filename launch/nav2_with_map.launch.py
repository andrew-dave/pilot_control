from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/avenblake/robot_maps/nav2_maps/nav2_map_20250716_174625.yaml',
        description='Path to the map YAML file'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('pilot_control'), 'config', 'nav2_params.yaml'
        ]),
        description='Path to the Nav2 parameters file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Static Transform Publisher: camera_init to map (identity transform)
    # This allows Nav2 to use the standard 'map' frame while Fast-LIO2 uses 'camera_init'
    camera_init_to_map_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_map_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        output='screen'
    )
    
    # Map Saver Node (optional, for saving maps)
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True
        }]
    )
    
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # Nav2 Bringup
    nav2_bringup = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
             'params_file:=' + str(LaunchConfiguration('params_file')),
             'use_sim_time:=' + str(LaunchConfiguration('use_sim_time')),
             'map:=' + str(LaunchConfiguration('map'))],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        
        # Map server and transforms
        map_server_node,
        camera_init_to_map_transform,
        
        # AMCL and lifecycle
        amcl_node,
        lifecycle_manager,
        
        # Nav2 bringup (includes controller, planner, etc.)
        nav2_bringup,
    ]) 