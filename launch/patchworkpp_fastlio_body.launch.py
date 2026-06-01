from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    ground_topic = LaunchConfiguration("ground_topic")
    nonground_topic = LaunchConfiguration("nonground_topic")
    colored_topic = LaunchConfiguration("colored_topic")
    world_scores_topic = LaunchConfiguration("world_scores_topic")
    world_ground_topic = LaunchConfiguration("world_ground_topic")
    world_nonground_topic = LaunchConfiguration("world_nonground_topic")
    world_colored_topic = LaunchConfiguration("world_colored_topic")
    scan_frame = LaunchConfiguration("scan_frame")
    world_frame = LaunchConfiguration("world_frame")
    sensor_height = LaunchConfiguration("sensor_height")
    enable_world_accumulation = LaunchConfiguration("enable_world_accumulation")
    score_voxel_size = LaunchConfiguration("score_voxel_size")
    ground_probability_threshold = LaunchConfiguration("ground_probability_threshold")
    min_observations_per_voxel = LaunchConfiguration("min_observations_per_voxel")
    publish_world_map_every_n_scans = LaunchConfiguration("publish_world_map_every_n_scans")
    transform_timeout_sec = LaunchConfiguration("transform_timeout_sec")
    min_patchwork_range = LaunchConfiguration("min_patchwork_range")
    max_patchwork_range = LaunchConfiguration("max_patchwork_range")
    run_fastlio = LaunchConfiguration("run_fastlio")
    fastlio_config = LaunchConfiguration("fastlio_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_patchwork_saver = LaunchConfiguration("run_patchwork_saver")
    patchwork_save_directory = LaunchConfiguration("patchwork_save_directory")
    calibration_file = LaunchConfiguration("calibration_file")
    lidar_pitch_deg = LaunchConfiguration("lidar_pitch_deg")
    play_bag = LaunchConfiguration("play_bag")
    bag_path = LaunchConfiguration("bag_path")
    bag_start_delay_sec = LaunchConfiguration("bag_start_delay_sec")
    bag_rate = LaunchConfiguration("bag_rate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/cloud_registered_body",
            description="FastLIO body-frame scan topic to classify with Patchwork++.",
        ),
        DeclareLaunchArgument(
            "ground_topic",
            default_value="/patchworkpp/ground",
            description="Output topic for per-scan Patchwork++ ground points.",
        ),
        DeclareLaunchArgument(
            "nonground_topic",
            default_value="/patchworkpp/nonground",
            description="Output topic for per-scan Patchwork++ non-ground points.",
        ),
        DeclareLaunchArgument(
            "colored_topic",
            default_value="/patchworkpp/colored",
            description="Combined colorized per-scan Patchwork++ cloud.",
        ),
        DeclareLaunchArgument(
            "world_scores_topic",
            default_value="/patchworkpp/world_scores",
            description="Accumulated world-frame score map, encoded as intensity in [0, 1].",
        ),
        DeclareLaunchArgument(
            "world_ground_topic",
            default_value="/patchworkpp/world_ground",
            description="Accumulated world-frame final ground voxels.",
        ),
        DeclareLaunchArgument(
            "world_nonground_topic",
            default_value="/patchworkpp/world_nonground",
            description="Accumulated world-frame final non-ground voxels.",
        ),
        DeclareLaunchArgument(
            "world_colored_topic",
            default_value="/patchworkpp/world_colored",
            description="Combined colorized accumulated world-frame classification.",
        ),
        DeclareLaunchArgument(
            "scan_frame",
            default_value="body",
            description="Frame id used by the FastLIO body-frame scan.",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="camera_init",
            description="FastLIO world frame used for accumulated Patchwork++ scores.",
        ),
        DeclareLaunchArgument(
            "sensor_height",
            default_value="0.55",
            description="Approximate LiDAR height above the ground for Patchwork++.",
        ),
        DeclareLaunchArgument(
            "enable_world_accumulation",
            default_value="true",
            description="Accumulate repeated ground/non-ground votes in the FastLIO world frame.",
        ),
        DeclareLaunchArgument(
            "score_voxel_size",
            default_value="0.10",
            description="Voxel size for world-frame score accumulation, in meters.",
        ),
        DeclareLaunchArgument(
            "ground_probability_threshold",
            default_value="0.5",
            description="Probability threshold used to split the accumulated world map into ground/nonground.",
        ),
        DeclareLaunchArgument(
            "min_observations_per_voxel",
            default_value="2",
            description="Minimum total observations before a world voxel participates in final outputs.",
        ),
        DeclareLaunchArgument(
            "publish_world_map_every_n_scans",
            default_value="5",
            description="Publish accumulated world-map outputs every N classified scans.",
        ),
        DeclareLaunchArgument(
            "transform_timeout_sec",
            default_value="0.10",
            description="TF lookup timeout for world-frame accumulation.",
        ),
        DeclareLaunchArgument(
            "min_patchwork_range",
            default_value="1.0",
            description="Minimum range Patchwork++ will consider.",
        ),
        DeclareLaunchArgument(
            "max_patchwork_range",
            default_value="60.0",
            description="Maximum range Patchwork++ will consider.",
        ),
        DeclareLaunchArgument(
            "run_fastlio",
            default_value="false",
            description="Launch FastLIO in the same test launch. Enable this for rosbag replay tests.",
        ),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("pilot_control"), "config", "fastlio_mid360.yaml"
            ]),
            description="FastLIO parameter file used when run_fastlio is true.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Enable ROS sim time for FastLIO/Patchwork nodes during bag replay.",
        ),
        DeclareLaunchArgument(
            "run_patchwork_saver",
            default_value="true",
            description="Launch the Patchwork bundle saver alongside the FastLIO/Patchwork test stack.",
        ),
        DeclareLaunchArgument(
            "patchwork_save_directory",
            default_value="/tmp/patchwork_maps",
            description="Directory where Patchwork world-map bundles are saved.",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value="",
            description="Optional tilt-calibration NPZ used to create corrected Patchwork outputs for F2C.",
        ),
        DeclareLaunchArgument(
            "lidar_pitch_deg",
            default_value="15.0",
            description="Fallback pitch used by patchwork_map_saver when no calibration file is provided.",
        ),
        DeclareLaunchArgument(
            "play_bag",
            default_value="false",
            description="If true, launch rosbag playback using bag_path.",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description="Path to the rosbag to play when play_bag is true.",
        ),
        DeclareLaunchArgument(
            "bag_start_delay_sec",
            default_value="2.0",
            description="Delay before starting rosbag playback so FastLIO subscribers are ready.",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="Playback rate for rosbag replay.",
        ),

        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping_patchwork_test",
            output="screen",
            condition=IfCondition(run_fastlio),
            parameters=[
                fastlio_config,
                {
                    "publish.scan_bodyframe_pub_en": True,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),

        Node(
            package="patchworkpp",
            executable="demo",
            name="patchworkpp_fastlio_body",
            output="screen",
            remappings=[
                ("ground", ground_topic),
                ("nonground", nonground_topic),
                ("world_scores", world_scores_topic),
                ("world_ground", world_ground_topic),
                ("world_nonground", world_nonground_topic),
                ("cloud", "/patchworkpp/debug_input"),
            ],
            parameters=[{
                "cloud_topic": input_topic,
                "frame_id": scan_frame,
                "sensor_height": sensor_height,
                "enable_world_accumulation": enable_world_accumulation,
                "world_frame": world_frame,
                "score_voxel_size": score_voxel_size,
                "ground_probability_threshold": ground_probability_threshold,
                "min_observations_per_voxel": min_observations_per_voxel,
                "publish_world_map_every_n_scans": publish_world_map_every_n_scans,
                "transform_timeout_sec": transform_timeout_sec,
                "num_iter": 3,
                "num_lpr": 20,
                "num_min_pts": 10,
                "th_seeds": 0.3,
                "th_dist": 0.125,
                "th_seeds_v": 0.25,
                "th_dist_v": 0.9,
                "max_range": max_patchwork_range,
                "min_range": min_patchwork_range,
                "uprightness_thr": 0.101,
                "verbose": False,
                "display_time": True,
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="pilot_control",
            executable="patchwork_cloud_colorizer",
            name="patchwork_cloud_colorizer_fastlio_body",
            output="screen",
            parameters=[{
                "ground_topic": ground_topic,
                "nonground_topic": nonground_topic,
                "output_topic": colored_topic,
                "output_frame": scan_frame,
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="pilot_control",
            executable="patchwork_cloud_colorizer",
            name="patchwork_world_cloud_colorizer_fastlio_body",
            output="screen",
            condition=IfCondition(enable_world_accumulation),
            parameters=[{
                "ground_topic": world_ground_topic,
                "nonground_topic": world_nonground_topic,
                "output_topic": world_colored_topic,
                "output_frame": world_frame,
                "max_stamp_skew_ms": 1000,
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="pilot_control",
            executable="patchwork_map_saver",
            name="patchwork_map_saver_fastlio_body",
            output="screen",
            condition=IfCondition(run_patchwork_saver),
            parameters=[{
                "scores_topic": world_scores_topic,
                "ground_topic": world_ground_topic,
                "nonground_topic": world_nonground_topic,
                "save_directory": patchwork_save_directory,
                "publish_world_map_service": "/publish_world_map_once",
                "calibration_file": calibration_file,
                "lidar_pitch_deg": lidar_pitch_deg,
                "use_sim_time": use_sim_time,
            }],
        ),

        TimerAction(
            period=bag_start_delay_sec,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate],
                    output="screen",
                    condition=IfCondition(play_bag),
                ),
            ],
        ),
    ])
