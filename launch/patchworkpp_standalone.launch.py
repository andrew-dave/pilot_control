from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    livox_topic = LaunchConfiguration("livox_topic")
    converted_topic = LaunchConfiguration("converted_topic")
    ground_topic = LaunchConfiguration("ground_topic")
    nonground_topic = LaunchConfiguration("nonground_topic")
    colored_topic = LaunchConfiguration("colored_topic")
    world_scores_topic = LaunchConfiguration("world_scores_topic")
    world_ground_topic = LaunchConfiguration("world_ground_topic")
    world_nonground_topic = LaunchConfiguration("world_nonground_topic")
    world_colored_topic = LaunchConfiguration("world_colored_topic")
    output_frame = LaunchConfiguration("output_frame")
    apply_pitch_correction = LaunchConfiguration("apply_pitch_correction")
    pitch_deg = LaunchConfiguration("pitch_deg")
    min_range_m = LaunchConfiguration("min_range_m")
    point_stride = LaunchConfiguration("point_stride")
    use_tag_filter = LaunchConfiguration("use_tag_filter")
    sensor_height = LaunchConfiguration("sensor_height")
    enable_world_accumulation = LaunchConfiguration("enable_world_accumulation")
    world_frame = LaunchConfiguration("world_frame")
    score_voxel_size = LaunchConfiguration("score_voxel_size")
    ground_probability_threshold = LaunchConfiguration("ground_probability_threshold")
    min_observations_per_voxel = LaunchConfiguration("min_observations_per_voxel")
    publish_world_map_every_n_scans = LaunchConfiguration("publish_world_map_every_n_scans")
    transform_timeout_sec = LaunchConfiguration("transform_timeout_sec")
    min_patchwork_range = LaunchConfiguration("min_patchwork_range")
    max_patchwork_range = LaunchConfiguration("max_patchwork_range")
    visualization_parent_frame = LaunchConfiguration("visualization_parent_frame")
    publish_visualization_tf = LaunchConfiguration("publish_visualization_tf")
    world_parent_frame = LaunchConfiguration("world_parent_frame")
    leveled_world_frame = LaunchConfiguration("leveled_world_frame")
    publish_world_correction_tf = LaunchConfiguration("publish_world_correction_tf")
    play_bag = LaunchConfiguration("play_bag")
    bag_path = LaunchConfiguration("bag_path")
    visualization_pitch_rad = PythonExpression([
        "-(",
        pitch_deg,
        ")*3.141592653589793/180.0",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "livox_topic",
            default_value="/livox/lidar",
            description="Input Livox CustomMsg topic from rosbag or live driver.",
        ),
        DeclareLaunchArgument(
            "converted_topic",
            default_value="/patchworkpp/input",
            description="Leveled PointCloud2 topic published for Patchwork++.",
        ),
        DeclareLaunchArgument(
            "ground_topic",
            default_value="/patchworkpp/ground",
            description="Output topic for Patchwork++ ground points.",
        ),
        DeclareLaunchArgument(
            "nonground_topic",
            default_value="/patchworkpp/nonground",
            description="Output topic for Patchwork++ non-ground points.",
        ),
        DeclareLaunchArgument(
            "colored_topic",
            default_value="/patchworkpp/colored",
            description="Combined colorized PointCloud2 topic for RViz.",
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
            "output_frame",
            default_value="foot",
            description="Frame id for the converted cloud. Set empty to preserve input frame.",
        ),
        DeclareLaunchArgument(
            "apply_pitch_correction",
            default_value="true",
            description="Rotate the raw Livox cloud to compensate for fixed sensor pitch.",
        ),
        DeclareLaunchArgument(
            "pitch_deg",
            default_value="15.0",
            description="Forward pitch of the mounted LiDAR in degrees.",
        ),
        DeclareLaunchArgument(
            "min_range_m",
            default_value="0.35",
            description="Minimum accepted Livox point range in meters.",
        ),
        DeclareLaunchArgument(
            "point_stride",
            default_value="1",
            description="Keep every Nth Livox point before Patchwork++.",
        ),
        DeclareLaunchArgument(
            "use_tag_filter",
            default_value="true",
            description="Use the Livox return-tag validity filter before publishing PointCloud2.",
        ),
        DeclareLaunchArgument(
            "sensor_height",
            default_value="0.55",
            description="Approximate LiDAR height above the ground for Patchwork++.",
        ),
        DeclareLaunchArgument(
            "enable_world_accumulation",
            default_value="false",
            description="Accumulate repeated ground/non-ground votes in a world-frame voxel map. Enable only when an external TF/odometry chain connects the scan frame to world_frame.",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="foot_init",
            description="World frame used for accumulated Patchwork++ scores when a connected TF tree is available.",
        ),
        DeclareLaunchArgument(
            "score_voxel_size",
            default_value="0.10",
            description="Voxel size for world-frame score accumulation, in meters.",
        ),
        DeclareLaunchArgument(
            "ground_probability_threshold",
            default_value="0.5",
            description="Probability threshold used to convert accumulated scores into final ground/nonground voxels.",
        ),
        DeclareLaunchArgument(
            "min_observations_per_voxel",
            default_value="2",
            description="Minimum total observations before a voxel participates in final accumulated outputs.",
        ),
        DeclareLaunchArgument(
            "publish_world_map_every_n_scans",
            default_value="5",
            description="Publish accumulated world-map outputs every N scans.",
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
            "visualization_parent_frame",
            default_value="body",
            description="Parent frame for the static TF published for RViz.",
        ),
        DeclareLaunchArgument(
            "publish_visualization_tf",
            default_value="true",
            description="Publish a robot-style static transform using pitch_deg to make output_frame visible in RViz.",
        ),
        DeclareLaunchArgument(
            "world_parent_frame",
            default_value="camera_init",
            description="Parent frame for the corrected global static TF, matching FastLIO world frame naming.",
        ),
        DeclareLaunchArgument(
            "leveled_world_frame",
            default_value="foot_init",
            description="Child frame for the corrected global static TF, matching the robot pipeline naming.",
        ),
        DeclareLaunchArgument(
            "publish_world_correction_tf",
            default_value="true",
            description="Publish the static camera_init->foot_init-style correction TF for accumulated world outputs.",
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

        Node(
            package="pilot_control",
            executable="livox_custommsg_to_cloud",
            name="livox_custommsg_to_cloud",
            output="screen",
            parameters=[{
                "input_topic": livox_topic,
                "output_topic": converted_topic,
                "output_frame": output_frame,
                "apply_pitch_correction": apply_pitch_correction,
                "pitch_deg": pitch_deg,
                "min_range_m": min_range_m,
                "point_stride": point_stride,
                "use_tag_filter": use_tag_filter,
            }],
        ),

        Node(
            package="patchworkpp",
            executable="demo",
            name="patchworkpp_standalone",
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
                "cloud_topic": converted_topic,
                "frame_id": output_frame,
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
            }],
        ),

        Node(
            package="pilot_control",
            executable="patchwork_cloud_colorizer",
            name="patchwork_cloud_colorizer",
            output="screen",
            parameters=[{
                "ground_topic": ground_topic,
                "nonground_topic": nonground_topic,
                "output_topic": colored_topic,
                "output_frame": output_frame,
            }],
        ),

        Node(
            package="pilot_control",
            executable="patchwork_cloud_colorizer",
            name="patchwork_world_cloud_colorizer",
            output="screen",
            condition=IfCondition(enable_world_accumulation),
            parameters=[{
                "ground_topic": world_ground_topic,
                "nonground_topic": world_nonground_topic,
                "output_topic": world_colored_topic,
                "output_frame": world_frame,
                "max_stamp_skew_ms": 1000,
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="patchworkpp_visualization_tf",
            output="screen",
            condition=IfCondition(publish_visualization_tf),
            arguments=[
                "0", "0", "0",
                "0", visualization_pitch_rad, "0",
                visualization_parent_frame,
                output_frame,
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="patchworkpp_world_correction_tf",
            output="screen",
            condition=IfCondition(publish_world_correction_tf),
            arguments=[
                "0", "0", "0",
                "0", visualization_pitch_rad, "0",
                world_parent_frame,
                leveled_world_frame,
            ],
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag_path],
            output="screen",
            condition=IfCondition(play_bag),
        ),
    ])
