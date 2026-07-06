"""Patchwork++ ground-segmentation check (gravity-leveled input).

Runs ON TOP of robot_complete.launch.py. It reuses the live Livox driver
(/livox/lidar CustomMsg) and the gravity-levelling rotation published by
odom_tilt_corrector (/lidar_leveling_rotation), so the cloud fed to
Patchwork++ is levelled off the exact same IMU gravity vector as the
odometry and the motor-arming gate -- no fixed 15 deg mount-pitch guess.

Pipeline:
  /livox/lidar (CustomMsg)
    -> livox_custommsg_to_cloud   (tag-filtered, gravity-leveled PointCloud2,
                                    frame 'livox_gravity')
    -> patchworkpp_node           (ground segmentation, base_frame livox_gravity)
    -> /patchworkpp/ground , /patchworkpp/nonground
    -> patchwork_cloud_colorizer  (/patchworkpp/colored for RViz)

RViz: set Fixed Frame = livox_gravity, add /patchworkpp/colored (+ ground /
nonground). The robot sits at the origin, z is gravity-up.

NOTE: the levelling rotation only latches once odom_tilt_corrector has a quiet
stationary IMU window, so the converter stays silent (throttled warning) until
the robot has been still for ~2 s after robot_complete start.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    converted_topic = "/patchworkpp/input"
    scan_frame = "livox_gravity"

    livox_topic = LaunchConfiguration("livox_topic")
    leveling_topic = LaunchConfiguration("leveling_topic")
    sensor_height = LaunchConfiguration("sensor_height")
    uprightness_thr = LaunchConfiguration("uprightness_thr")
    min_range_m = LaunchConfiguration("min_range_m")
    use_tag_filter = LaunchConfiguration("use_tag_filter")

    return LaunchDescription([
        DeclareLaunchArgument(
            "livox_topic", default_value="/livox/lidar",
            description="Live Livox CustomMsg topic from the driver.",
        ),
        DeclareLaunchArgument(
            "leveling_topic", default_value="/lidar_leveling_rotation",
            description="Latched gravity-levelling rotation from odom_tilt_corrector.",
        ),
        DeclareLaunchArgument(
            "sensor_height", default_value="0.23",
            description="LiDAR origin height above the ground (m).",
        ),
        DeclareLaunchArgument(
            "uprightness_thr", default_value="0.3",
            description="Patchwork++ ground-normal uprightness threshold "
                        "(raised off 0.101 so near-vertical parapet faces are "
                        "not accepted as ground).",
        ),
        DeclareLaunchArgument(
            "min_range_m", default_value="0.35",
            description="Minimum accepted Livox point range (m).",
        ),
        DeclareLaunchArgument(
            "use_tag_filter", default_value="true",
            description="Drop noisy/low-confidence Livox returns via the return-tag filter.",
        ),

        # Livox CustomMsg -> gravity-leveled PointCloud2 (sensor-centred, z-up)
        Node(
            package="pilot_control",
            executable="livox_custommsg_to_cloud",
            name="livox_custommsg_to_cloud_patchwork",
            output="screen",
            parameters=[{
                "input_topic": livox_topic,
                "output_topic": converted_topic,
                "output_frame": scan_frame,
                "use_gravity_leveling": True,
                "leveling_topic": leveling_topic,
                "apply_pitch_correction": False,
                "min_range_m": min_range_m,
                "point_stride": 1,
                "use_tag_filter": use_tag_filter,
            }],
        ),

        # Upstream Patchwork++ ground segmentation
        Node(
            package="patchworkpp",
            executable="patchworkpp_node",
            name="patchworkpp_node",
            output="screen",
            remappings=[("pointcloud_topic", converted_topic)],
            parameters=[
                {
                    "base_frame": scan_frame,
                    "use_sim_time": False,
                    "sensor_height": sensor_height,
                    "num_iter": 3,
                    "num_lpr": 20,
                    "num_min_pts": 10,
                    "th_seeds": 0.3,
                    "th_dist": 0.125,
                    "th_seeds_v": 0.25,
                    "th_dist_v": 0.9,
                    "max_range": 60.0,
                    "min_range": 1.0,
                    "uprightness_thr": uprightness_thr,
                    "verbose": False,
                },
                {"algorithm": "patchworkpp"},
            ],
        ),

        # Colorize ground + non-ground into one cloud for RViz
        Node(
            package="pilot_control",
            executable="patchwork_cloud_colorizer",
            name="patchwork_cloud_colorizer_check",
            output="screen",
            parameters=[{
                "ground_topic": "/patchworkpp/ground",
                "nonground_topic": "/patchworkpp/nonground",
                "output_topic": "/patchworkpp/colored",
                "output_frame": scan_frame,
            }],
        ),
    ])
