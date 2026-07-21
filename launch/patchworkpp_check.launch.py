"""Patchwork++ ground-segmentation check (gravity-leveled input, robot_init).

Runs ON TOP of robot_complete.launch.py. It reuses the live Livox driver
(/livox/lidar CustomMsg) and the gravity-levelling rotation published by
odom_tilt_corrector (/lidar_leveling_rotation), so the cloud fed to
Patchwork++ is levelled off the exact same IMU gravity vector as the
odometry and the motor-arming gate -- no fixed 15 deg mount-pitch guess.

Output frame is 'robot_init' (the levelled robot frame). roof_edge_costmap
(in robot_complete) broadcasts the camera_init->robot_init static TF from the
same corrected-odom leveling, and while the robot is stationary the leveled,
sensor-centred cloud aligns with robot_init on all axes -- so the costmap /
heightmap and this segmentation co-register in one frame, no extra frame.

Pipeline:
  /livox/lidar (CustomMsg)
    -> livox_custommsg_to_cloud   (tag-filtered, gravity-leveled PointCloud2,
                                    frame 'robot_init')
    -> patchworkpp_node           (ground segmentation, base_frame robot_init)
    -> /patchworkpp/ground , /patchworkpp/nonground
    -> patchwork_cloud_colorizer  (/patchworkpp/colored for RViz)

RViz: Fixed Frame = robot_init, add /patchworkpp/colored (+ ground / nonground).

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
    scan_frame = "robot_init"

    livox_topic = LaunchConfiguration("livox_topic")
    leveling_topic = LaunchConfiguration("leveling_topic")
    sensor_height = LaunchConfiguration("sensor_height")
    uprightness_thr = LaunchConfiguration("uprightness_thr")
    th_seeds = LaunchConfiguration("th_seeds")
    th_dist = LaunchConfiguration("th_dist")
    min_range_m = LaunchConfiguration("min_range_m")
    patchwork_min_range = LaunchConfiguration("patchwork_min_range")
    patchwork_max_range = LaunchConfiguration("patchwork_max_range")
    use_tag_filter = LaunchConfiguration("use_tag_filter")
    roi_enable = LaunchConfiguration("roi_enable")
    roi_x_min = LaunchConfiguration("roi_x_min")
    roi_x_max = LaunchConfiguration("roi_x_max")
    roi_y_min = LaunchConfiguration("roi_y_min")
    roi_y_max = LaunchConfiguration("roi_y_max")

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
            "th_seeds", default_value="0.3",
            description="Patchwork++ LPR seed z-band (m). Raise for undulating ground.",
        ),
        DeclareLaunchArgument(
            "th_dist", default_value="0.125",
            description="Patchwork++ ground-plane thickness (m). Primary precision/"
                        "recall knob: raise (0.15-0.2) if real ground is marked nonground.",
        ),
        DeclareLaunchArgument(
            "min_range_m", default_value="0.35",
            description="Minimum accepted Livox point range in the converter (m).",
        ),
        DeclareLaunchArgument(
            "patchwork_min_range", default_value="0.4",
            description="Patchwork++ min ground-estimation range (m). The upstream "
                        "default 1.0 blinds the near ground of a low, short-range "
                        "robot (near cone falls through to nonground); keep just "
                        "under the converter's min_range_m.",
        ),
        DeclareLaunchArgument(
            "patchwork_max_range", default_value="60.0",
            description="Patchwork++ max ground-estimation range (m).",
        ),
        DeclareLaunchArgument(
            "use_tag_filter", default_value="true",
            description="Drop noisy/low-confidence Livox returns via the return-tag filter.",
        ),
        DeclareLaunchArgument(
            "roi_enable", default_value="true",
            description="Restrict the cloud to a forward box in the leveled sensor "
                        "frame (tracks the robot as it turns).",
        ),
        DeclareLaunchArgument(
            "roi_x_min", default_value="0.40",
            description="ROI forward start (m). Small offset skips the near blind cone.",
        ),
        DeclareLaunchArgument(
            "roi_x_max", default_value="2.5",
            description="ROI forward reach (m).",
        ),
        DeclareLaunchArgument(
            "roi_y_min", default_value="-1.0",
            description="ROI right extent (m).",
        ),
        DeclareLaunchArgument(
            "roi_y_max", default_value="1.0",
            description="ROI left extent (m).",
        ),

        # Livox CustomMsg -> gravity-leveled PointCloud2 (sensor-centred, z-up),
        # stamped in robot_init (aligned while stationary).
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
                "roi_enable": roi_enable,
                "roi_x_min": roi_x_min,
                "roi_x_max": roi_x_max,
                "roi_y_min": roi_y_min,
                "roi_y_max": roi_y_max,
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
                    "num_min_pts": 20,
                    "th_seeds": th_seeds,
                    "th_dist": th_dist,
                    "th_seeds_v": 0.25,
                    "th_dist_v": 0.9,
                    "max_range": patchwork_max_range,
                    "min_range": patchwork_min_range,
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
                # ground+nonground come from the same scan but can arrive one
                # 10 Hz frame apart under callback jitter; 120 ms absorbs a
                # single-scan slip so pairs aren't dropped as "out of sync".
                "max_stamp_skew_ms": 120,
            }],
        ),
    ])
