#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <cmath>
#include <string>

class LivoxCustomMsgToCloud : public rclcpp::Node {
public:
    LivoxCustomMsgToCloud() : Node("livox_custommsg_to_cloud") {
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/patchworkpp/input");
        this->declare_parameter<std::string>("output_frame", "foot");
        this->declare_parameter<bool>("apply_pitch_correction", true);
        this->declare_parameter<double>("pitch_deg", 15.0);
        this->declare_parameter<double>("min_range_m", 0.35);
        this->declare_parameter<int>("point_stride", 1);
        this->declare_parameter<bool>("use_tag_filter", true);
        // Gravity levelling: when enabled, ignore the fixed pitch and instead
        // rotate every cloud by the live IMU-derived levelling rotation published
        // by odom_tilt_corrector on `leveling_topic`. This is the correct,
        // full pitch+roll levelling the odometry + arming gate already use, so
        // Patchwork++ sees a gravity-aligned (z-up), sensor-centred cloud.
        this->declare_parameter<bool>("use_gravity_leveling", false);
        this->declare_parameter<std::string>("leveling_topic", "/lidar_leveling_rotation");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        apply_pitch_correction_ = this->get_parameter("apply_pitch_correction").as_bool();
        pitch_deg_ = this->get_parameter("pitch_deg").as_double();
        min_range_m_ = std::max(0.0, this->get_parameter("min_range_m").as_double());
        point_stride_ = static_cast<int>(std::max<int64_t>(
            1, this->get_parameter("point_stride").as_int()));
        use_tag_filter_ = this->get_parameter("use_tag_filter").as_bool();
        use_gravity_leveling_ = this->get_parameter("use_gravity_leveling").as_bool();
        leveling_topic_ = this->get_parameter("leveling_topic").as_string();

        updatePitchCorrection();

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::SensorDataQoS());

        cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&LivoxCustomMsgToCloud::cloudCallback, this, std::placeholders::_1));

        if (use_gravity_leveling_) {
            // Match the corrector's latched publisher QoS (transient-local) so we
            // still receive the rotation even if leveling latched before we start.
            rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
            latched_qos.reliable().transient_local();
            leveling_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
                leveling_topic_, latched_qos,
                std::bind(&LivoxCustomMsgToCloud::levelingCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(), "LivoxCustomMsgToCloud started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output frame: %s",
                    output_frame_.empty() ? "<preserve input frame>" : output_frame_.c_str());
        if (use_gravity_leveling_) {
            RCLCPP_INFO(this->get_logger(),
                        "Leveling: GRAVITY (live IMU rotation from '%s'); fixed pitch ignored",
                        leveling_topic_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Leveling: fixed pitch %s (pitch=%.2f deg)",
                        apply_pitch_correction_ ? "enabled" : "disabled", pitch_deg_);
        }
        RCLCPP_INFO(this->get_logger(), "Minimum range: %.2f m | stride: %d | tag filter: %s",
                    min_range_m_, point_stride_, use_tag_filter_ ? "enabled" : "disabled");
    }

private:
    void updatePitchCorrection() {
        if (!apply_pitch_correction_) {
            pitch_correction_ = Eigen::Matrix3d::Identity();
            return;
        }

        // Positive pitch_deg means the LiDAR is tilted forward; apply the inverse.
        const double pitch_rad = -pitch_deg_ * M_PI / 180.0;
        const double cp = std::cos(pitch_rad);
        const double sp = std::sin(pitch_rad);

        pitch_correction_ << cp, 0.0, sp,
                            0.0, 1.0, 0.0,
                           -sp, 0.0, cp;
    }

    void levelingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        const auto& q = msg->quaternion;
        Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
        const double n = eq.norm();
        if (n < 1e-9) {
            RCLCPP_WARN(this->get_logger(), "Ignoring degenerate leveling quaternion");
            return;
        }
        eq.normalize();
        leveling_rotation_ = eq.toRotationMatrix();
        if (!leveling_received_) {
            leveling_received_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Received gravity leveling rotation; publishing leveled clouds.");
        }
    }

    bool keepPoint(const livox_ros_driver2::msg::CustomPoint& pt, size_t index) const {
        if ((index % static_cast<size_t>(point_stride_)) != 0u) {
            return false;
        }

        if (use_tag_filter_) {
            const uint8_t return_group = static_cast<uint8_t>(pt.tag & 0x30);
            if (!(return_group == 0x10 || return_group == 0x00)) {
                return false;
            }
        }

        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            return false;
        }

        const double range = std::sqrt(
            static_cast<double>(pt.x) * static_cast<double>(pt.x) +
            static_cast<double>(pt.y) * static_cast<double>(pt.y) +
            static_cast<double>(pt.z) * static_cast<double>(pt.z));
        return range >= min_range_m_;
    }

    void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        // In gravity mode, hold off publishing until the levelling rotation is
        // received. Emitting an un-levelled (tilted) cloud would feed Patchwork++
        // a skewed ground plane and defeats the point of gravity levelling.
        if (use_gravity_leveling_ && !leveling_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for gravity leveling rotation on '%s' (odom_tilt_corrector not latched yet)...",
                leveling_topic_.c_str());
            return;
        }

        const bool do_rotate = use_gravity_leveling_ || apply_pitch_correction_;
        const Eigen::Matrix3d& rot = use_gravity_leveling_ ? leveling_rotation_ : pitch_correction_;

        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(msg->points.size());

        for (size_t i = 0; i < msg->points.size(); ++i) {
            const auto& pt = msg->points[i];
            if (!keepPoint(pt, i)) {
                continue;
            }

            Eigen::Vector3d corrected(
                static_cast<double>(pt.x),
                static_cast<double>(pt.y),
                static_cast<double>(pt.z));
            if (do_rotate) {
                corrected = rot * corrected;
            }

            pcl::PointXYZI out_pt;
            out_pt.x = static_cast<float>(corrected.x());
            out_pt.y = static_cast<float>(corrected.y());
            out_pt.z = static_cast<float>(corrected.z());
            out_pt.intensity = static_cast<float>(pt.reflectivity);
            cloud.push_back(out_pt);
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header = msg->header;
        if (!output_frame_.empty()) {
            output_msg.header.frame_id = output_frame_;
        }
        cloud_pub_->publish(output_msg);

        messages_processed_++;
        if ((messages_processed_ % 50) == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Converted %zu Livox clouds | input points: %zu | published points: %zu",
                        messages_processed_, msg->points.size(), cloud.size());
        }
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr leveling_sub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_;
    bool apply_pitch_correction_ = true;
    double pitch_deg_ = 15.0;
    double min_range_m_ = 0.35;
    int point_stride_ = 1;
    bool use_tag_filter_ = true;
    bool use_gravity_leveling_ = false;
    std::string leveling_topic_;
    Eigen::Matrix3d pitch_correction_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d leveling_rotation_ = Eigen::Matrix3d::Identity();
    bool leveling_received_ = false;
    size_t messages_processed_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxCustomMsgToCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
