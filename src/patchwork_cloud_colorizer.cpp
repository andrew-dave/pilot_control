#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <string>

class PatchworkCloudColorizer : public rclcpp::Node {
public:
    PatchworkCloudColorizer() : Node("patchwork_cloud_colorizer") {
        this->declare_parameter<std::string>("ground_topic", "/patchworkpp/ground");
        this->declare_parameter<std::string>("nonground_topic", "/patchworkpp/nonground");
        this->declare_parameter<std::string>("output_topic", "/patchworkpp/colored");
        this->declare_parameter<std::string>("output_frame", "");
        this->declare_parameter<int>("max_stamp_skew_ms", 50);
        this->declare_parameter<int>("ground_r", 60);
        this->declare_parameter<int>("ground_g", 180);
        this->declare_parameter<int>("ground_b", 75);
        this->declare_parameter<int>("nonground_r", 230);
        this->declare_parameter<int>("nonground_g", 65);
        this->declare_parameter<int>("nonground_b", 60);

        ground_topic_ = this->get_parameter("ground_topic").as_string();
        nonground_topic_ = this->get_parameter("nonground_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        max_stamp_skew_ns_ = std::max<int64_t>(
            0, this->get_parameter("max_stamp_skew_ms").as_int()) * 1000000LL;

        ground_rgb_ = {
            clampColor(this->get_parameter("ground_r").as_int()),
            clampColor(this->get_parameter("ground_g").as_int()),
            clampColor(this->get_parameter("ground_b").as_int()),
        };
        nonground_rgb_ = {
            clampColor(this->get_parameter("nonground_r").as_int()),
            clampColor(this->get_parameter("nonground_g").as_int()),
            clampColor(this->get_parameter("nonground_b").as_int()),
        };

        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        pub_qos.reliable();
        pub_qos.durability_volatile();
        colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, pub_qos);

        ground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            ground_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PatchworkCloudColorizer::groundCallback, this, std::placeholders::_1));

        nonground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            nonground_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PatchworkCloudColorizer::nongroundCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PatchworkCloudColorizer started");
        RCLCPP_INFO(this->get_logger(), "Ground topic: %s", ground_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Non-ground topic: %s", nonground_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    }

private:
    struct RgbColor {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    static uint8_t clampColor(int value) {
        return static_cast<uint8_t>(std::max(0, std::min(255, value)));
    }

    void groundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ground_ = msg;
        maybePublishColoredCloud();
    }

    void nongroundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_nonground_ = msg;
        maybePublishColoredCloud();
    }

    static void appendColoredPoints(const pcl::PointCloud<pcl::PointXYZ>& input,
                                    const RgbColor& color,
                                    pcl::PointCloud<pcl::PointXYZRGB>* output) {
        if (!output) {
            return;
        }

        for (const auto& pt : input.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            pcl::PointXYZRGB colored_pt;
            colored_pt.x = pt.x;
            colored_pt.y = pt.y;
            colored_pt.z = pt.z;
            colored_pt.r = color.r;
            colored_pt.g = color.g;
            colored_pt.b = color.b;
            output->push_back(colored_pt);
        }
    }

    void maybePublishColoredCloud() {
        if (!latest_ground_ || !latest_nonground_) {
            return;
        }

        const rclcpp::Time ground_stamp(latest_ground_->header.stamp);
        const rclcpp::Time nonground_stamp(latest_nonground_->header.stamp);
        const int64_t skew_ns =
            std::llabs((ground_stamp - nonground_stamp).nanoseconds());

        if (skew_ns > max_stamp_skew_ns_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Ground/non-ground clouds are out of sync by %.1f ms; waiting for a closer pair.",
                static_cast<double>(skew_ns) / 1e6);
            return;
        }

        if (last_ground_stamp_ns_ == ground_stamp.nanoseconds() &&
            last_nonground_stamp_ns_ == nonground_stamp.nanoseconds()) {
            return;
        }

        if (latest_ground_->header.frame_id != latest_nonground_->header.frame_id) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Ground/non-ground frame mismatch: '%s' vs '%s'. Using ground frame.",
                latest_ground_->header.frame_id.c_str(),
                latest_nonground_->header.frame_id.c_str());
        }

        // Patchwork++ publishes ground/non-ground as XYZ (no intensity field);
        // load as PointXYZ so PCL doesn't spam "Failed to find match for field
        // 'intensity'". Only x/y/z are used for colorizing anyway.
        pcl::PointCloud<pcl::PointXYZ> ground_cloud;
        pcl::PointCloud<pcl::PointXYZ> nonground_cloud;
        pcl::fromROSMsg(*latest_ground_, ground_cloud);
        pcl::fromROSMsg(*latest_nonground_, nonground_cloud);

        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        colored_cloud.reserve(ground_cloud.size() + nonground_cloud.size());
        appendColoredPoints(ground_cloud, ground_rgb_, &colored_cloud);
        appendColoredPoints(nonground_cloud, nonground_rgb_, &colored_cloud);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(colored_cloud, output_msg);
        output_msg.header.stamp = ground_stamp;
        output_msg.header.frame_id =
            output_frame_.empty() ? latest_ground_->header.frame_id : output_frame_;
        colored_pub_->publish(output_msg);

        last_ground_stamp_ns_ = ground_stamp.nanoseconds();
        last_nonground_stamp_ns_ = nonground_stamp.nanoseconds();
        publishes_++;

        if ((publishes_ % 20) == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Published %zu colored Patchwork++ clouds | ground: %zu pts | non-ground: %zu pts",
                        publishes_, ground_cloud.size(), nonground_cloud.size());
        }
    }

    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;

    std::string ground_topic_;
    std::string nonground_topic_;
    std::string output_topic_;
    std::string output_frame_;
    int64_t max_stamp_skew_ns_ = 50000000LL;
    RgbColor ground_rgb_{60, 180, 75};
    RgbColor nonground_rgb_{230, 65, 60};

    sensor_msgs::msg::PointCloud2::SharedPtr latest_ground_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_nonground_;
    int64_t last_ground_stamp_ns_ = std::numeric_limits<int64_t>::min();
    int64_t last_nonground_stamp_ns_ = std::numeric_limits<int64_t>::min();
    size_t publishes_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatchworkCloudColorizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
