#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr int kGridRows = 48;
constexpr int kGridCols = 48;
constexpr int kGridBits = kGridRows * kGridCols;
constexpr int kGridBytes = kGridBits / 8;
constexpr double kWindowSizeM = 3.0;
constexpr double kHalfWindowM = kWindowSizeM * 0.5;
constexpr double kCellSizeM = kWindowSizeM / static_cast<double>(kGridCols);
constexpr double kHalfHeightSliceM = 0.10;
constexpr double kFallbackPitchDeg = 15.0;
constexpr double kPi = 3.14159265358979323846;
constexpr auto kBuildPeriod = std::chrono::milliseconds(500);
constexpr auto kKeepalivePeriod = std::chrono::seconds(2);

Eigen::Matrix3d buildPitchRotation(double pitch_rad) {
    const double cp = std::cos(pitch_rad);
    const double sp = std::sin(pitch_rad);
    Eigen::Matrix3d rotation;
    rotation << cp, 0.0, sp,
                0.0, 1.0, 0.0,
                -sp, 0.0, cp;
    return rotation;
}

std::string shellEscapeSingleQuoted(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size() + 8);
    for (char ch : text) {
        if (ch == '\'') {
            escaped += "'\\''";
        } else {
            escaped.push_back(ch);
        }
    }
    return escaped;
}

std::string makeTempPath(const std::string& stem) {
    const auto now_ticks =
        std::chrono::steady_clock::now().time_since_epoch().count();
    return (std::filesystem::temp_directory_path() /
            (stem + "_" + std::to_string(now_ticks) + ".txt"))
        .string();
}

bool loadMatrixFromText(const std::string& path, Eigen::Matrix3d& matrix) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            file >> matrix(row, col);
            if (!file.good()) {
                return false;
            }
        }
    }
    return true;
}

bool loadVectorFromText(const std::string& path, Eigen::Vector3d& vector) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        file >> vector(index);
        if (!file.good()) {
            return false;
        }
    }
    return true;
}

bool extractCalibrationMatrices(const std::string& npz_file,
                                Eigen::Matrix3d& r_map,
                                Eigen::Vector3d& p0_world,
                                const rclcpp::Logger& logger) {
    const std::string r_map_file = makeTempPath("local_nav_r_map");
    const std::string p0_world_file = makeTempPath("local_nav_p0_world");
    const std::string cmd =
        "python3 -c \"import numpy as np; "
        "d=np.load('" + shellEscapeSingleQuoted(npz_file) + "'); "
        "np.savetxt('" + shellEscapeSingleQuoted(r_map_file) + "', d['R_map']); "
        "np.savetxt('" + shellEscapeSingleQuoted(p0_world_file) + "', d['p0_world'])\" "
        ">/dev/null 2>&1";

    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        RCLCPP_WARN(logger, "Failed to extract calibration from %s", npz_file.c_str());
        std::filesystem::remove(r_map_file);
        std::filesystem::remove(p0_world_file);
        return false;
    }

    const bool ok = loadMatrixFromText(r_map_file, r_map) &&
                    loadVectorFromText(p0_world_file, p0_world);
    std::filesystem::remove(r_map_file);
    std::filesystem::remove(p0_world_file);
    if (!ok) {
        RCLCPP_WARN(logger, "Failed to parse extracted calibration matrices from %s", npz_file.c_str());
    }
    return ok;
}

}  // namespace

class LocalNavGridPublisher : public rclcpp::Node {
public:
    LocalNavGridPublisher()
        : Node("local_nav_grid_publisher") {
        this->declare_parameter<std::string>("input_topic", "/Laser_map");
        this->declare_parameter<std::string>("corrected_odometry_topic", "/Odometry_tilt_corrected_diff");
        this->declare_parameter<std::string>("output_topic", "/local_nav_grid");
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<double>("lidar_pitch_deg", kFallbackPitchDeg);

        input_topic_ = this->get_parameter("input_topic").as_string();
        corrected_odometry_topic_ = this->get_parameter("corrected_odometry_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        lidar_pitch_deg_ = this->get_parameter("lidar_pitch_deg").as_double();

        loadTiltCorrection();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&LocalNavGridPublisher::onCloud, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            corrected_odometry_topic_,
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
            std::bind(&LocalNavGridPublisher::onOdom, this, std::placeholders::_1));

        rclcpp::QoS grid_qos(rclcpp::KeepLast(1));
        grid_qos.best_effort();
        grid_pub_ =
            this->create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic_, grid_qos);

        build_timer_ = this->create_wall_timer(
            kBuildPeriod, std::bind(&LocalNavGridPublisher::onBuildTimer, this));

        RCLCPP_INFO(this->get_logger(), "Local nav grid publisher started");
        RCLCPP_INFO(this->get_logger(), "  cloud input: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  odom input:  %s", corrected_odometry_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  output:      %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  grid:        %dx%d (%.4f m/cell, %d bytes)",
                    kGridCols,
                    kGridRows,
                    kCellSizeM,
                    kGridBytes);
        RCLCPP_INFO(this->get_logger(), "  window:      %.1f m x %.1f m", kWindowSizeM, kWindowSizeM);
        RCLCPP_INFO(this->get_logger(), "  z slice:     [%.2f, %.2f] m around corrected LiDAR/body origin",
                    -kHalfHeightSliceM,
                    kHalfHeightSliceM);
    }

private:
    void loadTiltCorrection() {
        if (!calibration_file_.empty() &&
            extractCalibrationMatrices(calibration_file_, r_map_, p0_world_, this->get_logger())) {
            RCLCPP_INFO(this->get_logger(),
                        "Loaded tilt calibration from %s",
                        calibration_file_.c_str());
            return;
        }

        const double pitch_rad = lidar_pitch_deg_ * kPi / 180.0;
        r_map_ = buildPitchRotation(-pitch_rad);
        p0_world_ = Eigen::Vector3d::Zero();
        RCLCPP_WARN(this->get_logger(),
                    "Using fixed %.1f deg pitch correction for local nav grid",
                    lidar_pitch_deg_);
    }

    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_cloud_ = msg;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
            return;
        }
        latest_robot_position_.x() = msg->pose.pose.position.x;
        latest_robot_position_.y() = msg->pose.pose.position.y;
        latest_robot_position_.z() = msg->pose.pose.position.z;
        odom_ready_ = true;
    }

    void onBuildTimer() {
        if (!odom_ready_ || !latest_cloud_) {
            return;
        }

        std::array<uint8_t, kGridBytes> next_grid{};
        buildGridFromLatestCloud(next_grid);

        const auto now = std::chrono::steady_clock::now();
        const bool keepalive_due =
            !published_once_ || (now - last_publish_tp_) >= kKeepalivePeriod;
        if (published_once_ && next_grid == last_grid_ && !keepalive_due) {
            return;
        }

        std_msgs::msg::UInt8MultiArray msg;
        msg.data.assign(next_grid.begin(), next_grid.end());
        grid_pub_->publish(msg);

        last_grid_ = next_grid;
        published_once_ = true;
        last_publish_tp_ = now;
    }

    void buildGridFromLatestCloud(std::array<uint8_t, kGridBytes>& grid) {
        const auto cloud = latest_cloud_;
        if (!cloud || cloud->data.empty() || cloud->point_step == 0) {
            return;
        }

        const double r00 = r_map_(0, 0);
        const double r01 = r_map_(0, 1);
        const double r02 = r_map_(0, 2);
        const double r10 = r_map_(1, 0);
        const double r11 = r_map_(1, 1);
        const double r12 = r_map_(1, 2);
        const double r20 = r_map_(2, 0);
        const double r21 = r_map_(2, 1);
        const double r22 = r_map_(2, 2);
        const double p0x = p0_world_.x();
        const double p0y = p0_world_.y();
        const double p0z = p0_world_.z();
        const double robot_x = latest_robot_position_.x();
        const double robot_y = latest_robot_position_.y();
        const double robot_z = latest_robot_position_.z();

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                const double raw_x = static_cast<double>(*iter_x);
                const double raw_y = static_cast<double>(*iter_y);
                const double raw_z = static_cast<double>(*iter_z);
                if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_z)) {
                    continue;
                }

                const double centered_x = raw_x - p0x;
                const double centered_y = raw_y - p0y;
                const double centered_z = raw_z - p0z;

                const double corrected_x =
                    (r00 * centered_x) + (r01 * centered_y) + (r02 * centered_z);
                const double corrected_y =
                    (r10 * centered_x) + (r11 * centered_y) + (r12 * centered_z);
                const double corrected_z =
                    (r20 * centered_x) + (r21 * centered_y) + (r22 * centered_z);

                const double dx = corrected_x - robot_x;
                const double dy = corrected_y - robot_y;
                const double dz = corrected_z - robot_z;

                if (std::abs(dz) > kHalfHeightSliceM ||
                    std::abs(dx) > kHalfWindowM ||
                    std::abs(dy) > kHalfWindowM) {
                    continue;
                }

                // Screen up aligns with corrected +X, and screen left aligns with corrected +Y.
                const int row = std::clamp(
                    static_cast<int>((kHalfWindowM - dx) / kCellSizeM), 0, kGridRows - 1);
                const int col = std::clamp(
                    static_cast<int>((kHalfWindowM - dy) / kCellSizeM), 0, kGridCols - 1);
                const int bit_index = (row * kGridCols) + col;
                grid[static_cast<size_t>(bit_index >> 3)] |=
                    static_cast<uint8_t>(1u << (bit_index & 7));
            }
        } catch (const std::runtime_error& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "Cannot iterate Laser_map cloud fields: %s",
                                 ex.what());
        }
    }

    std::string input_topic_;
    std::string corrected_odometry_topic_;
    std::string output_topic_;
    std::string calibration_file_;
    double lidar_pitch_deg_ = kFallbackPitchDeg;

    Eigen::Matrix3d r_map_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p0_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_robot_position_ = Eigen::Vector3d::Zero();
    bool odom_ready_ = false;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr build_timer_;

    std::array<uint8_t, kGridBytes> last_grid_{};
    bool published_once_ = false;
    std::chrono::steady_clock::time_point last_publish_tp_{};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalNavGridPublisher>());
    rclcpp::shutdown();
    return 0;
}
