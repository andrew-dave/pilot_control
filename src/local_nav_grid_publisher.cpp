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
#include <deque>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

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
constexpr double kDefaultRollingHorizonSec = 0.75;
constexpr double kDefaultMaxPublishRateHz = 10.0;
constexpr double kDefaultPrimaryActiveTimeoutSec = 0.50;
constexpr double kPi = 3.14159265358979323846;

struct Point3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct BufferedScan {
    std::chrono::steady_clock::time_point received_tp;
    std::vector<Point3f> corrected_points;
};

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
    const auto now_ticks = std::chrono::steady_clock::now().time_since_epoch().count();
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
        RCLCPP_WARN(logger,
                    "Failed to parse extracted calibration matrices from %s",
                    npz_file.c_str());
    }
    return ok;
}

}  // namespace

class LocalNavGridPublisher : public rclcpp::Node {
public:
    LocalNavGridPublisher()
        : Node("local_nav_grid_publisher") {
        this->declare_parameter<std::string>("primary_input_topic", "/cloud_registered");
        this->declare_parameter<std::string>("fallback_input_topic", "/Laser_map");
        this->declare_parameter<std::string>("corrected_odometry_topic",
                                             "/Odometry_tilt_corrected_diff");
        this->declare_parameter<std::string>("output_topic", "/local_nav_grid");
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<double>("lidar_pitch_deg", kFallbackPitchDeg);
        this->declare_parameter<double>("rolling_horizon_sec", kDefaultRollingHorizonSec);
        this->declare_parameter<double>("max_publish_rate_hz", kDefaultMaxPublishRateHz);
        this->declare_parameter<double>("primary_active_timeout_sec",
                                        kDefaultPrimaryActiveTimeoutSec);

        primary_input_topic_ = this->get_parameter("primary_input_topic").as_string();
        fallback_input_topic_ = this->get_parameter("fallback_input_topic").as_string();
        corrected_odometry_topic_ =
            this->get_parameter("corrected_odometry_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        lidar_pitch_deg_ = this->get_parameter("lidar_pitch_deg").as_double();
        rolling_horizon_sec_ = this->get_parameter("rolling_horizon_sec").as_double();
        max_publish_rate_hz_ = this->get_parameter("max_publish_rate_hz").as_double();
        primary_active_timeout_sec_ =
            this->get_parameter("primary_active_timeout_sec").as_double();

        if (!(rolling_horizon_sec_ > 0.0)) {
            rolling_horizon_sec_ = kDefaultRollingHorizonSec;
        }
        if (!(max_publish_rate_hz_ > 0.0)) {
            max_publish_rate_hz_ = kDefaultMaxPublishRateHz;
        }
        if (!(primary_active_timeout_sec_ > 0.0)) {
            primary_active_timeout_sec_ = kDefaultPrimaryActiveTimeoutSec;
        }

        rolling_horizon_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(rolling_horizon_sec_));
        min_publish_period_ =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / max_publish_rate_hz_));
        primary_active_timeout_ =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(primary_active_timeout_sec_));

        loadTiltCorrection();

        rclcpp::QoS cloud_qos(rclcpp::KeepLast(20));
        cloud_qos.reliable();
        primary_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            primary_input_topic_,
            cloud_qos,
            std::bind(&LocalNavGridPublisher::onPrimaryCloud, this, std::placeholders::_1));
        fallback_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            fallback_input_topic_,
            cloud_qos,
            std::bind(&LocalNavGridPublisher::onFallbackCloud, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            corrected_odometry_topic_,
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
            std::bind(&LocalNavGridPublisher::onOdom, this, std::placeholders::_1));

        rclcpp::QoS grid_qos(rclcpp::KeepLast(1));
        grid_qos.best_effort();
        grid_pub_ =
            this->create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic_, grid_qos);

        RCLCPP_INFO(this->get_logger(), "Local nav grid publisher started");
        RCLCPP_INFO(this->get_logger(), "  primary cloud:   %s", primary_input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  fallback cloud:  %s", fallback_input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  odom input:      %s",
                    corrected_odometry_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  output:          %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  grid:            %dx%d (%.4f m/cell, %d bytes)",
                    kGridCols,
                    kGridRows,
                    kCellSizeM,
                    kGridBytes);
        RCLCPP_INFO(this->get_logger(), "  window:          %.1f m x %.1f m",
                    kWindowSizeM,
                    kWindowSizeM);
        RCLCPP_INFO(this->get_logger(), "  z slice:         [%.2f, %.2f] m",
                    -kHalfHeightSliceM,
                    kHalfHeightSliceM);
        RCLCPP_INFO(this->get_logger(), "  rolling horizon: %.2f s", rolling_horizon_sec_);
        RCLCPP_INFO(this->get_logger(), "  max publish:     %.1f Hz", max_publish_rate_hz_);
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

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
            return;
        }
        latest_robot_position_.x() = msg->pose.pose.position.x;
        latest_robot_position_.y() = msg->pose.pose.position.y;
        latest_robot_position_.z() = msg->pose.pose.position.z;
        odom_ready_ = true;
    }

    void onPrimaryCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        last_primary_rx_tp_ = now;
        primary_received_ = true;

        BufferedScan buffered_scan;
        buffered_scan.received_tp = now;
        transformCloudToCorrectedPoints(*msg, buffered_scan.corrected_points);
        primary_scan_buffer_.push_back(std::move(buffered_scan));
        prunePrimaryScanBuffer(now);

        if (!odom_ready_) {
            return;
        }

        std::array<uint8_t, kGridBytes> grid{};
        buildGridFromPrimaryBuffer(grid);
        publishGrid(grid, now);
    }

    void onFallbackCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg) {
            return;
        }

        latest_fallback_cloud_ = msg;
        last_fallback_rx_tp_ = std::chrono::steady_clock::now();

        if (!odom_ready_ || primaryActive(last_fallback_rx_tp_)) {
            return;
        }

        std::array<uint8_t, kGridBytes> grid{};
        buildGridFromFallbackCloud(*msg, grid);
        publishGrid(grid, last_fallback_rx_tp_);
    }

    bool publishDue(const std::chrono::steady_clock::time_point& now) const {
        return !published_once_ || (now - last_publish_tp_) >= min_publish_period_;
    }

    bool primaryActive(const std::chrono::steady_clock::time_point& now) const {
        return primary_received_ && ((now - last_primary_rx_tp_) <= primary_active_timeout_);
    }

    void prunePrimaryScanBuffer(const std::chrono::steady_clock::time_point& now) {
        while (!primary_scan_buffer_.empty() &&
               (now - primary_scan_buffer_.front().received_tp) > rolling_horizon_) {
            primary_scan_buffer_.pop_front();
        }
    }

    void transformCloudToCorrectedPoints(const sensor_msgs::msg::PointCloud2& cloud,
                                         std::vector<Point3f>& corrected_points) const {
        corrected_points.clear();
        if (cloud.data.empty() || cloud.point_step == 0) {
            return;
        }

        corrected_points.reserve(cloud.data.size() / cloud.point_step);

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

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

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
                if (!std::isfinite(corrected_x) || !std::isfinite(corrected_y) ||
                    !std::isfinite(corrected_z)) {
                    continue;
                }

                corrected_points.push_back(Point3f{
                    static_cast<float>(corrected_x),
                    static_cast<float>(corrected_y),
                    static_cast<float>(corrected_z)});
            }
        } catch (const std::runtime_error& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot iterate primary cloud fields for local nav grid: %s",
                        ex.what());
            corrected_points.clear();
        }
    }

    void rasterizeCorrectedPoint(float corrected_x,
                                 float corrected_y,
                                 float corrected_z,
                                 std::array<uint8_t, kGridBytes>& grid) const {
        const double dx = static_cast<double>(corrected_x) - latest_robot_position_.x();
        const double dy = static_cast<double>(corrected_y) - latest_robot_position_.y();
        const double dz = static_cast<double>(corrected_z) - latest_robot_position_.z();

        if (std::abs(dz) > kHalfHeightSliceM ||
            std::abs(dx) > kHalfWindowM ||
            std::abs(dy) > kHalfWindowM) {
            return;
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

    void buildGridFromPrimaryBuffer(std::array<uint8_t, kGridBytes>& grid) const {
        for (const auto& scan : primary_scan_buffer_) {
            for (const auto& point : scan.corrected_points) {
                rasterizeCorrectedPoint(point.x, point.y, point.z, grid);
            }
        }
    }

    void buildGridFromFallbackCloud(const sensor_msgs::msg::PointCloud2& cloud,
                                    std::array<uint8_t, kGridBytes>& grid) const {
        if (cloud.data.empty() || cloud.point_step == 0) {
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

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

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
                if (!std::isfinite(corrected_x) || !std::isfinite(corrected_y) ||
                    !std::isfinite(corrected_z)) {
                    continue;
                }

                rasterizeCorrectedPoint(static_cast<float>(corrected_x),
                                        static_cast<float>(corrected_y),
                                        static_cast<float>(corrected_z),
                                        grid);
            }
        } catch (const std::runtime_error& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot iterate fallback cloud fields for local nav grid: %s",
                        ex.what());
        }
    }

    void publishGrid(const std::array<uint8_t, kGridBytes>& grid,
                     const std::chrono::steady_clock::time_point& now) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data.assign(grid.begin(), grid.end());
        grid_pub_->publish(msg);
        published_once_ = true;
        last_publish_tp_ = now;
    }

    std::string primary_input_topic_;
    std::string fallback_input_topic_;
    std::string corrected_odometry_topic_;
    std::string output_topic_;
    std::string calibration_file_;
    double lidar_pitch_deg_ = kFallbackPitchDeg;
    double rolling_horizon_sec_ = kDefaultRollingHorizonSec;
    double max_publish_rate_hz_ = kDefaultMaxPublishRateHz;
    double primary_active_timeout_sec_ = kDefaultPrimaryActiveTimeoutSec;

    Eigen::Matrix3d r_map_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p0_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_robot_position_ = Eigen::Vector3d::Zero();
    bool odom_ready_ = false;

    std::chrono::steady_clock::duration rolling_horizon_{};
    std::chrono::steady_clock::duration min_publish_period_{};
    std::chrono::steady_clock::duration primary_active_timeout_{};

    std::deque<BufferedScan> primary_scan_buffer_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_fallback_cloud_;
    std::chrono::steady_clock::time_point last_primary_rx_tp_{};
    std::chrono::steady_clock::time_point last_fallback_rx_tp_{};
    std::chrono::steady_clock::time_point last_publish_tp_{};
    bool primary_received_ = false;
    bool published_once_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fallback_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr grid_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalNavGridPublisher>());
    rclcpp::shutdown();
    return 0;
}
