#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

namespace {

std::string sanitizeToken(const std::string& input) {
    std::string output;
    output.reserve(input.size());
    for (char ch : input) {
        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9')) {
            output.push_back(ch);
        } else {
            output.push_back('_');
        }
    }
    if (output.empty()) {
        return "unknown";
    }
    return output;
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
                                Eigen::Vector3d& p0_world) {
    const std::string r_map_file = makeTempPath("patchwork_r_map");
    const std::string p0_world_file = makeTempPath("patchwork_p0_world");
    const std::string cmd =
        "python3 -c \"import numpy as np; "
        "d=np.load('" + shellEscapeSingleQuoted(npz_file) + "'); "
        "np.savetxt('" + shellEscapeSingleQuoted(r_map_file) + "', d['R_map']); "
        "np.savetxt('" + shellEscapeSingleQuoted(p0_world_file) + "', d['p0_world'])\" "
        ">/dev/null 2>&1";

    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::filesystem::remove(r_map_file);
        std::filesystem::remove(p0_world_file);
        return false;
    }

    const bool ok = loadMatrixFromText(r_map_file, r_map) &&
                    loadVectorFromText(p0_world_file, p0_world);
    std::filesystem::remove(r_map_file);
    std::filesystem::remove(p0_world_file);
    return ok;
}

Eigen::Matrix3d buildPitchRotation(double pitch_rad) {
    const double cp = std::cos(pitch_rad);
    const double sp = std::sin(pitch_rad);
    Eigen::Matrix3d rotation;
    rotation << cp, 0.0, sp,
                0.0, 1.0, 0.0,
                -sp, 0.0, cp;
    return rotation;
}

std::string currentTimestampString() {
    const auto now = std::chrono::system_clock::now();
    const auto time_t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream stream;
    stream << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return stream.str();
}

template<typename PointT>
void applyTiltCorrectionToCloud(pcl::PointCloud<PointT>* cloud,
                                const Eigen::Matrix3d& r_map,
                                const Eigen::Vector3d& p0_world) {
    if (cloud == nullptr) {
        return;
    }

    for (auto& point : cloud->points) {
        const Eigen::Vector3d raw(point.x, point.y, point.z);
        const Eigen::Vector3d corrected = r_map * (raw - p0_world);
        point.x = static_cast<float>(corrected.x());
        point.y = static_cast<float>(corrected.y());
        point.z = static_cast<float>(corrected.z());
    }
}

template<typename PointT>
bool saveCloudWithFormat(const std::string& filepath,
                         const pcl::PointCloud<PointT>& cloud,
                         const std::string& save_format,
                         std::string& format_used) {
    try {
        if (save_format == "compressed") {
            pcl::io::savePCDFile(filepath, cloud, true);
            format_used = "compressed PCD (lossless)";
            return true;
        }
        if (save_format == "ascii") {
            pcl::io::savePCDFileASCII(filepath, cloud);
            format_used = "ASCII PCD";
            return true;
        }

        pcl::io::savePCDFileBinary(filepath, cloud);
        format_used = "binary PCD";
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

class PatchworkMapSaver : public rclcpp::Node {
public:
    PatchworkMapSaver() : Node("patchwork_map_saver") {
        this->declare_parameter<std::string>("scores_topic", "/patchworkpp/world_scores");
        this->declare_parameter<std::string>("ground_topic", "/patchworkpp/world_ground");
        this->declare_parameter<std::string>("nonground_topic", "/patchworkpp/world_nonground");
        this->declare_parameter<std::string>("save_directory", "/tmp/patchwork_maps");
        this->declare_parameter<bool>("trigger_publish_before_save", true);
        this->declare_parameter<std::string>("publish_world_map_service", "/publish_world_map_once");
        this->declare_parameter<double>("publish_map_timeout_sec", 5.0);
        this->declare_parameter<double>("cloud_message_timeout_sec", 5.0);
        this->declare_parameter<bool>("apply_tilt_correction", true);
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<double>("lidar_pitch_deg", 15.0);
        this->declare_parameter<std::string>("save_format", "compressed");
        this->declare_parameter<bool>("save_camera_init_copy", true);
        this->declare_parameter<bool>("save_corrected_copy", true);

        scores_topic_ = this->get_parameter("scores_topic").as_string();
        ground_topic_ = this->get_parameter("ground_topic").as_string();
        nonground_topic_ = this->get_parameter("nonground_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        trigger_publish_before_save_ = this->get_parameter("trigger_publish_before_save").as_bool();
        publish_world_map_service_ = this->get_parameter("publish_world_map_service").as_string();
        publish_map_timeout_sec_ = this->get_parameter("publish_map_timeout_sec").as_double();
        cloud_message_timeout_sec_ = this->get_parameter("cloud_message_timeout_sec").as_double();
        apply_tilt_correction_ = this->get_parameter("apply_tilt_correction").as_bool();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        lidar_pitch_deg_ = this->get_parameter("lidar_pitch_deg").as_double();
        save_format_ = this->get_parameter("save_format").as_string();
        save_camera_init_copy_ = this->get_parameter("save_camera_init_copy").as_bool();
        save_corrected_copy_ = this->get_parameter("save_corrected_copy").as_bool();

        std::filesystem::create_directories(save_directory_);

        subscription_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliable();

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = subscription_callback_group_;
        scores_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            scores_topic_, qos, std::bind(&PatchworkMapSaver::scoresCallback, this, std::placeholders::_1), sub_options);
        ground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            ground_topic_, qos, std::bind(&PatchworkMapSaver::groundCallback, this, std::placeholders::_1), sub_options);
        nonground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            nonground_topic_, qos, std::bind(&PatchworkMapSaver::nongroundCallback, this, std::placeholders::_1), sub_options);

        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_patchwork_map",
            std::bind(&PatchworkMapSaver::savePatchworkMapService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);
        set_dir_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/patchwork_map_saver/set_directory",
            std::bind(&PatchworkMapSaver::setDirectoryService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);
        publish_world_map_client_ = this->create_client<std_srvs::srv::Trigger>(
            publish_world_map_service_,
            rmw_qos_profile_services_default,
            client_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Patchwork Map Saver started");
        RCLCPP_INFO(this->get_logger(), "  scores topic:    %s", scores_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ground topic:    %s", ground_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  nonground topic: %s", nonground_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  save directory:  %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "  save raw copy:   %s", save_camera_init_copy_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "  save corrected:  %s", save_corrected_copy_ ? "ENABLED" : "DISABLED");
    }

private:
    void scoresCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_scores_ = msg;
        ++scores_generation_;
        cloud_cv_.notify_all();
    }

    void groundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_ground_ = msg;
        ++ground_generation_;
        cloud_cv_.notify_all();
    }

    void nongroundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_nonground_ = msg;
        ++nonground_generation_;
        cloud_cv_.notify_all();
    }

    bool latestCloudsAvailableLocked() const {
        return latest_scores_ && latest_ground_ && latest_nonground_;
    }

    bool fetchCloudsForSave(sensor_msgs::msg::PointCloud2::SharedPtr& scores_msg,
                            sensor_msgs::msg::PointCloud2::SharedPtr& ground_msg,
                            sensor_msgs::msg::PointCloud2::SharedPtr& nonground_msg,
                            std::string& error_message) {
        if (!trigger_publish_before_save_) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!latestCloudsAvailableLocked()) {
                error_message = "Patchwork world clouds are not available yet";
                return false;
            }
            scores_msg = latest_scores_;
            ground_msg = latest_ground_;
            nonground_msg = latest_nonground_;
            return true;
        }

        if (!publish_world_map_client_->wait_for_service(
                std::chrono::duration<double>(publish_map_timeout_sec_))) {
            error_message = "Patchwork publish service " + publish_world_map_service_ + " is not available";
            return false;
        }

        size_t scores_before = 0;
        size_t ground_before = 0;
        size_t nonground_before = 0;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            scores_before = scores_generation_;
            ground_before = ground_generation_;
            nonground_before = nonground_generation_;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = publish_world_map_client_->async_send_request(request);
        if (future.wait_for(std::chrono::duration<double>(publish_map_timeout_sec_)) !=
            std::future_status::ready) {
            error_message = "Timed out waiting for " + publish_world_map_service_;
            return false;
        }
        auto result = future.get();
        if (!result) {
            error_message = publish_world_map_service_ + " returned no response";
            return false;
        }
        if (!result->success) {
            error_message = publish_world_map_service_ + " failed: " + result->message;
            return false;
        }

        std::unique_lock<std::mutex> lock(cloud_mutex_);
        const bool received_fresh = cloud_cv_.wait_for(
            lock,
            std::chrono::duration<double>(cloud_message_timeout_sec_),
            [this, scores_before, ground_before, nonground_before]() {
                return latest_scores_ && latest_ground_ && latest_nonground_ &&
                       scores_generation_ > scores_before &&
                       ground_generation_ > ground_before &&
                       nonground_generation_ > nonground_before;
            });
        if (!received_fresh) {
            error_message = "Timed out waiting for fresh Patchwork world clouds";
            return false;
        }

        scores_msg = latest_scores_;
        ground_msg = latest_ground_;
        nonground_msg = latest_nonground_;
        return true;
    }

    bool loadTiltCorrection(Eigen::Matrix3d& r_map,
                            Eigen::Vector3d& p0_world,
                            std::string& calibration_used) {
        if (!calibration_file_.empty() && std::filesystem::exists(calibration_file_)) {
            if (extractCalibrationMatrices(calibration_file_, r_map, p0_world)) {
                calibration_used = calibration_file_;
                return true;
            }
            RCLCPP_WARN(this->get_logger(),
                        "Failed to load calibration file %s, falling back to fixed pitch",
                        calibration_file_.c_str());
        }

        calibration_used.clear();
        r_map = buildPitchRotation(-lidar_pitch_deg_ * M_PI / 180.0);
        p0_world = Eigen::Vector3d::Zero();
        return true;
    }

    bool copyCalibrationFileIfPresent(const std::string& timestamp,
                                      std::string& copied_path) {
        copied_path.clear();
        if (calibration_file_.empty() || !std::filesystem::exists(calibration_file_)) {
            return false;
        }

        const std::filesystem::path destination =
            std::filesystem::path(save_directory_) /
            ("patchwork_tilt_calibration_" + timestamp + ".npz");
        try {
            std::filesystem::copy_file(
                calibration_file_,
                destination,
                std::filesystem::copy_options::overwrite_existing);
            copied_path = destination.string();
            return true;
        } catch (const std::exception& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to copy calibration file into save directory: %s",
                        ex.what());
            return false;
        }
    }

    bool saveVariantCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                          const std::string& raw_prefix,
                          const std::string& corrected_prefix,
                          const Eigen::Matrix3d& r_map,
                          const Eigen::Vector3d& p0_world,
                          std::string& raw_output,
                          std::string& corrected_output,
                          size_t& point_count) {
        raw_output.clear();
        corrected_output.clear();
        point_count = 0;

        if (!msg) {
            return false;
        }

        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        point_count = cloud.size();

        if (save_camera_init_copy_) {
            const std::string filepath =
                (std::filesystem::path(save_directory_) / (raw_prefix + ".pcd")).string();
            std::string format_used;
            if (!saveCloudWithFormat(filepath, cloud, save_format_, format_used)) {
                return false;
            }
            raw_output = filepath;
        }

        if (save_corrected_copy_) {
            pcl::PointCloud<pcl::PointXYZI> corrected = cloud;
            if (apply_tilt_correction_) {
                applyTiltCorrectionToCloud(&corrected, r_map, p0_world);
            }
            const std::string filepath =
                (std::filesystem::path(save_directory_) / (corrected_prefix + ".pcd")).string();
            std::string format_used;
            if (!saveCloudWithFormat(filepath, corrected, save_format_, format_used)) {
                return false;
            }
            corrected_output = filepath;
        }

        return true;
    }

    bool writeMetadataFile(const std::string& timestamp,
                           const sensor_msgs::msg::PointCloud2::SharedPtr& scores_msg,
                           const std::string& raw_scores_file,
                           const std::string& raw_ground_file,
                           const std::string& raw_nonground_file,
                           const std::string& corrected_scores_file,
                           const std::string& corrected_ground_file,
                           const std::string& corrected_nonground_file,
                           size_t score_points,
                           size_t ground_points,
                           size_t nonground_points,
                           const std::string& calibration_used,
                           const std::string& copied_calibration_file,
                           std::string& metadata_path) {
        metadata_path =
            (std::filesystem::path(save_directory_) / ("patchwork_map_meta_" + timestamp + ".json")).string();

        const std::string world_frame =
            (scores_msg && !scores_msg->header.frame_id.empty()) ? scores_msg->header.frame_id : "unknown";

        std::ofstream file(metadata_path);
        if (!file.is_open()) {
            metadata_path.clear();
            return false;
        }

        file << "{\n";
        file << "  \"schema_version\": 1,\n";
        file << "  \"world_frame\": \"" << world_frame << "\",\n";
        file << "  \"save_directory\": \"" << save_directory_ << "\",\n";
        file << "  \"scores_topic\": \"" << scores_topic_ << "\",\n";
        file << "  \"ground_topic\": \"" << ground_topic_ << "\",\n";
        file << "  \"nonground_topic\": \"" << nonground_topic_ << "\",\n";
        file << "  \"apply_tilt_correction\": " << (apply_tilt_correction_ ? "true" : "false") << ",\n";
        file << "  \"calibration_file\": \"" << calibration_used << "\",\n";
        file << "  \"copied_calibration_file\": \"" << copied_calibration_file << "\",\n";
        file << "  \"fallback_pitch_deg\": " << lidar_pitch_deg_ << ",\n";
        file << "  \"point_counts\": {\n";
        file << "    \"scores\": " << score_points << ",\n";
        file << "    \"ground\": " << ground_points << ",\n";
        file << "    \"nonground\": " << nonground_points << "\n";
        file << "  },\n";
        file << "  \"files\": {\n";
        file << "    \"raw_scores\": \"" << raw_scores_file << "\",\n";
        file << "    \"raw_ground\": \"" << raw_ground_file << "\",\n";
        file << "    \"raw_nonground\": \"" << raw_nonground_file << "\",\n";
        file << "    \"corrected_scores\": \"" << corrected_scores_file << "\",\n";
        file << "    \"corrected_ground\": \"" << corrected_ground_file << "\",\n";
        file << "    \"corrected_nonground\": \"" << corrected_nonground_file << "\"\n";
        file << "  }\n";
        file << "}\n";
        return true;
    }

    bool saveSnapshot(std::string& status_message) {
        sensor_msgs::msg::PointCloud2::SharedPtr scores_msg;
        sensor_msgs::msg::PointCloud2::SharedPtr ground_msg;
        sensor_msgs::msg::PointCloud2::SharedPtr nonground_msg;
        if (!fetchCloudsForSave(scores_msg, ground_msg, nonground_msg, status_message)) {
            return false;
        }

        const std::string timestamp = currentTimestampString();
        const std::string raw_frame = sanitizeToken(scores_msg->header.frame_id);

        Eigen::Matrix3d r_map = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p0_world = Eigen::Vector3d::Zero();
        std::string calibration_used;
        if (apply_tilt_correction_) {
            loadTiltCorrection(r_map, p0_world, calibration_used);
        }

        std::string raw_scores_file;
        std::string raw_ground_file;
        std::string raw_nonground_file;
        std::string corrected_scores_file;
        std::string corrected_ground_file;
        std::string corrected_nonground_file;
        size_t score_points = 0;
        size_t ground_points = 0;
        size_t nonground_points = 0;

        if (!saveVariantCloud(scores_msg,
                              "patchwork_scores_" + raw_frame + "_" + timestamp,
                              "corrected_patchwork_scores_" + timestamp,
                              r_map, p0_world,
                              raw_scores_file, corrected_scores_file, score_points)) {
            status_message = "Failed to save Patchwork score cloud";
            return false;
        }
        if (!saveVariantCloud(ground_msg,
                              "patchwork_ground_" + raw_frame + "_" + timestamp,
                              "corrected_patchwork_ground_" + timestamp,
                              r_map, p0_world,
                              raw_ground_file, corrected_ground_file, ground_points)) {
            status_message = "Failed to save Patchwork ground cloud";
            return false;
        }
        if (!saveVariantCloud(nonground_msg,
                              "patchwork_nonground_" + raw_frame + "_" + timestamp,
                              "corrected_patchwork_nonground_" + timestamp,
                              r_map, p0_world,
                              raw_nonground_file, corrected_nonground_file, nonground_points)) {
            status_message = "Failed to save Patchwork non-ground cloud";
            return false;
        }

        std::string copied_calibration_file;
        copyCalibrationFileIfPresent(timestamp, copied_calibration_file);

        std::string metadata_path;
        if (!writeMetadataFile(timestamp, scores_msg,
                               raw_scores_file, raw_ground_file, raw_nonground_file,
                               corrected_scores_file, corrected_ground_file, corrected_nonground_file,
                               score_points, ground_points, nonground_points,
                               calibration_used, copied_calibration_file, metadata_path)) {
            status_message = "Failed to write Patchwork metadata sidecar";
            return false;
        }

        std::ostringstream status;
        status << "Saved Patchwork bundle with " << score_points << " score voxels, "
               << ground_points << " ground voxels, and " << nonground_points
               << " non-ground voxels";
        status_message = status.str();
        return true;
    }

    void savePatchworkMapService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response) {
        (void)request;
        response->success = saveSnapshot(response->message);
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void setDirectoryService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                             std_srvs::srv::Trigger::Response::SharedPtr response) {
        (void)request;
        const std::string new_dir = this->get_parameter("save_directory").as_string();
        if (new_dir.empty()) {
            response->success = false;
            response->message = "save_directory parameter is empty";
            return;
        }

        save_directory_ = new_dir;
        std::filesystem::create_directories(save_directory_);
        response->success = true;
        response->message = "Directory set to: " + save_directory_;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scores_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_dir_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr publish_world_map_client_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    std::mutex cloud_mutex_;
    std::condition_variable cloud_cv_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_scores_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_ground_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_nonground_;
    size_t scores_generation_ = 0;
    size_t ground_generation_ = 0;
    size_t nonground_generation_ = 0;

    std::string scores_topic_;
    std::string ground_topic_;
    std::string nonground_topic_;
    std::string save_directory_;
    bool trigger_publish_before_save_ = true;
    std::string publish_world_map_service_;
    double publish_map_timeout_sec_ = 5.0;
    double cloud_message_timeout_sec_ = 5.0;
    bool apply_tilt_correction_ = true;
    std::string calibration_file_;
    double lidar_pitch_deg_ = 15.0;
    std::string save_format_ = "compressed";
    bool save_camera_init_copy_ = true;
    bool save_corrected_copy_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatchworkMapSaver>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
