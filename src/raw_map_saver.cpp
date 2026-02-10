#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Dense>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstring>

// Simple NPZ file loader for tilt correction matrices
class NPZLoader {
public:
    static bool load_npz_matrices(const std::string& npz_file, 
                                  Eigen::Matrix3d& R_map, 
                                  Eigen::Vector3d& p0_world) {
        std::ifstream file(npz_file, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        // NPZ files are ZIP archives - read the ZIP structure
        // For simplicity, we'll parse the uncompressed numpy format
        // This assumes the .npz was saved with compression=False or we use a ZIP library
        
        // Simple approach: Try to find and parse the raw numpy arrays
        // In practice, .npz files need proper ZIP parsing
        // For now, we'll use a more direct approach with numpy's .npy format
        
        file.close();
        return false; // Will implement proper loader below
    }
};

class RawMapSaver : public rclcpp::Node
{
public:
    RawMapSaver() : Node("raw_map_saver")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/Laser_map");
        this->declare_parameter("save_directory", "/tmp/robot_maps");
        this->declare_parameter("raw_map_filename", "");
        this->declare_parameter("auto_save_enabled", false);
        this->declare_parameter("auto_save_interval_sec", 30.0);
        
        // Tilt correction parameters
        this->declare_parameter("apply_tilt_correction", true);
        this->declare_parameter("save_raw_backup", false);
        this->declare_parameter("calibration_file", "");  // Path to tilt_correction_matrices.npz
        this->declare_parameter("lidar_pitch_deg", 15.0);  // Fallback pitch angle
        
        // Save format parameter for wireless transfer optimization
        this->declare_parameter("save_format", "compressed"); // "binary", "compressed", "ascii"
        
        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        raw_map_filename_ = this->get_parameter("raw_map_filename").as_string();
        auto_save_enabled_ = this->get_parameter("auto_save_enabled").as_bool();
        auto_save_interval_ = this->get_parameter("auto_save_interval_sec").as_double();
        apply_tilt_correction_ = this->get_parameter("apply_tilt_correction").as_bool();
        save_raw_backup_ = this->get_parameter("save_raw_backup").as_bool();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        lidar_pitch_deg_ = this->get_parameter("lidar_pitch_deg").as_double();
        save_format_ = this->get_parameter("save_format").as_string();
        
        // Create save directory
        std::filesystem::create_directories(save_directory_);
        
        // Create subscriber for Laser_map point cloud
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::QoS(10).durability_volatile(),
            std::bind(&RawMapSaver::cloud_callback, this, std::placeholders::_1)
        );
        
        // Create service for manual map saving
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_raw_map",
            std::bind(&RawMapSaver::save_raw_map_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create service for updating save directory (multi-section support)
        set_dir_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/raw_map_saver/set_directory",
            std::bind(&RawMapSaver::set_directory_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create timer for automatic periodic saving if enabled
        if (auto_save_enabled_) {
            auto_save_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(auto_save_interval_),
                std::bind(&RawMapSaver::auto_save_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "Auto-save ENABLED: saving every %.1f seconds", auto_save_interval_);
        }
        
        RCLCPP_INFO(this->get_logger(), "Raw Map Saver started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Services: /save_raw_map, /raw_map_saver/set_directory");
        RCLCPP_INFO(this->get_logger(), "Auto-save: %s", auto_save_enabled_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "Tilt correction: %s", apply_tilt_correction_ ? "ENABLED" : "DISABLED");
        if (apply_tilt_correction_) {
            if (!calibration_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Calibration file: %s", calibration_file_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Fallback pitch: %.1f°", lidar_pitch_deg_);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Save format: %s", save_format_.c_str());
        if (save_raw_backup_) {
            RCLCPP_INFO(this->get_logger(), "Raw backup: ENABLED");
        }
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Store the latest cloud for saving
        latest_cloud_ = msg;
        clouds_received_++;
        
        // Log every 50th cloud to reduce spam
        if (clouds_received_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received %d clouds, latest: %lu points", 
                       clouds_received_, msg->data.size() / msg->point_step);
        }
    }
    
    // NEW: Find .npz file in directory
    std::string find_npz_file(const std::string& directory)
    {
        try {
            for (const auto& entry : std::filesystem::directory_iterator(directory)) {
                if (entry.path().extension() == ".npz") {
                    std::string filename = entry.path().filename().string();
                    if (filename.find("tilt_correction_matrices") != std::string::npos) {
                        return entry.path().string();
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error searching for .npz file: %s", e.what());
        }
        return "";
    }
    
    // Build fixed pitch rotation matrix (rotation around Y-axis)
    Eigen::Matrix3d build_pitch_rotation(double pitch_rad)
    {
        double cp = std::cos(pitch_rad);
        double sp = std::sin(pitch_rad);
        Eigen::Matrix3d R;
        R << cp,  0.0, sp,
             0.0, 1.0, 0.0,
            -sp,  0.0, cp;
        return R;
    }
    
    // Load transformation matrices from .npz file using Python helper
    bool load_npz_matrices(const std::string& npz_file, Eigen::Matrix3d& R_map, Eigen::Vector3d& p0_world)
    {
        RCLCPP_INFO(this->get_logger(), "Loading tilt correction from: %s", npz_file.c_str());
        
        // Extract matrices using Python (simplest approach since numpy is already available)
        std::string temp_dir = "/tmp";
        std::string r_map_file = temp_dir + "/r_map_temp.txt";
        std::string p0_file = temp_dir + "/p0_world_temp.txt";
        
        // Python one-liner to extract matrices
        std::string cmd = "python3 -c \""
                         "import numpy as np; "
                         "d = np.load('" + npz_file + "'); "
                         "np.savetxt('" + r_map_file + "', d['R_map']); "
                         "np.savetxt('" + p0_file + "', d['p0_world']); "
                         "\" 2>/dev/null";
        
        int ret = std::system(cmd.c_str());
        
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract matrices from .npz file");
            return false;
        }
        
        // Read R_map (3x3 matrix)
        std::ifstream r_file(r_map_file);
        if (!r_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open R_map temp file");
            return false;
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r_file >> R_map(i, j);
            }
        }
        r_file.close();
        
        // Read p0_world (3D vector)
        std::ifstream p0_file_stream(p0_file);
        if (!p0_file_stream.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open p0_world temp file");
            return false;
        }
        
        for (int i = 0; i < 3; i++) {
            p0_file_stream >> p0_world(i);
        }
        p0_file_stream.close();
        
        // Clean up temp files
        std::filesystem::remove(r_map_file);
        std::filesystem::remove(p0_file);
        
        RCLCPP_INFO(this->get_logger(), "✓ Tilt correction matrices loaded successfully");
        RCLCPP_INFO(this->get_logger(), "  p0_world: [%.3f, %.3f, %.3f]", 
                   p0_world(0), p0_world(1), p0_world(2));
        
        return true;
    }
    
    // Load transformation matrices - tries calibration file, then session folder, then fixed pitch
    bool load_tilt_correction_matrices(Eigen::Matrix3d& R_map, Eigen::Vector3d& p0_world)
    {
        // Priority 1: Use calibration file parameter if provided
        if (!calibration_file_.empty() && std::filesystem::exists(calibration_file_)) {
            RCLCPP_INFO(this->get_logger(), "Using calibration file: %s", calibration_file_.c_str());
            if (load_npz_matrices(calibration_file_, R_map, p0_world)) {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration file, trying session folder...");
        }
        
        // Priority 2: Look for session-specific .npz file in save_directory
        std::string session_npz = find_npz_file(save_directory_);
        if (!session_npz.empty()) {
            RCLCPP_INFO(this->get_logger(), "Using session transformation file");
            if (load_npz_matrices(session_npz, R_map, p0_world)) {
                return true;
            }
        }
        
        // Priority 3: Fall back to fixed pitch rotation
        RCLCPP_WARN(this->get_logger(), 
                   "No calibration file found, using fixed %.1f° pitch correction", 
                   lidar_pitch_deg_);
        
        // Build fixed pitch rotation (LiDAR is pitched forward, so rotate by -pitch)
        double pitch_rad = lidar_pitch_deg_ * M_PI / 180.0;
        R_map = build_pitch_rotation(-pitch_rad);
        p0_world = Eigen::Vector3d::Zero();  // No origin offset for fixed correction
        
        return true;
    }
    
    // NEW: Apply tilt correction to point cloud
    void apply_tilt_correction_to_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       const Eigen::Matrix3d& R_map,
                                       const Eigen::Vector3d& p0_world)
    {
        RCLCPP_INFO(this->get_logger(), "Applying tilt correction to %lu points...", cloud->size());
        
        // Get original bounds for logging
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double z_min = std::numeric_limits<double>::max();
        double z_max = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : cloud->points) {
            x_min = std::min(x_min, (double)pt.x);
            x_max = std::max(x_max, (double)pt.x);
            z_min = std::min(z_min, (double)pt.z);
            z_max = std::max(z_max, (double)pt.z);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Original bounds: x=[%.2f, %.2f], z=[%.2f, %.2f]",
                   x_min, x_max, z_min, z_max);
        
        // Apply transformation: p_corrected = R_map @ (p_raw - p0_world)
        for (auto& point : cloud->points) {
            Eigen::Vector3d p_raw(point.x, point.y, point.z);
            Eigen::Vector3d p_centered = p_raw - p0_world;
            Eigen::Vector3d p_corrected = R_map * p_centered;
            
            point.x = p_corrected.x();
            point.y = p_corrected.y();
            point.z = p_corrected.z();
        }
        
        // Get corrected bounds
        x_min = std::numeric_limits<double>::max();
        x_max = std::numeric_limits<double>::lowest();
        z_min = std::numeric_limits<double>::max();
        z_max = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : cloud->points) {
            x_min = std::min(x_min, (double)pt.x);
            x_max = std::max(x_max, (double)pt.x);
            z_min = std::min(z_min, (double)pt.z);
            z_max = std::max(z_max, (double)pt.z);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Corrected bounds: x=[%.2f, %.2f], z=[%.2f, %.2f]",
                   x_min, x_max, z_min, z_max);
        
        RCLCPP_INFO(this->get_logger(), "✓ Tilt correction applied successfully");
    }
    
    // Helper function to save point cloud in the chosen format
    bool save_cloud_with_format(const std::string& filepath, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                std::string& format_used)
    {
        try {
            if (save_format_ == "compressed") {
                // Lossless compressed PCD - best for wireless transfer
                // 40-60% smaller than binary, no data loss
                pcl::io::savePCDFile(filepath, *cloud, true);
                format_used = "compressed PCD (lossless)";
                return true;
            } 
            else if (save_format_ == "ascii") {
                // ASCII PCD - human readable but large
                // Not recommended for large maps
                pcl::io::savePCDFileASCII(filepath, *cloud);
                format_used = "ASCII PCD";
                return true;
            } 
            else {
                // Binary PCD (default/fallback) - fast but larger
                pcl::io::savePCDFileBinary(filepath, *cloud);
                format_used = "binary PCD";
                return true;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save cloud: %s", e.what());
            return false;
        }
    }
    
    void auto_save_callback()
    {
        if (!latest_cloud_) {
            RCLCPP_WARN(this->get_logger(), "Auto-save: No point cloud data available");
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Auto-save: Saving point cloud map...");
            
            // Generate filename with timestamp
            std::string filename = generate_filename("raw_map");
            std::string filepath = save_directory_ + "/" + filename;
            
            // Convert to PCL format
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*latest_cloud_, *cloud);

            // Save using configured format
            std::string format_used;
            if (save_cloud_with_format(filepath, cloud, format_used)) {
                RCLCPP_INFO(this->get_logger(), 
                           "Auto-save: Map saved successfully (%s): %s (%lu points)", 
                           format_used.c_str(), filename.c_str(), cloud->size());
                auto_saves_count_++;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Auto-save: Failed to save map");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Auto-save error: %s", e.what());
        }
    }
    
    void save_raw_map_service(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request; // Unused parameter
        
        if (!latest_cloud_) {
            response->success = false;
            response->message = "No point cloud data available for saving";
            RCLCPP_WARN(this->get_logger(), "No point cloud data available for saving");
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Processing point cloud map from Fast-LIO2...");
            
            // Convert ROS message to PCL PointXYZ for processing
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*latest_cloud_, *cloud);
            
            RCLCPP_INFO(this->get_logger(), "Loaded %lu points for processing", cloud->size());
            
            // Save raw backup if requested
            if (save_raw_backup_) {
                std::string raw_filename = generate_filename("raw_map");
                std::string raw_filepath = save_directory_ + "/" + raw_filename;
                pcl::io::savePCDFileBinary(raw_filepath, *cloud);
                RCLCPP_INFO(this->get_logger(), "Raw backup saved: %s", raw_filename.c_str());
            }
            
            // Apply tilt correction if enabled
            bool corrected = false;
            if (apply_tilt_correction_) {
                Eigen::Matrix3d R_map;
                Eigen::Vector3d p0_world;
                
                if (load_tilt_correction_matrices(R_map, p0_world)) {
                    apply_tilt_correction_to_cloud(cloud, R_map, p0_world);
                    corrected = true;
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                               "⚠ Could not load tilt correction, saving uncorrected map");
                }
            }
            
            // Save the (possibly corrected) map
            std::string prefix = corrected ? "corrected_map" : "raw_map";
            std::string filename = generate_filename(prefix);
            std::string filepath = save_directory_ + "/" + filename;
            
            // Save using configured format
            std::string format_used;
            if (!save_cloud_with_format(filepath, cloud, format_used)) {
                throw std::runtime_error("Failed to save point cloud file");
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "✓ %s saved (%s): %s (%lu points)",
                       corrected ? "Corrected map" : "Raw map",
                       format_used.c_str(),
                       filename.c_str(),
                       cloud->size());
            
            response->success = true;
            response->message = (corrected ? "Tilt-corrected map" : "Raw map") 
                              + std::string(" saved as ") + format_used 
                              + std::string(" to ") + filepath;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Save error: %s", e.what());
            response->success = false;
            response->message = "Save error: " + std::string(e.what());
        }
    }
    
    std::string generate_filename(const std::string& prefix)
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << prefix << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        ss << ".pcd";
        
        return ss.str();
    }
    
    // Keep for backward compatibility
    std::string generate_raw_filename()
    {
        return generate_filename("raw_map");
    }
    
    void set_directory_service(
        const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        std::string new_dir = this->get_parameter("save_directory").as_string();
        if (new_dir.empty()) {
            response->success = false;
            response->message = "save_directory parameter is empty";
            return;
        }
        save_directory_ = new_dir;
        std::filesystem::create_directories(save_directory_);
        RCLCPP_INFO(this->get_logger(), "Save directory updated: %s", save_directory_.c_str());
        response->success = true;
        response->message = "Directory set to: " + save_directory_;
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_dir_service_;
    rclcpp::TimerBase::SharedPtr auto_save_timer_;
    
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    int clouds_received_ = 0;
    int auto_saves_count_ = 0;
    
    std::string input_topic_;
    std::string save_directory_;
    std::string raw_map_filename_;
    bool auto_save_enabled_;
    double auto_save_interval_;
    
    // Tilt correction parameters
    bool apply_tilt_correction_;
    bool save_raw_backup_;
    std::string calibration_file_;
    double lidar_pitch_deg_;
    
    // Save format parameter for wireless transfer optimization
    std::string save_format_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RawMapSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
