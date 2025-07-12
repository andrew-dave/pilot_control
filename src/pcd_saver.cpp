#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <Eigen/Dense>

class PCDSaver : public rclcpp::Node
{
public:
    PCDSaver() : Node("pcd_saver")
    {
        // Declare parameters
        this->declare_parameter("save_directory", "/tmp/pcd_maps");
        this->declare_parameter("save_interval", 10); // Save every N seconds
        this->declare_parameter("max_height", 2.0); // Maximum height from ground in meters
        this->declare_parameter("min_height", 0.1); // Minimum height from ground in meters
        this->declare_parameter("remove_outliers", true); // Remove flying points
        this->declare_parameter("outlier_std_dev", 1.0); // Standard deviation for outlier removal
        this->declare_parameter("voxel_size", 0.05); // Voxel grid filter size
        this->declare_parameter("auto_save", true);
        this->declare_parameter("save_on_laptop", false); // Save on laptop instead of robot
        this->declare_parameter("apply_rotation_correction", true); // Apply 30-degree rotation correction
        this->declare_parameter("rotation_angle", -0.5230); // -30 degrees in radians
        this->declare_parameter("manual_save_filename", ""); // Custom filename for manual save
        
        // Get parameters
        save_directory_ = this->get_parameter("save_directory").as_string();
        save_interval_ = this->get_parameter("save_interval").as_int();
        max_height_ = this->get_parameter("max_height").as_double();
        min_height_ = this->get_parameter("min_height").as_double();
        remove_outliers_ = this->get_parameter("remove_outliers").as_bool();
        outlier_std_dev_ = this->get_parameter("outlier_std_dev").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        auto_save_ = this->get_parameter("auto_save").as_bool();
        save_on_laptop_ = this->get_parameter("save_on_laptop").as_bool();
        apply_rotation_correction_ = this->get_parameter("apply_rotation_correction").as_bool();
        rotation_angle_ = this->get_parameter("rotation_angle").as_double();
        manual_save_filename_ = this->get_parameter("manual_save_filename").as_string();
        
        // If saving on laptop, use home directory
        if (save_on_laptop_) {
            const char* home_dir = std::getenv("HOME");
            if (home_dir) {
                save_directory_ = std::string(home_dir) + "/robot_maps";
            }
        }
        
        // Create save directory if it doesn't exist
        std::filesystem::create_directories(save_directory_);
        
        // Create subscriber for Laser_map point cloud
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", // Specifically subscribe to Laser_map
            rclcpp::QoS(10).durability_volatile(),
            std::bind(&PCDSaver::cloud_callback, this, std::placeholders::_1)
        );
        
        // Create service for manual map saving
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map",
            std::bind(&PCDSaver::save_map_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create service for manual save with custom filename
        manual_save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "manual_save_map",
            std::bind(&PCDSaver::manual_save_map_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create timer for periodic saving
        if (auto_save_) {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(save_interval_),
                std::bind(&PCDSaver::timer_callback, this)
            );
        }
        
        RCLCPP_INFO(this->get_logger(), "PCD Saver started - saving from /Laser_map");
        RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save on laptop: %s", save_on_laptop_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Height filter: %.2f to %.2f meters", min_height_, max_height_);
        RCLCPP_INFO(this->get_logger(), "Remove outliers: %s", remove_outliers_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.3f meters", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "Rotation correction: %s (%.1f degrees)", 
                   apply_rotation_correction_ ? "enabled" : "disabled", rotation_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Manual save service available at: /save_map");
        RCLCPP_INFO(this->get_logger(), "Manual save with custom filename available at: /manual_save_map");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %lu points", cloud->size());
        
        // Preprocess the point cloud
        auto filtered_cloud = preprocess_cloud(cloud);
        
        // Store the filtered cloud
        latest_cloud_ = filtered_cloud;
        cloud_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Preprocessed cloud: %lu -> %lu points", 
                   cloud->size(), filtered_cloud->size());
    }
    
    void save_map_service(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request; // Unused parameter
        
        if (cloud_received_ && latest_cloud_ && !latest_cloud_->empty()) {
            save_pcd();
            response->success = true;
            response->message = "Map saved successfully to " + save_directory_;
            RCLCPP_INFO(this->get_logger(), "Map saved via service call to: %s", save_directory_.c_str());
        } else {
            response->success = false;
            response->message = "No point cloud data available for saving";
            RCLCPP_WARN(this->get_logger(), "No point cloud data available for saving");
        }
    }
    
    void manual_save_map_service(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request; // Unused parameter
        
        if (cloud_received_ && latest_cloud_ && !latest_cloud_->empty()) {
            // Disable auto-save timer when manual save is triggered
            if (timer_) {
                timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Auto-save disabled after manual save");
            }
            
            // Generate custom filename with date and mapping duration
            std::string custom_filename = generate_manual_filename();
            save_pcd_with_filename(custom_filename);
            
            response->success = true;
            response->message = "Manual map saved successfully to " + save_directory_ + "/" + custom_filename;
            RCLCPP_INFO(this->get_logger(), "Manual map saved: %s", custom_filename.c_str());
        } else {
            response->success = false;
            response->message = "No point cloud data available for saving";
            RCLCPP_WARN(this->get_logger(), "No point cloud data available for manual save");
        }
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocess_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        if (cloud->empty()) {
            return cloud;
        }
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Step 1: Apply rotation correction for tilted LiDAR FIRST
        if (apply_rotation_correction_ && !cloud->empty()) {
            // Create rotation matrix for 30-degree pitch correction
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(1, 1) = cos(rotation_angle_);  // cos(-30°)
            transform(1, 2) = -sin(rotation_angle_); // -sin(-30°)
            transform(2, 1) = sin(rotation_angle_);  // sin(-30°)
            transform(2, 2) = cos(rotation_angle_);  // cos(-30°)
            
            pcl::transformPointCloud(*cloud, *filtered_cloud, transform);
            
            RCLCPP_DEBUG(this->get_logger(), "Applied rotation correction (%.1f degrees)", 
                       rotation_angle_ * 180.0 / M_PI);
        } else {
            *filtered_cloud = *cloud;  // Copy if no rotation needed
        }
        
        // Step 2: Height filtering (remove points above max_height and below min_height)
        // Now applied to correctly oriented coordinate system
        pcl::PassThrough<pcl::PointXYZI> height_filter;
        height_filter.setInputCloud(filtered_cloud);
        height_filter.setFilterFieldName("z");
        height_filter.setFilterLimits(min_height_, max_height_);
        height_filter.filter(*filtered_cloud);
        
        RCLCPP_DEBUG(this->get_logger(), "After height filter: %lu points", filtered_cloud->size());
        
        // Step 3: Voxel grid filtering to reduce density
        if (voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(filtered_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
            
            RCLCPP_DEBUG(this->get_logger(), "After voxel filter: %lu points", filtered_cloud->size());
        }
        
        // Step 4: Statistical outlier removal (remove flying points)
        if (remove_outliers_ && filtered_cloud->size() > 10) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outlier_filter;
            outlier_filter.setInputCloud(filtered_cloud);
            outlier_filter.setMeanK(50); // Number of neighbors to analyze
            outlier_filter.setStddevMulThresh(outlier_std_dev_); // Standard deviation multiplier
            outlier_filter.filter(*filtered_cloud);
            
            RCLCPP_DEBUG(this->get_logger(), "After outlier removal: %lu points", filtered_cloud->size());
        }
        
        return filtered_cloud;
    }
    
    void timer_callback()
    {
        if (cloud_received_ && latest_cloud_ && !latest_cloud_->empty()) {
            save_pcd();
        } else {
            RCLCPP_WARN(this->get_logger(), "No point cloud data available for saving");
        }
    }
    
    std::string generate_manual_filename()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        // Get current date
        std::stringstream date_ss;
        date_ss << std::put_time(std::localtime(&time_t), "%B_%d"); // e.g., "July_25"
        
        // Calculate mapping duration (simplified - you might want to track actual start time)
        auto duration = std::chrono::duration_cast<std::chrono::minutes>(now.time_since_epoch());
        int minutes = duration.count() % 60;
        int hours = duration.count() / 60;
        
        std::stringstream filename_ss;
        filename_ss << "map_" << date_ss.str() << "_duration_" << hours << "h_" << minutes << "m.pcd";
        
        return filename_ss.str();
    }
    
    void save_pcd_with_filename(const std::string& custom_filename)
    {
        if (!latest_cloud_ || latest_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No point cloud data to save");
            return;
        }
        
        std::string full_path = save_directory_ + "/" + custom_filename;
        
        // Save the point cloud
        pcl::PCDWriter writer;
        if (writer.writeBinary(full_path, *latest_cloud_) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved manual PCD file: %s (%lu points)", 
                       full_path.c_str(), latest_cloud_->size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file: %s", full_path.c_str());
        }
    }
    
    void save_pcd()
    {
        if (!latest_cloud_ || latest_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No point cloud data to save");
            return;
        }
        
        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << save_directory_ << "/laser_map_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".pcd";
        
        std::string filename = ss.str();
        
        // Save the point cloud
        pcl::PCDWriter writer;
        if (writer.writeBinary(filename, *latest_cloud_) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved filtered PCD file: %s (%lu points)", 
                       filename.c_str(), latest_cloud_->size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file: %s", filename.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_save_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
    bool cloud_received_ = false;
    
    std::string save_directory_;
    double max_height_;
    double min_height_;
    bool remove_outliers_;
    double outlier_std_dev_;
    double voxel_size_;
    int save_interval_;
    bool auto_save_;
    bool save_on_laptop_;
    bool apply_rotation_correction_;
    double rotation_angle_;
    std::string manual_save_filename_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 