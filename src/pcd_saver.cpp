#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

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
        RCLCPP_INFO(this->get_logger(), "Manual save service available at: /save_map");
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
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocess_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        if (cloud->empty()) {
            return cloud;
        }
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Step 1: Height filtering (remove points above max_height and below min_height)
        pcl::PassThrough<pcl::PointXYZI> height_filter;
        height_filter.setInputCloud(cloud);
        height_filter.setFilterFieldName("z");
        height_filter.setFilterLimits(min_height_, max_height_);
        height_filter.filter(*filtered_cloud);
        
        RCLCPP_DEBUG(this->get_logger(), "After height filter: %lu points", filtered_cloud->size());
        
        // Step 2: Voxel grid filtering to reduce density
        if (voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(filtered_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
            
            RCLCPP_DEBUG(this->get_logger(), "After voxel filter: %lu points", filtered_cloud->size());
        }
        
        // Step 3: Statistical outlier removal (remove flying points)
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 