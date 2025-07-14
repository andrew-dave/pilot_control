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
#include <thread>
#include <atomic>
#include <algorithm>

class PCDProcessor : public rclcpp::Node
{
public:
    PCDProcessor() : Node("pcd_processor")
    {
        // Declare parameters
        this->declare_parameter("raw_map_directory", "/home/robot/maps");
        this->declare_parameter("processed_map_directory", "/home/robot/maps");
        this->declare_parameter("processing_mode", "high_quality"); // high_quality, fast, minimal
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("remove_outliers", true);
        this->declare_parameter("outlier_std_dev", 2.0);
        this->declare_parameter("apply_rotation_correction", true);
        this->declare_parameter("rotation_angle", -0.5230);
        this->declare_parameter("save_format", "pcd");
        this->declare_parameter("auto_shutdown", true);
        this->declare_parameter("max_height", 2.0);
        this->declare_parameter("min_height", 0.1);
        this->declare_parameter("find_latest_raw_map", true);
        
        // Get parameters
        raw_map_directory_ = this->get_parameter("raw_map_directory").as_string();
        processed_map_directory_ = this->get_parameter("processed_map_directory").as_string();
        processing_mode_ = this->get_parameter("processing_mode").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        remove_outliers_ = this->get_parameter("remove_outliers").as_bool();
        outlier_std_dev_ = this->get_parameter("outlier_std_dev").as_double();
        apply_rotation_correction_ = this->get_parameter("apply_rotation_correction").as_bool();
        rotation_angle_ = this->get_parameter("rotation_angle").as_double();
        save_format_ = this->get_parameter("save_format").as_string();
        auto_shutdown_ = this->get_parameter("auto_shutdown").as_bool();
        max_height_ = this->get_parameter("max_height").as_double();
        min_height_ = this->get_parameter("min_height").as_double();
        find_latest_raw_map_ = this->get_parameter("find_latest_raw_map").as_bool();
        
        // Create directories
        std::filesystem::create_directories(raw_map_directory_);
        std::filesystem::create_directories(processed_map_directory_);
        
        // Create service for manual processing trigger
        process_service_ = this->create_service<std_srvs::srv::Trigger>(
            "process_and_save_map",
            std::bind(&PCDProcessor::process_service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Initialize processing state
        processing_ = false;
        
        RCLCPP_INFO(this->get_logger(), "PCD Processor started");
        RCLCPP_INFO(this->get_logger(), "Raw map directory: %s", raw_map_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Processed map directory: %s", processed_map_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Processing mode: %s", processing_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Service available at: /process_and_save_map");
    }

private:
    void process_service_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request; // Unused parameter
        
        if (processing_) {
            response->success = false;
            response->message = "Already processing a map";
            return;
        }
        
        // Start processing in separate thread to avoid blocking
        processing_ = true;
        std::thread([this, response]() {
            this->process_map_async(response);
        }).detach();
    }
    
    void process_map_async(std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Starting map processing...");
            
            // Find the latest raw map file
            std::string raw_map_path = find_latest_raw_map();
            if (raw_map_path.empty()) {
                response->success = false;
                response->message = "No raw map files found in " + raw_map_directory_;
                processing_ = false;
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loading raw map: %s", raw_map_path.c_str());
            
            // Load the raw point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(raw_map_path, *raw_cloud) == -1) {
                response->success = false;
                response->message = "Failed to load raw map file: " + raw_map_path;
                processing_ = false;
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded raw cloud with %lu points", raw_cloud->size());
            
            // Process based on mode
            auto processed_cloud = process_cloud_by_mode(raw_cloud);
            
            RCLCPP_INFO(this->get_logger(), "Processed cloud has %lu points", processed_cloud->size());
            
            // Save the processed cloud
            std::string filename = generate_processed_filename();
            std::string filepath = processed_map_directory_ + "/" + filename;
            
            if (pcl::io::savePCDFile(filepath, *processed_cloud) == 0) {
                RCLCPP_INFO(this->get_logger(), "Processed map saved successfully: %s", filename.c_str());
                response->success = true;
                response->message = "Processed map saved to " + filepath;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save processed map");
                response->success = false;
                response->message = "Failed to save processed map file";
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
            response->success = false;
            response->message = "Processing error: " + std::string(e.what());
        }
        
        processing_ = false;
        
        if (auto_shutdown_) {
            RCLCPP_INFO(this->get_logger(), "Auto-shutdown enabled, shutting down...");
            rclcpp::shutdown();
        }
    }
    
    std::string find_latest_raw_map()
    {
        if (!find_latest_raw_map_) {
            // If not auto-finding, look for any raw_map_*.pcd file
            for (const auto& entry : std::filesystem::directory_iterator(raw_map_directory_)) {
                if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                    std::string filename = entry.path().filename().string();
                    if (filename.find("raw_map_") == 0) {
                        return entry.path().string();
                    }
                }
            }
            return "";
        }
        
        // Find the most recent raw_map_*.pcd file
        std::vector<std::filesystem::path> raw_maps;
        
        for (const auto& entry : std::filesystem::directory_iterator(raw_map_directory_)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                std::string filename = entry.path().filename().string();
                if (filename.find("raw_map_") == 0) {
                    raw_maps.push_back(entry.path());
                }
            }
        }
        
        if (raw_maps.empty()) {
            return "";
        }
        
        // Sort by modification time (newest first)
        std::sort(raw_maps.begin(), raw_maps.end(), 
                  [](const std::filesystem::path& a, const std::filesystem::path& b) {
                      return std::filesystem::last_write_time(a) > std::filesystem::last_write_time(b);
                  });
        
        RCLCPP_INFO(this->get_logger(), "Found %zu raw map files, using latest: %s", 
                   raw_maps.size(), raw_maps[0].filename().c_str());
        
        return raw_maps[0].string();
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr process_cloud_by_mode(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        if (processing_mode_ == "minimal") {
            return process_minimal(cloud);
        } else if (processing_mode_ == "fast") {
            return process_fast(cloud);
        } else { // high_quality
            return process_high_quality(cloud);
        }
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr process_minimal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        // Minimal processing - just rotation correction
        if (apply_rotation_correction_) {
            return apply_rotation(cloud);
        }
        return cloud;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr process_fast(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        // Fast processing - rotation + basic filtering
        auto processed = cloud;
        
        if (apply_rotation_correction_) {
            processed = apply_rotation(processed);
        }
        
        // Height filtering
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(processed);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*processed);
        
        // Light voxel filtering
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(processed);
        voxel.setLeafSize(voxel_size_ * 2.0, voxel_size_ * 2.0, voxel_size_ * 2.0); // Larger voxels for speed
        voxel.filter(*processed);
        
        return processed;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr process_high_quality(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        // High quality processing - full pipeline
        auto processed = cloud;
        
        // Rotation correction
        if (apply_rotation_correction_) {
            processed = apply_rotation(processed);
        }
        
        // Height filtering
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(processed);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*processed);
        
        // Outlier removal
        if (remove_outliers_) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            sor.setInputCloud(processed);
            sor.setMeanK(50);
            sor.setStddevMulThresh(outlier_std_dev_);
            sor.filter(*processed);
        }
        
        // Voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(processed);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*processed);
        
        return processed;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr apply_rotation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float cos_angle = cos(rotation_angle_);
        float sin_angle = sin(rotation_angle_);
        
        transform(1, 1) = cos_angle;
        transform(1, 2) = -sin_angle;
        transform(2, 1) = sin_angle;
        transform(2, 2) = cos_angle;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
        
        return rotated_cloud;
    }
    
    std::string generate_processed_filename()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << "processed_map_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        ss << "_" << processing_mode_ << ".pcd";
        
        return ss.str();
    }
    
    // Member variables
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr process_service_;
    
    std::atomic<bool> processing_;
    
    std::string raw_map_directory_;
    std::string processed_map_directory_;
    std::string processing_mode_;
    std::string save_format_;
    double voxel_size_;
    bool remove_outliers_;
    double outlier_std_dev_;
    bool apply_rotation_correction_;
    double rotation_angle_;
    bool auto_shutdown_;
    double max_height_;
    double min_height_;
    bool find_latest_raw_map_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 