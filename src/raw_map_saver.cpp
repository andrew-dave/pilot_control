#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

class RawMapSaver : public rclcpp::Node
{
public:
    RawMapSaver() : Node("raw_map_saver")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/Laser_map");
        this->declare_parameter("save_directory", "/tmp/robot_maps");  // Use /tmp which has write permissions
        this->declare_parameter("raw_map_filename", "");
        
        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        raw_map_filename_ = this->get_parameter("raw_map_filename").as_string();
        
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
        
        RCLCPP_INFO(this->get_logger(), "Raw Map Saver started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Service available at: /save_raw_map");
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
            RCLCPP_INFO(this->get_logger(), "Saving raw map from Fast-LIO2...");
            
            // Generate filename with timestamp
            std::string filename = generate_raw_filename();
            std::string filepath = save_directory_ + "/" + filename;
            
            // Save the raw point cloud directly without any processing
            if (pcl::io::savePCDFile(filepath, *latest_cloud_) == 0) {
                RCLCPP_INFO(this->get_logger(), "Raw map saved successfully: %s", filename.c_str());
                RCLCPP_INFO(this->get_logger(), "Points saved: %lu", latest_cloud_->data.size() / latest_cloud_->point_step);
                response->success = true;
                response->message = "Raw map saved to " + filepath;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save raw map");
                response->success = false;
                response->message = "Failed to save raw map file";
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Save error: %s", e.what());
            response->success = false;
            response->message = "Save error: " + std::string(e.what());
        }
    }
    
    std::string generate_raw_filename()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << "raw_map_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        ss << ".pcd";
        
        return ss.str();
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    int clouds_received_ = 0;
    
    std::string input_topic_;
    std::string save_directory_;
    std::string raw_map_filename_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RawMapSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 