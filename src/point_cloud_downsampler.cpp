#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudDownsampler : public rclcpp::Node
{
public:
    PointCloudDownsampler() : Node("point_cloud_downsampler")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/Laser_map");
        this->declare_parameter("output_topic", "/Laser_map_downsampled");
        this->declare_parameter("voxel_size", 0.2);  // 20cm voxels (much coarser than original)
        this->declare_parameter("target_reduction_factor", 20);  // Aim for 20x smaller
        
        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        target_reduction_ = this->get_parameter("target_reduction_factor").as_int();
        
        // Create subscriber and publisher
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::QoS(10).durability_volatile(),
            std::bind(&PointCloudDownsampler::cloud_callback, this, std::placeholders::_1)
        );
        
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_,
            rclcpp::QoS(10).durability_volatile()
        );
        
        RCLCPP_INFO(this->get_logger(), "Point Cloud Downsampler started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.2f m", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "Target reduction: %dx", target_reduction_);
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Convert ROS2 message to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud_in);
            
            size_t original_size = cloud_in->points.size();
            
            // Apply voxel grid downsampling
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud_in);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*cloud_filtered);
            
            size_t filtered_size = cloud_filtered->points.size();
            
            // Convert back to ROS2 message
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_filtered, output_msg);
            output_msg.header = msg->header;
            
            // Publish downsampled cloud
            cloud_pub_->publish(output_msg);
            
            // Log stats every 50 messages
            message_count_++;
            if (message_count_ % 50 == 0) {
                float reduction_factor = static_cast<float>(original_size) / static_cast<float>(filtered_size);
                RCLCPP_INFO(this->get_logger(), 
                    "Processed %d clouds | Original: %zu pts | Downsampled: %zu pts | Reduction: %.1fx",
                    message_count_, original_size, filtered_size, reduction_factor);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Downsampling error: %s", e.what());
        }
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    std::string input_topic_;
    std::string output_topic_;
    double voxel_size_;
    int target_reduction_;
    int message_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudDownsampler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

