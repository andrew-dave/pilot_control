#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class TopicThrottle : public rclcpp::Node {
public:
    TopicThrottle() : Node("topic_throttle") {
        // Declare parameters
        this->declare_parameter("throttle_ratio", 0.3);  // Only publish 30% of messages
        this->declare_parameter("livox_throttle_ratio", 0.2);  // Only publish 20% of LiDAR messages
        
        throttle_ratio_ = this->get_parameter("throttle_ratio").as_double();
        livox_throttle_ratio_ = this->get_parameter("livox_throttle_ratio").as_double();
        
        // Create publishers and subscribers
        // Note: Livox throttling removed due to dependency issues
        
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_throttled", 5);
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 5,
            std::bind(&TopicThrottle::cloudCallback, this, std::placeholders::_1));
        
        cloud_body_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body_throttled", 5);
        cloud_body_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", 5,
            std::bind(&TopicThrottle::cloudBodyCallback, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_throttled", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TopicThrottle::odomCallback, this, std::placeholders::_1));
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_throttled", 10);
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&TopicThrottle::pathCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Topic throttle initialized with ratios: %.2f, %.2f", 
                   throttle_ratio_, livox_throttle_ratio_);
    }

private:
    // Livox callback removed due to dependency issues
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (shouldPublish(throttle_ratio_)) {
            cloud_pub_->publish(*msg);
        }
    }
    
    void cloudBodyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (shouldPublish(throttle_ratio_)) {
            cloud_body_pub_->publish(*msg);
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (shouldPublish(throttle_ratio_)) {
            odom_pub_->publish(*msg);
        }
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (shouldPublish(throttle_ratio_)) {
            path_pub_->publish(*msg);
        }
    }
    
    bool shouldPublish(double ratio) {
        return (static_cast<double>(rand()) / RAND_MAX) < ratio;
    }
    
    double throttle_ratio_;
    double livox_throttle_ratio_;
    
    // Livox publishers/subscribers removed due to dependency issues
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicThrottle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 