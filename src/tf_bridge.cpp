#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFBridge : public rclcpp::Node
{
public:
    TFBridge() : Node("tf_bridge")
    {
        // Create transform broadcaster (not static, so it publishes continuously)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer to publish transform periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&TFBridge::publish_transform, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "TF Bridge started - publishing transform from camera_init to base_link");
    }

private:
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "base_link";
        
        // Identity transform (no translation, no rotation)
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        // Send the transform
        tf_broadcaster_->sendTransform(transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 