#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFBridge : public rclcpp::Node
{
public:
    TFBridge() : Node("tf_bridge")
    {
        // Create static transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish the transform from camera_init to base_link
        // This connects the two TF trees
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
        
        RCLCPP_INFO(this->get_logger(), "Published static transform from camera_init to base_link");
    }

private:
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 