#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LaserMapRotator : public rclcpp::Node
{
public:
  LaserMapRotator() : Node("laser_map_rotator")
  {
    // Declare & read parameters
    this->declare_parameter<std::string>("input_topic", "/Laser_map");
    this->declare_parameter<std::string>("output_topic", "/Laser_map_rotated");
    this->declare_parameter<double>("pitch_rad", 0.0); // 30 deg default
    this->declare_parameter<std::string>("output_frame", "foot_init");
    this->declare_parameter<double>("max_height", 1.0);
    this->declare_parameter<double>("min_height", 0.05);

    input_topic_  = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    pitch_rad_    = this->get_parameter("pitch_rad").as_double();
    output_frame_ = this->get_parameter("output_frame").as_string();
    min_height_  = this->get_parameter("min_height").as_double();
    max_height_  = this->get_parameter("max_height").as_double();

    // Pre-compute transform both as Eigen (possibly used later) and as tf2 TransformStamped for safe PointCloud2 transform
    Eigen::AngleAxisf pitch_rot(pitch_rad_, Eigen::Vector3f::UnitY());
    transform_ = Eigen::Affine3f(pitch_rot);

    tf2::Quaternion q;
    q.setRPY(0.0, pitch_rad_, 0.0);
    q.normalize();
    tf2_transform_.transform.rotation.x = q.x();
    tf2_transform_.transform.rotation.y = q.y();
    tf2_transform_.transform.rotation.z = q.z();
    tf2_transform_.transform.rotation.w = q.w();
    tf2_transform_.transform.translation.x = 0.0;
    tf2_transform_.transform.translation.y = 0.0;
    tf2_transform_.transform.translation.z = 0.0;

    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    pub_qos.reliable();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, pub_qos);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LaserMapRotator::pointcloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LaserMapRotator started. Subscribing %s, publishing %s (pitch %.3f rad)",
                input_topic_.c_str(), output_topic_.c_str(), pitch_rad_);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Use tf2 to transform the PointCloud2 directly (robust, avoids SSE issues)
    tf2_transform_.header.stamp = msg->header.stamp;
    tf2_transform_.header.frame_id = msg->header.frame_id;  // source frame
    tf2_transform_.child_frame_id = output_frame_;

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      tf2::doTransform(*msg, transformed, tf2_transform_);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Transform failed: %s", ex.what());
      return;
    }

    // Height filtering on transformed cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_in;
    pcl::fromROSMsg(transformed, pcl_in);

    pcl::PointCloud<pcl::PointXYZ> pcl_out;
    pcl_out.reserve(pcl_in.size());
    for (const auto &pt : pcl_in.points) {
      if (pt.z > min_height_ && pt.z < max_height_) {
        pcl_out.emplace_back(pt);
      }
    }

    if (pcl_out.empty()) {
      // Nothing to publish
      return;
    }

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(pcl_out, filtered_msg);
    filtered_msg.header = transformed.header;
    filtered_msg.header.frame_id = output_frame_;

    publisher_->publish(filtered_msg);
  }

  // Parameters / internals
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double pitch_rad_{};
  Eigen::Affine3f transform_;
  geometry_msgs::msg::TransformStamped tf2_transform_;
  double min_height_{0.04};
  double max_height_{2.0};

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserMapRotator>());
  rclcpp::shutdown();
  return 0;
} 