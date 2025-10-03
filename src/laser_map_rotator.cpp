#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>

class LaserMapRotator : public rclcpp::Node
{
public:
  LaserMapRotator() : Node("laser_map_rotator")
  {
    // Declare & read parameters
    this->declare_parameter<std::string>("input_topic", "/Laser_map");
    this->declare_parameter<std::string>("output_topic", "/Laser_map_rotated");
    this->declare_parameter<double>("pitch_rad", -0.2617993878); // 15 deg default
    this->declare_parameter<std::string>("output_frame", "foot_init");
    this->declare_parameter<double>("max_height", 0.5);
    this->declare_parameter<double>("min_height", -0.2);
    this->declare_parameter<std::string>("accel_topic", "/livox/imu");
    this->declare_parameter<int>("accel_samples", 10);

    input_topic_  = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    pitch_rad_    = this->get_parameter("pitch_rad").as_double();
    output_frame_ = this->get_parameter("output_frame").as_string();
    min_height_  = this->get_parameter("min_height").as_double();
    max_height_  = this->get_parameter("max_height").as_double();
    accel_topic_  = this->get_parameter("accel_topic").as_string();
    {
      int param_samples = this->get_parameter("accel_samples").as_int();
      if (param_samples < 1) param_samples = 1;
      accel_samples_target_ = param_samples;
    }

    // Initialize transform with fixed pitch as a fallback until IMU-based init completes
    Eigen::AngleAxisf roll_rot(pitch_rad_, Eigen::Vector3f::UnitY());
    transform_ = Eigen::Affine3f(roll_rot);

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

    // Subscribe to IMU acceleration for one-time alignment initialization
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        accel_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LaserMapRotator::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "LaserMapRotator started. cloud: %s -> %s, output_frame: %s, accel_topic: %s, accel_samples: %d",
                input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str(),
                accel_topic_.c_str(), accel_samples_target_);
  }

private:
  // Compute quaternion aligning vector 'from_vec' to 'to_vec'
  static tf2::Quaternion computeAlignmentQuat(const Eigen::Vector3d &from_vec,
                                              const Eigen::Vector3d &to_vec)
  {
    Eigen::Vector3d v1 = from_vec.normalized();
    Eigen::Vector3d v2 = to_vec.normalized();
    double cos_theta = v1.dot(v2);

    Eigen::Quaterniond q_eig;
    // Handle edge cases for numerical stability
    if (cos_theta > 1.0 - 1e-9) {
      q_eig = Eigen::Quaterniond::Identity();
    } else if (cos_theta < -1.0 + 1e-9) {
      // 180-degree rotation around any axis orthogonal to v1
      Eigen::Vector3d axis = v1.unitOrthogonal();
      q_eig = Eigen::AngleAxisd(M_PI, axis.normalized());
    } else {
      Eigen::Vector3d axis = v1.cross(v2);
      double s = std::sqrt((1.0 + cos_theta) * 2.0);
      double invs = 1.0 / s;
      q_eig.w() = s * 0.5;
      q_eig.x() = axis.x() * invs;
      q_eig.y() = axis.y() * invs;
      q_eig.z() = axis.z() * invs;
      q_eig.normalize();
    }

    tf2::Quaternion q_tf;
    q_tf.setW(q_eig.w());
    q_tf.setX(q_eig.x());
    q_tf.setY(q_eig.y());
    q_tf.setZ(q_eig.z());
    q_tf.normalize();
    return q_tf;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (accel_initialized_) {
      return; // already initialized
    }

    // Accumulate linear acceleration samples
    Eigen::Vector3d a(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z);

    // Ignore clearly invalid NaN/Inf
    if (!std::isfinite(a.x()) || !std::isfinite(a.y()) || !std::isfinite(a.z())) {
      return;
    }
    a.x() = -a.x();
    a.z() = -a.z();

    accel_sum_ += a;
    accel_count_ += 1;

    if (accel_count_ < accel_samples_target_) {
      return;
    }

    // Compute average acceleration and derive alignment quaternion to -Z
    Eigen::Vector3d avg = accel_sum_ / static_cast<double>(accel_count_);
    double norm = avg.norm();
    if (norm < 1e-3) {
      RCLCPP_WARN(this->get_logger(), "Average acceleration magnitude too small (%.6f). Waiting for more samples...", norm);
      return;
    }

    // Align avg acceleration to -Z axis of the target frame
    tf2::Quaternion q_align = computeAlignmentQuat(avg, Eigen::Vector3d(0.0, 0.0, -1.0));

    tf2_transform_.transform.rotation.x = q_align.x();
    tf2_transform_.transform.rotation.y = q_align.y();
    tf2_transform_.transform.rotation.z = q_align.z();
    tf2_transform_.transform.rotation.w = q_align.w();

    accel_initialized_ = true;
    // Optionally reset subscription to free resources; keep it alive but no-op is fine
    RCLCPP_INFO(this->get_logger(),
                "IMU-based alignment initialized with %d samples. avg=[%.4f %.4f %.4f], q=[%.4f %.4f %.4f %.4f]",
                accel_count_, avg.x(), avg.y(), avg.z(),
                q_align.x(), q_align.y(), q_align.z(), q_align.w());
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Gate processing until IMU-based initialization has completed
    if (!accel_initialized_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for IMU alignment (%d/%d samples)...",
                           accel_count_, accel_samples_target_);
      return;
    }

    // Use tf2 to transform the PointCloud2 directly (robust)
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
  double min_height_{0.03};
  double max_height_{1.0};
  std::string accel_topic_;
  int accel_samples_target_{10};
  bool accel_initialized_{false};
  Eigen::Vector3d accel_sum_{0.0, 0.0, 0.0};
  int accel_count_{0};

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserMapRotator>());
  rclcpp::shutdown();
  return 0;
} 