#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class GPRMotionFollower : public rclcpp::Node {
public:
  GPRMotionFollower() : rclcpp::Node("gpr_motion_follower") {
    // Parameters
    this->declare_parameter<std::string>("odrive_namespace", "gpr");
    this->declare_parameter<double>("encoder_wheel_diameter", 0.06); // 60 mm
    this->declare_parameter<double>("micro_gear_ratio", 1.0);
    this->declare_parameter<double>("velocity_multiplier", 1.0);
    this->declare_parameter<double>("velocity_deadband", 0.01);

    odrive_namespace_     = this->get_parameter("odrive_namespace").as_string();
    encoder_wheel_diameter_ = this->get_parameter("encoder_wheel_diameter").as_double();
    micro_gear_ratio_     = this->get_parameter("micro_gear_ratio").as_double();
    velocity_multiplier_  = this->get_parameter("velocity_multiplier").as_double();
    velocity_deadband_    = this->get_parameter("velocity_deadband").as_double();

    // Topics/services for the micro ODrive (node_id 2) under its namespace
    const std::string control_topic = "/" + odrive_namespace_ + "/control_message";
    const std::string axis_service  = "/" + odrive_namespace_ + "/request_axis_state";

    control_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(control_topic, 10);
    axis_client_ = this->create_client<odrive_can::srv::AxisState>(axis_service);

    // Subscribe to robot odometry to mirror ACTUAL forward motion
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&GPRMotionFollower::odomCallback, this, std::placeholders::_1)
    );

    // Attempt to arm the micro axis shortly after startup
    arm_timer_ = this->create_wall_timer(200ms, [this]() {
      this->armMicroAxis();
      this->arm_timer_->cancel();
    });

    RCLCPP_INFO(this->get_logger(),
                "GPRMotionFollower started (ns: %s, wheel_diam=%.3f m, gear=%.2f)",
                odrive_namespace_.c_str(), encoder_wheel_diameter_, micro_gear_ratio_);
  }

private:
  void armMicroAxis() {
    RCLCPP_INFO(this->get_logger(), "Waiting for micro ODrive service to become available...");
    if (!axis_client_->wait_for_service(3s)) {
      RCLCPP_WARN(this->get_logger(), "Micro ODrive service not available; follower will still publish velocities");
      return;
    }
    auto req = std::make_shared<odrive_can::srv::AxisState::Request>();
    req->axis_requested_state = 8; // CLOSED_LOOP
    (void)axis_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested CLOSED_LOOP for micro ODrive axis");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double linear_v = msg->twist.twist.linear.x; // m/s (base_link forward)

    // Only move when forward command is given; otherwise command zero velocity
    if (linear_v <= velocity_deadband_) {
      publishVelocity(0.0);
      return;
    }

    const double circumference = M_PI * encoder_wheel_diameter_; // m/turn
    double wheel_turns_per_sec = linear_v / circumference;       // turns/s at 60 mm wheel
    double motor_turns_per_sec = wheel_turns_per_sec * micro_gear_ratio_ * velocity_multiplier_;

    publishVelocity(motor_turns_per_sec);
  }

  void publishVelocity(double turns_per_sec) {
    odrive_can::msg::ControlMessage m;
    m.control_mode = 2; // VELOCITY_CONTROL
    m.input_mode   = 2; // VEL_RAMP
    m.input_vel    = turns_per_sec;
    m.input_torque = 0.0;
    control_pub_->publish(m);
  }

  // Members
  std::string odrive_namespace_;
  double encoder_wheel_diameter_{};
  double micro_gear_ratio_{};
  double velocity_multiplier_{};
  double velocity_deadband_{};

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr control_pub_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr axis_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr arm_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPRMotionFollower>());
  rclcpp::shutdown();
  return 0;
}


