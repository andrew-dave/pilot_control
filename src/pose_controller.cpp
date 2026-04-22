/**
 * Pose Controller Node (C++)
 * ====================
 * Subscribes to Fast-LIO2 odometry and generates left/right wheel velocity commands
 * to drive the robot to target poses.
 *
 * Features:
 * - Subscribes to Fast-LIO2 /Odometry for localization (x, y, yaw)
 * - Control loop runs at 10 Hz
 * - Generates left and right wheel velocity commands
 * - Publishes to ODrive CAN nodes
 *
 * Topics:
 *   Subscribed:
 *     - /Odometry (nav_msgs/Odometry) - Fast-LIO2 localization
 *     - /set_target_pose (std_msgs/Float64MultiArray) - [x, y, z, yaw(rad)]
 *   Published:
 *     - /left/control_message (odrive_can/ControlMessage) - Left wheel velocity
 *     - /right/control_message (odrive_can/ControlMessage) - Right wheel velocity
 */

#include "pilot_control/pose_controller.hpp"
#include <cmath>
#include <algorithm>

namespace pilot_control
{

PoseController::PoseController()
: Node("pose_controller"),
  current_x_(0.0),
  current_y_(0.0),
  current_yaw_(0.0),
  current_vx_(0.0),
  current_vy_(0.0),
  current_vyaw_(0.0),
  pose_initialized_(false),
  accel_initialized_(false),
  accel_sum_(Eigen::Vector3d::Zero()),
  accel_count_(0),
  target_x_(0.0),
  target_y_(0.0),
  target_yaw_(0.0),
  has_target_(false),
  left_wheel_velocity_(0.0),
  right_wheel_velocity_(0.0),
  last_linear_vel_(0.0),
  last_angular_vel_(0.0),
  control_counter_(0),
  goal_reached_(false),
  arm_attempts_(0),
  arm_max_attempts_(5)
{
  // ============================================================
  // PARAMETERS
  // ============================================================
  
  // Robot kinematics
  this->declare_parameter("wheel_radius", 0.072);
  this->declare_parameter("wheel_base", 0.32);
  this->declare_parameter("gear_ratio", 1.0);
  this->declare_parameter("invert_left", false);
  this->declare_parameter("invert_right", true);
  this->declare_parameter("velocity_multiplier", 1.0);
  
  // Control parameters
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("max_linear_velocity", 0.3);
  this->declare_parameter("max_angular_velocity", 1.5);
  this->declare_parameter("max_linear_accel", 0.5);
  this->declare_parameter("max_angular_accel", 2.0);
  this->declare_parameter("position_tolerance", 0.05);
  this->declare_parameter("orientation_tolerance", 0.1);
  this->declare_parameter("min_wheel_rps", 0.2);
  
  // Pure pursuit parameters
  this->declare_parameter("lookahead_distance", 0.3);
  this->declare_parameter("lookahead_gain", 0.5);
  this->declare_parameter("min_lookahead_distance", 0.2);
  this->declare_parameter("max_lookahead_distance", 1.0);
  
  // Tilt correction parameters
  this->declare_parameter("pitch_rad", -0.2617993878);  // ~ -15 deg fallback
  this->declare_parameter("accel_topic", "/livox/imu");
  this->declare_parameter("accel_samples", 10);
  
  // PID gains
  this->declare_parameter("Kp_linear", 25.0);
  this->declare_parameter("Ki_linear", 0.0);
  this->declare_parameter("Kd_linear", 0.0);
  this->declare_parameter("Kp_angular", 25.0);
  this->declare_parameter("Ki_angular", 0.0);
  this->declare_parameter("Kd_angular", 0.0);
  
  // Safety parameters
  this->declare_parameter("enable_controller", true);
  this->declare_parameter("odometry_timeout_ms", 500);
  
  // Topic names
  this->declare_parameter("odometry_topic", "/Odometry");
  this->declare_parameter("left_control_topic", "/left/control_message");
  this->declare_parameter("right_control_topic", "/right/control_message");
  this->declare_parameter("corrected_odometry_topic", "/Odometry_tilt_corrected");
  
  // Get parameters
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_base_ = this->get_parameter("wheel_base").as_double();
  gear_ratio_ = this->get_parameter("gear_ratio").as_double();
  invert_left_ = this->get_parameter("invert_left").as_bool();
  invert_right_ = this->get_parameter("invert_right").as_bool();
  velocity_multiplier_ = this->get_parameter("velocity_multiplier").as_double();
  
  control_freq_ = this->get_parameter("control_frequency").as_double();
  max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
  max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
  max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
  pos_tolerance_ = this->get_parameter("position_tolerance").as_double();
  ori_tolerance_ = this->get_parameter("orientation_tolerance").as_double();
  min_wheel_rps_ = this->get_parameter("min_wheel_rps").as_double();
  
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();
  min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
  max_lookahead_distance_ = this->get_parameter("max_lookahead_distance").as_double();
  
  pitch_rad_ = this->get_parameter("pitch_rad").as_double();
  accel_topic_ = this->get_parameter("accel_topic").as_string();
  accel_samples_target_ = std::max(1, static_cast<int>(this->get_parameter("accel_samples").as_int()));
  
  Kp_linear_ = this->get_parameter("Kp_linear").as_double();
  Ki_linear_ = this->get_parameter("Ki_linear").as_double();
  Kd_linear_ = this->get_parameter("Kd_linear").as_double();
  Kp_angular_ = this->get_parameter("Kp_angular").as_double();
  Ki_angular_ = this->get_parameter("Ki_angular").as_double();
  Kd_angular_ = this->get_parameter("Kd_angular").as_double();
  
  controller_enabled_ = this->get_parameter("enable_controller").as_bool();
  odom_timeout_ms_ = this->get_parameter("odometry_timeout_ms").as_int();
  
  odometry_topic_ = this->get_parameter("odometry_topic").as_string();
  left_control_topic_ = this->get_parameter("left_control_topic").as_string();
  right_control_topic_ = this->get_parameter("right_control_topic").as_string();
  corrected_odom_topic_ = this->get_parameter("corrected_odometry_topic").as_string();
  
  // Initialize alignment quaternion from pitch parameter
  Eigen::Vector3d axis(0.0, 1.0, 0.0);
  align_quat_ = quat_from_axis_angle(axis, pitch_rad_);
  
  last_odom_time_ = this->now();
  
  // ============================================================
  // ROS INTERFACES
  // ============================================================
  
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 10,
    std::bind(&PoseController::odometry_callback, this, std::placeholders::_1));
  
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    accel_topic_, 10,
    std::bind(&PoseController::imu_callback, this, std::placeholders::_1));
  
  set_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/set_target_pose", 10,
    std::bind(&PoseController::set_target_callback, this, std::placeholders::_1));
  
  // Publishers for wheel velocities
  left_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(left_control_topic_, 10);
  right_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(right_control_topic_, 10);
  
  // Publisher for tilt-corrected odometry
  odom_corr_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(corrected_odom_topic_, 10);
  
  // Diagnostic publishers
  errors_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pose_control/errors", 10);
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pose_control/cmd_twist", 10);
  wheels_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pose_control/wheel_vel", 10);
  
  // Individual error publishers for rqt_plot
  err_dx_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/error_dx", 10);
  err_dy_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/error_dy", 10);
  err_e_lat_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/error_e_lat", 10);
  err_v_e_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/error_v_e", 10);
  err_dyaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/error_dyaw", 10);
  left_rps_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/left_rps", 10);
  right_rps_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/right_rps", 10);
  lookahead_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_control/lookahead_distance", 10);
  
  // Goal status publisher
  goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pose_control/goal_reached", 10);
  
  // Services
  enable_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/enable_controller",
    std::bind(&PoseController::enable_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  disable_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/disable_controller",
    std::bind(&PoseController::disable_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  // Control loop timer
  auto control_period = std::chrono::duration<double>(1.0 / control_freq_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&PoseController::control_loop, this));
  
  // ============================================================
  // STARTUP: ARM MOTORS (CLOSED-LOOP) VIA ODRIVE CAN SERVICES
  // ============================================================
  
  left_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
  right_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
  left_clear_client_ = this->create_client<std_srvs::srv::Empty>("/left/clear_errors");
  right_clear_client_ = this->create_client<std_srvs::srv::Empty>("/right/clear_errors");
  
  arm_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PoseController::attempt_arm_motors, this));
  
  // ============================================================
  // LOGGING
  // ============================================================
  
  RCLCPP_INFO(this->get_logger(), "======================================================================");
  RCLCPP_INFO(this->get_logger(), "Pose Controller Node Started");
  RCLCPP_INFO(this->get_logger(), "======================================================================");
  RCLCPP_INFO(this->get_logger(), "Robot Parameters:");
  RCLCPP_INFO(this->get_logger(), "  Wheel radius: %.3f m", wheel_radius_);
  RCLCPP_INFO(this->get_logger(), "  Wheel base: %.3f m", wheel_base_);
  RCLCPP_INFO(this->get_logger(), "  Gear ratio: %.2f", gear_ratio_);
  RCLCPP_INFO(this->get_logger(), "  Invert left/right: %s/%s",
    invert_left_ ? "true" : "false", invert_right_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Control Parameters:");
  RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_freq_);
  RCLCPP_INFO(this->get_logger(), "  Max linear velocity: %.2f m/s", max_linear_vel_);
  RCLCPP_INFO(this->get_logger(), "  Max angular velocity: %.2f rad/s", max_angular_vel_);
  RCLCPP_INFO(this->get_logger(), "  Position tolerance: %.3f m", pos_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  Orientation tolerance: %.3f rad", ori_tolerance_);
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Topics:");
  RCLCPP_INFO(this->get_logger(), "  Odometry: %s", odometry_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Left control: %s", left_control_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Right control: %s", right_control_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Services:");
  RCLCPP_INFO(this->get_logger(), "  /set_target_pose - Set new target pose");
  RCLCPP_INFO(this->get_logger(), "  /enable_controller - Enable controller");
  RCLCPP_INFO(this->get_logger(), "  /disable_controller - Disable controller");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Controller Status: %s", controller_enabled_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "======================================================================");
  
  if (!controller_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Controller is currently disabled; it will auto-enable on first target pose.");
  }
}

PoseController::~PoseController()
{
  publish_zero_velocity();
}

// ============================================================
// CALLBACK: ODOMETRY
// ============================================================

void PoseController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract and tilt-correct pose immediately
  Eigen::Vector3d raw_pos(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z
  );
  
  Eigen::Quaterniond raw_q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  );
  
  Eigen::Vector3d corr_pos = rotate_vector_by_quat(raw_pos, align_quat_);
  Eigen::Quaterniond corr_q = align_quat_ * raw_q;
  corr_q = quat_normalize(corr_q);
  
  current_x_ = corr_pos.x();
  current_y_ = corr_pos.y();
  current_yaw_ = quaternion_to_yaw(corr_q.x(), corr_q.y(), corr_q.z(), corr_q.w());
  
  // Extract velocities
  current_vx_ = msg->twist.twist.linear.x;
  current_vy_ = msg->twist.twist.linear.y;
  current_vyaw_ = msg->twist.twist.angular.z;
  
  // Mark pose as initialized
  if (!pose_initialized_) {
    pose_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
      "✓ Odometry initialized: x=%.3f, y=%.3f, yaw=%.1f°",
      current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
  }
  
  // Update last odometry time
  last_odom_time_ = this->now();
  
  // Publish corrected odometry
  try {
    auto odom_corr = std::make_unique<nav_msgs::msg::Odometry>();
    odom_corr->header.stamp = msg->header.stamp;
    odom_corr->header.frame_id = msg->header.frame_id;
    odom_corr->child_frame_id = msg->child_frame_id;
    
    odom_corr->pose.pose.position.x = current_x_;
    odom_corr->pose.pose.position.y = current_y_;
    
    Eigen::Vector3d z_pos(0.0, 0.0, msg->pose.pose.position.z);
    Eigen::Vector3d corr_z = rotate_vector_by_quat(z_pos, align_quat_);
    odom_corr->pose.pose.position.z = corr_z.z();
    
    odom_corr->pose.pose.orientation.x = corr_q.x();
    odom_corr->pose.pose.orientation.y = corr_q.y();
    odom_corr->pose.pose.orientation.z = corr_q.z();
    odom_corr->pose.pose.orientation.w = corr_q.w();
    
    // Pass-through twist (not rotated)
    odom_corr->twist = msg->twist;
    
    odom_corr_pub_->publish(std::move(odom_corr));
  } catch (...) {
    // Silently ignore publishing errors
  }
}

// ============================================================
// CALLBACK: IMU (for tilt correction)
// ============================================================

void PoseController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (accel_initialized_) {
    return;
  }
  
  Eigen::Vector3d a(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z
  );
  
  if (!a.allFinite()) {
    return;
  }
  
  // Match laser_map_rotator conventions
  a.x() = -a.x();
  a.z() = -a.z();
  
  accel_sum_ += a;
  accel_count_++;
  
  if (accel_count_ < accel_samples_target_) {
    return;
  }
  
  Eigen::Vector3d avg = accel_sum_ / static_cast<double>(accel_count_);
  double norm = avg.norm();
  
  if (norm < 1e-3) {
    warn_throttled("imu_avg", "Average acceleration too small; waiting…", 5.0);
    return;
  }
  
  // Align avg acceleration to -Z
  Eigen::Vector3d target_dir(0.0, 0.0, -1.0);
  align_quat_ = compute_alignment_quat(avg, target_dir);
  accel_initialized_ = true;
  
  RCLCPP_INFO(this->get_logger(),
    "Pose tilt correction initialized with %d samples", accel_count_);
}

// ============================================================
// CALLBACK: CONTROL LOOP
// ============================================================

void PoseController::control_loop()
{
  // Check if controller is enabled
  if (!controller_enabled_) {
    publish_zero_velocity();
    return;
  }
  
  // Check if odometry is initialized
  if (!pose_initialized_) {
    warn_throttled("waiting_odom", "⚠️  Waiting for odometry from Fast-LIO2...", 5.0);
    publish_zero_velocity();
    return;
  }
  
  // Check odometry timeout
  auto time_since_odom = (this->now() - last_odom_time_).nanoseconds() / 1e6;
  if (time_since_odom > odom_timeout_ms_) {
    std::string msg = "⚠️  Odometry timeout (" + std::to_string(time_since_odom) +
                      " ms > " + std::to_string(odom_timeout_ms_) + " ms). Stopping motors.";
    warn_throttled("odom_timeout", msg, 2.0);
    publish_zero_velocity();
    return;
  }
  
  // Check if target is set
  if (!has_target_) {
    publish_zero_velocity();
    return;
  }
  
  // ============================================================
  // CALL CONTROL FUNCTION
  // ============================================================
  
  auto [linear_vel, angular_vel, diag] = control_function(
    current_x_, current_y_, current_yaw_,
    target_x_, target_y_, target_yaw_
  );
  
  // ============================================================
  // ACCELERATION LIMITING
  // ============================================================
  
  double dt = 1.0 / control_freq_;
  
  // Limit linear acceleration
  double max_linear_change = max_linear_accel_ * dt;
  double linear_change = linear_vel - last_linear_vel_;
  if (std::abs(linear_change) > max_linear_change) {
    linear_vel = last_linear_vel_ + std::copysign(max_linear_change, linear_change);
  }
  
  // Limit angular acceleration
  double max_angular_change = max_angular_accel_ * dt;
  double angular_change = angular_vel - last_angular_vel_;
  if (std::abs(angular_change) > max_angular_change) {
    angular_vel = last_angular_vel_ + std::copysign(max_angular_change, angular_change);
  }
  
  // Store for next iteration
  last_linear_vel_ = linear_vel;
  last_angular_vel_ = angular_vel;
  
  // ============================================================
  // CONVERT TO WHEEL VELOCITIES
  // ============================================================
  
  auto [left_vel, right_vel] = diff_drive_inverse_kinematics(linear_vel, angular_vel);
  
  // Apply minimum velocity threshold
  if (std::abs(left_vel) > 0.0 && std::abs(left_vel) < min_wheel_rps_) {
    left_vel = std::copysign(min_wheel_rps_, left_vel);
  }
  if (std::abs(right_vel) > 0.0 && std::abs(right_vel) < min_wheel_rps_) {
    right_vel = std::copysign(min_wheel_rps_, right_vel);
  }
  
  // Store for logging
  left_wheel_velocity_ = left_vel;
  right_wheel_velocity_ = right_vel;
  
  // ============================================================
  // PUBLISH COMMANDS
  // ============================================================
  
  publish_wheel_velocities(left_vel, right_vel);
  
  // ============================================================
  // PUBLISH DIAGNOSTICS
  // ============================================================
  
  try {
    // Errors
    auto err_msg = std_msgs::msg::Float64MultiArray();
    err_msg.data = {diag.dx, diag.dy, diag.e_lat, diag.v_e, diag.dyaw};
    errors_pub_->publish(err_msg);
    
    // Individual errors
    auto dx_msg = std_msgs::msg::Float64(); dx_msg.data = diag.dx; err_dx_pub_->publish(dx_msg);
    auto dy_msg = std_msgs::msg::Float64(); dy_msg.data = diag.dy; err_dy_pub_->publish(dy_msg);
    auto el_msg = std_msgs::msg::Float64(); el_msg.data = diag.e_lat; err_e_lat_pub_->publish(el_msg);
    auto ve_msg = std_msgs::msg::Float64(); ve_msg.data = diag.v_e; err_v_e_pub_->publish(ve_msg);
    auto yw_msg = std_msgs::msg::Float64(); yw_msg.data = diag.dyaw; err_dyaw_pub_->publish(yw_msg);
    
    // Command v, w
    auto tw = geometry_msgs::msg::Twist();
    tw.linear.x = linear_vel;
    tw.angular.z = angular_vel;
    cmd_pub_->publish(tw);
    
    // Wheel velocities
    auto wheels_msg = std_msgs::msg::Float64MultiArray();
    wheels_msg.data = {left_vel, right_vel};
    wheels_pub_->publish(wheels_msg);
    
    // Individual wheel velocities
    auto l_msg = std_msgs::msg::Float64(); l_msg.data = left_vel; left_rps_pub_->publish(l_msg);
    auto r_msg = std_msgs::msg::Float64(); r_msg.data = right_vel; right_rps_pub_->publish(r_msg);
    
    // Lookahead distance
    auto lookahead_msg = std_msgs::msg::Float64(); 
    lookahead_msg.data = diag.lookahead; 
    lookahead_pub_->publish(lookahead_msg);
  } catch (...) {
    // Silently ignore publishing errors
  }
  
  control_counter_++;
}

// ============================================================
// CONTROL FUNCTION
// ============================================================

std::tuple<double, double, PoseController::ControlDiagnostics>
PoseController::control_function(
  double current_x, double current_y, double current_yaw,
  double target_x, double target_y, double target_yaw)
{
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  current_yaw = normalize_angle(current_yaw);
  
  // Compute distance to target
  double dx_target = target_x - current_x;
  double dy_target = target_y - current_y;
  double distance_to_target = std::sqrt(dx_target * dx_target + dy_target * dy_target);
  
  double dyaw = normalize_angle(target_yaw - current_yaw);
  
  // ============================================================
  // GOAL REACHED CHECK
  // ============================================================
  
  if (distance_to_target < pos_tolerance_ && std::abs(dyaw) < ori_tolerance_) {
    // Goal reached!
    if (!goal_reached_) {
      goal_reached_ = true;
      auto goal_msg = std_msgs::msg::Bool();
      goal_msg.data = true;
      goal_reached_pub_->publish(goal_msg);
      RCLCPP_INFO(this->get_logger(), 
        "✓ Goal reached! Distance: %.3fm, Yaw error: %.1f°",
        distance_to_target, std::abs(dyaw) * 180.0 / M_PI);
    }
    
    ControlDiagnostics diag{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return {0.0, 0.0, diag};
  }
  
  goal_reached_ = false;
  
  // ============================================================
  // PURE PURSUIT ALGORITHM
  // ============================================================
  
  // Adaptive lookahead distance based on speed
  double current_speed = std::sqrt(current_vx_ * current_vx_ + current_vy_ * current_vy_);
  double adaptive_lookahead = lookahead_distance_ + lookahead_gain_ * current_speed;
  adaptive_lookahead = std::clamp(adaptive_lookahead, 
                                   min_lookahead_distance_, 
                                   max_lookahead_distance_);
  
  // Limit lookahead to remaining distance
  adaptive_lookahead = std::min(adaptive_lookahead, distance_to_target);
  
  // Find lookahead point along the path to target
  double lookahead_x, lookahead_y;
  
  if (distance_to_target < pos_tolerance_) {
    // Very close - aim at target directly
    lookahead_x = target_x;
    lookahead_y = target_y;
  } else {
    // Compute lookahead point along line to target
    double direction_x = dx_target / distance_to_target;
    double direction_y = dy_target / distance_to_target;
    lookahead_x = current_x + direction_x * adaptive_lookahead;
    lookahead_y = current_y + direction_y * adaptive_lookahead;
  }
  
  // Transform lookahead point to robot frame
  double dx_lookahead = lookahead_x - current_x;
  double dy_lookahead = lookahead_y - current_y;
  
  // Rotate to robot frame
  double local_x = std::cos(-current_yaw) * dx_lookahead - std::sin(-current_yaw) * dy_lookahead;
  double local_y = std::sin(-current_yaw) * dx_lookahead + std::cos(-current_yaw) * dy_lookahead;
  
  // Pure pursuit curvature formula: κ = 2y / L²
  double L_squared = local_x * local_x + local_y * local_y;
  double curvature = 0.0;
  
  if (L_squared > 1e-6) {
    curvature = 2.0 * local_y / L_squared;
  }
  
  // ============================================================
  // VELOCITY CONTROL
  // ============================================================
  
  // Linear velocity - slow down as we approach target
  double speed_scale = 1.0;
  
  if (distance_to_target < 0.5) {
    // Slow down in last 0.5m
    speed_scale = std::max(0.3, distance_to_target / 0.5);
  }
  
  // Also slow down for sharp turns
  double turn_scale = 1.0 / (1.0 + 2.0 * std::abs(curvature));
  speed_scale *= turn_scale;
  
  linear_vel = max_linear_vel_ * speed_scale;
  
  // Angular velocity from curvature: ω = v * κ
  angular_vel = linear_vel * curvature;
  
  // ============================================================
  // FINAL ORIENTATION ADJUSTMENT
  // ============================================================
  
  // When close to position, prioritize orientation
  if (distance_to_target < pos_tolerance_ * 2.0) {
    // Slow down linear motion
    linear_vel *= 0.3;
    
    // Focus on orientation
    angular_vel = Kp_angular_ * dyaw;
  }
  
  // Clamp angular velocity
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
  
  // ============================================================
  // DIAGNOSTICS
  // ============================================================
  
  // Lateral error (perpendicular to robot heading)
  double e_lat = -std::sin(current_yaw) * dx_target + std::cos(current_yaw) * dy_target;
  
  // Forward error (along robot heading)
  double v_e = std::cos(current_yaw) * dx_target + std::sin(current_yaw) * dy_target;
  
  ControlDiagnostics diag{
    dx_target, 
    dy_target, 
    e_lat, 
    v_e, 
    dyaw,
    adaptive_lookahead,
    curvature
  };
  
  return {linear_vel, angular_vel, diag};
}

// ============================================================
// KINEMATICS: DIFFERENTIAL DRIVE
// ============================================================

std::pair<double, double> PoseController::diff_drive_inverse_kinematics(
  double linear_vel, double angular_vel)
{
  // Differential drive kinematics
  double v_left = linear_vel - (angular_vel * wheel_base_ / 2.0);
  double v_right = linear_vel + (angular_vel * wheel_base_ / 2.0);
  
  // Convert m/s to wheel turns/s
  double wheel_circumference = 2.0 * M_PI * wheel_radius_;
  double left_wheel_rps = v_left / wheel_circumference;
  double right_wheel_rps = v_right / wheel_circumference;
  
  // Apply gear ratio and velocity multiplier
  left_wheel_rps *= gear_ratio_ * velocity_multiplier_;
  right_wheel_rps *= gear_ratio_ * velocity_multiplier_;
  
  return {left_wheel_rps, right_wheel_rps};
}

// ============================================================
// PUBLISHING HELPERS
// ============================================================

void PoseController::publish_wheel_velocities(double left_rps, double right_rps)
{
  // Apply motor inversion
  double left_cmd = invert_left_ ? -left_rps : left_rps;
  double right_cmd = invert_right_ ? -right_rps : right_rps;
  
  // Create left motor command
  auto left_msg = odrive_can::msg::ControlMessage();
  left_msg.control_mode = 2;  // VELOCITY_CONTROL
  left_msg.input_mode = 2;    // VEL_RAMP
  left_msg.input_vel = static_cast<float>(left_cmd);
  left_msg.input_torque = 0.0f;
  left_msg.input_pos = 0.0f;
  
  // Create right motor command
  auto right_msg = odrive_can::msg::ControlMessage();
  right_msg.control_mode = 2;  // VELOCITY_CONTROL
  right_msg.input_mode = 2;    // VEL_RAMP
  right_msg.input_vel = static_cast<float>(right_cmd);
  right_msg.input_torque = 0.0f;
  right_msg.input_pos = 0.0f;
  
  // Publish
  left_pub_->publish(left_msg);
  right_pub_->publish(right_msg);
}

void PoseController::publish_zero_velocity()
{
  publish_wheel_velocities(0.0, 0.0);
}

// ============================================================
// SERVICE CALLBACKS
// ============================================================

void PoseController::set_target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(this->get_logger(), "set_target_pose requires 4 elements: [x,y,z,yaw]");
    return;
  }
  
  target_x_ = msg->data[0];
  target_y_ = msg->data[1];
  // z currently unused in control
  target_yaw_ = msg->data[3];
  
  has_target_ = true;
  goal_reached_ = false;
  
  // Reset velocity tracking for smooth start
  last_linear_vel_ = 0.0;
  last_angular_vel_ = 0.0;
  
  // Auto-enable controller on first target if disabled
  if (!controller_enabled_) {
    controller_enabled_ = true;
    RCLCPP_INFO(this->get_logger(), "✓ Controller auto-enabled on target reception");
  }
  
  // Compute distance for info
  double dx = target_x_ - current_x_;
  double dy = target_y_ - current_y_;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  RCLCPP_INFO(this->get_logger(),
    "🎯 New target set: x=%.2fm, y=%.2fm, yaw=%.1f° (distance: %.2fm)",
    target_x_, target_y_, target_yaw_ * 180.0 / M_PI, distance);
}

void PoseController::enable_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  controller_enabled_ = true;
  response->success = true;
  response->message = "✓ Controller ENABLED";
  RCLCPP_INFO(this->get_logger(), "✓ Controller ENABLED");
}

void PoseController::disable_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  controller_enabled_ = false;
  publish_zero_velocity();
  response->success = true;
  response->message = "✓ Controller DISABLED";
  RCLCPP_INFO(this->get_logger(), "✓ Controller DISABLED");
}

// ============================================================
// QUATERNION & TILT CORRECTION UTILITIES
// ============================================================

Eigen::Quaterniond PoseController::quat_normalize(const Eigen::Quaterniond& q)
{
  double norm = q.norm();
  if (norm <= 1e-12) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  // w, x, y, z
  }
  return Eigen::Quaterniond(q.w() / norm, q.x() / norm, q.y() / norm, q.z() / norm);
}

Eigen::Quaterniond PoseController::quat_from_axis_angle(const Eigen::Vector3d& axis, double angle_rad)
{
  double norm = axis.norm();
  if (norm <= 1e-12) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  
  Eigen::Vector3d ax = axis / norm;
  double s = std::sin(angle_rad * 0.5);
  double c = std::cos(angle_rad * 0.5);
  
  return Eigen::Quaterniond(c, ax.x() * s, ax.y() * s, ax.z() * s);
}

Eigen::Vector3d PoseController::rotate_vector_by_quat(const Eigen::Vector3d& v, const Eigen::Quaterniond& q)
{
  // Using quaternion rotation: v' = q * v * q^-1
  return q * v;
}

Eigen::Quaterniond PoseController::compute_alignment_quat(
  const Eigen::Vector3d& from_vec, const Eigen::Vector3d& to_vec)
{
  Eigen::Vector3d v1 = from_vec.normalized();
  Eigen::Vector3d v2 = to_vec.normalized();
  
  double cos_theta = v1.dot(v2);
  cos_theta = std::clamp(cos_theta, -1.0, 1.0);
  
  if (cos_theta > 1.0 - 1e-9) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  
  if (cos_theta < -1.0 + 1e-9) {
    // 180 degree rotation
    Eigen::Vector3d axis(1.0, 0.0, 0.0);
    if (std::abs(v1.x()) > 0.9) {
      axis = Eigen::Vector3d(0.0, 1.0, 0.0);
    }
    axis = axis - v1 * v1.dot(axis);
    if (axis.norm() < 1e-12) {
      axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    axis.normalize();
    return quat_from_axis_angle(axis, M_PI);
  }
  
  Eigen::Vector3d axis = v1.cross(v2);
  double s = std::sqrt((1.0 + cos_theta) * 2.0);
  double invs = 1.0 / s;
  double w = 0.5 * s;
  double x = axis.x() * invs;
  double y = axis.y() * invs;
  double z = axis.z() * invs;
  
  return quat_normalize(Eigen::Quaterniond(w, x, y, z));
}

double PoseController::quaternion_to_yaw(double qx, double qy, double qz, double qw)
{
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

double PoseController::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// ============================================================
// MOTOR ARMING
// ============================================================

void PoseController::attempt_arm_motors()
{
  if (arm_attempts_ >= arm_max_attempts_) {
    arm_timer_->cancel();
    return;
  }
  arm_attempts_++;
  
  // Ensure service availability
  if (!left_axis_client_->service_is_ready() ||
      !right_axis_client_->service_is_ready() ||
      !left_clear_client_->service_is_ready() ||
      !right_clear_client_->service_is_ready()) {
    warn_throttled("odrive_services", "Waiting for ODrive CAN services to be ready...", 5.0);
    return;
  }
  
  try {
    // Clear errors first
    auto clear_req = std::make_shared<std_srvs::srv::Empty::Request>();
    left_clear_client_->async_send_request(clear_req);
    right_clear_client_->async_send_request(clear_req);
    
    // Request CLOSED_LOOP_CONTROL (8 per ODrive enums)
    auto req_left = std::make_shared<odrive_can::srv::AxisState::Request>();
    req_left->axis_requested_state = 8;
    auto req_right = std::make_shared<odrive_can::srv::AxisState::Request>();
    req_right->axis_requested_state = 8;
    
    left_axis_client_->async_send_request(req_left);
    right_axis_client_->async_send_request(req_right);
    
    RCLCPP_INFO(this->get_logger(), "Arming ODrive axes (CLOSED_LOOP_CONTROL requested)");
    
    // Stop timer after successful dispatch
    arm_timer_->cancel();
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Arm attempt failed: %s", e.what());
  }
}

// ============================================================
// LOGGING UTILITIES
// ============================================================

void PoseController::warn_throttled(const std::string& key, const std::string& message, double period_sec)
{
  int64_t now_ns = this->now().nanoseconds();
  int64_t last_ns = last_log_time_ns_[key];
  
  if (last_ns == 0 || (now_ns - last_ns) >= static_cast<int64_t>(period_sec * 1e9)) {
    last_log_time_ns_[key] = now_ns;
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  }
}

}  // namespace pilot_control

// ============================================================
// MAIN
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pilot_control::PoseController>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  
  // Send zero velocity before shutdown
  node.reset();
  rclcpp::shutdown();
  
  return 0;
}

