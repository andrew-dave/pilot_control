#ifndef PILOT_CONTROL__POSE_CONTROLLER_HPP_
#define PILOT_CONTROL__POSE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <unordered_map>

namespace pilot_control
{

class PoseController : public rclcpp::Node
{
public:
  PoseController();
  ~PoseController();

private:
  // ============================================================
  // PARAMETERS
  // ============================================================
  
  // Robot kinematics
  double wheel_radius_;
  double wheel_base_;
  double gear_ratio_;
  bool invert_left_;
  bool invert_right_;
  double velocity_multiplier_;
  
  // Control parameters
  double control_freq_;
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;
  double max_angular_accel_;
  double pos_tolerance_;
  double ori_tolerance_;
  double min_wheel_rps_;
  
  // Pure pursuit parameters
  double lookahead_distance_;
  double lookahead_gain_;
  double min_lookahead_distance_;
  double max_lookahead_distance_;
  
  // Tilt correction parameters
  double pitch_rad_;
  std::string accel_topic_;
  int accel_samples_target_;
  
  // PID gains
  double Kp_linear_;
  double Ki_linear_;
  double Kd_linear_;
  double Kp_angular_;
  double Ki_angular_;
  double Kd_angular_;
  
  // Safety parameters
  bool controller_enabled_;
  int odom_timeout_ms_;
  
  // Topic names
  std::string odometry_topic_;
  std::string left_control_topic_;
  std::string right_control_topic_;
  std::string corrected_odom_topic_;
  
  // ============================================================
  // STATE VARIABLES
  // ============================================================
  
  // Current pose from Fast-LIO2
  double current_x_;
  double current_y_;
  double current_yaw_;
  double current_vx_;
  double current_vy_;
  double current_vyaw_;
  bool pose_initialized_;
  rclcpp::Time last_odom_time_;
  
  // Throttled logging state
  std::unordered_map<std::string, int64_t> last_log_time_ns_;
  
  // Tilt correction state
  bool accel_initialized_;
  Eigen::Vector3d accel_sum_;
  int accel_count_;
  Eigen::Quaterniond align_quat_;
  
  // Target pose
  double target_x_;
  double target_y_;
  double target_yaw_;
  bool has_target_;
  
  // Control outputs
  double left_wheel_velocity_;
  double right_wheel_velocity_;
  
  // Previous velocities for acceleration limiting
  double last_linear_vel_;
  double last_angular_vel_;
  
  // Control counter for logging
  int control_counter_;
  
  // Goal tracking
  bool goal_reached_;
  
  // Motor arming state
  int arm_attempts_;
  int arm_max_attempts_;
  
  // ============================================================
  // ROS INTERFACES
  // ============================================================
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_target_sub_;
  
  // Publishers
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr left_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr right_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_corr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr errors_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheels_pub_;
  
  // Individual error publishers for rqt_plot
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_dx_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_dy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_e_lat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_v_e_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_dyaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lookahead_pub_;
  
  // Goal status publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  
  // Service clients for ODrive
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr right_axis_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr left_clear_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr right_clear_client_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr arm_timer_;
  
  // ============================================================
  // CALLBACK METHODS
  // ============================================================
  
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void control_loop();
  void set_target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  void enable_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void disable_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // ============================================================
  // CONTROL FUNCTIONS
  // ============================================================
  
  struct ControlDiagnostics {
    double dx;
    double dy;
    double e_lat;
    double v_e;
    double dyaw;
    double lookahead;
    double curvature;
  };
  
  std::tuple<double, double, ControlDiagnostics> control_function(
    double current_x, double current_y, double current_yaw,
    double target_x, double target_y, double target_yaw);
  
  std::pair<double, double> diff_drive_inverse_kinematics(
    double linear_vel, double angular_vel);
  
  // ============================================================
  // PUBLISHING HELPERS
  // ============================================================
  
  void publish_wheel_velocities(double left_rps, double right_rps);
  void publish_zero_velocity();
  
  // ============================================================
  // QUATERNION & TILT CORRECTION UTILITIES
  // ============================================================
  
  static Eigen::Quaterniond quat_normalize(const Eigen::Quaterniond& q);
  static Eigen::Quaterniond quat_from_axis_angle(const Eigen::Vector3d& axis, double angle_rad);
  static Eigen::Vector3d rotate_vector_by_quat(const Eigen::Vector3d& v, const Eigen::Quaterniond& q);
  static Eigen::Quaterniond compute_alignment_quat(const Eigen::Vector3d& from_vec, const Eigen::Vector3d& to_vec);
  static double quaternion_to_yaw(double qx, double qy, double qz, double qw);
  static double normalize_angle(double angle);
  
  // ============================================================
  // MOTOR ARMING
  // ============================================================
  
  void attempt_arm_motors();
  
  // ============================================================
  // LOGGING UTILITIES
  // ============================================================
  
  void warn_throttled(const std::string& key, const std::string& message, double period_sec);
};

}  // namespace pilot_control

#endif  // PILOT_CONTROL__POSE_CONTROLLER_HPP_

