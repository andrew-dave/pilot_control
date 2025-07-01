#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <signal.h>
#include <atomic>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_shutdown_requested = true;
    }
}

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController()
        : Node("diff_drive_controller"), last_time_(this->get_clock()->now()) {
        // ---------------- Parameters ----------------
        this->declare_parameter<double>("wheel_radius", 0.063);
        this->declare_parameter<double>("wheel_base", 0.315);
        this->declare_parameter<double>("gear_ratio", 10.0);          // NEW
        this->declare_parameter<double>("velocity_multiplier", 1.0);
        this->declare_parameter<double>("turn_speed_multiplier", 1.0);
        this->declare_parameter<double>("velocity_deadband", 0.01);
        this->declare_parameter<int>("stop_timeout_ms", 500);

        wheel_radius_         = this->get_parameter("wheel_radius").as_double();
        wheel_base_           = this->get_parameter("wheel_base").as_double();
        gear_ratio_           = this->get_parameter("gear_ratio").as_double();   // NEW
        velocity_multiplier_  = this->get_parameter("velocity_multiplier").as_double();
        turn_speed_multiplier_= this->get_parameter("turn_speed_multiplier").as_double();
        velocity_deadband_    = this->get_parameter("velocity_deadband").as_double();
        stop_timeout_ms_      = this->get_parameter("stop_timeout_ms").as_int();

        RCLCPP_INFO(this->get_logger(), "Wheel Radius: %.3f m, Wheel Base: %.3f m, Gear Ratio: %.1f", wheel_radius_, wheel_base_, gear_ratio_);

        // ---------------- Interfaces ----------------
        odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        left_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/left/control_message", 10);
        right_motor_pub_= this->create_publisher<odrive_can::msg::ControlMessage>("/right/control_message", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        left_axis_client_  = this->create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/right/request_axis_state");

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));
        left_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/left/controller_status", 10, std::bind(&DiffDriveController::left_status_callback, this, std::placeholders::_1));
        right_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/right/controller_status", 10, std::bind(&DiffDriveController::right_status_callback, this, std::placeholders::_1));

        odom_timer_ = this->create_wall_timer(50ms, std::bind(&DiffDriveController::update_odometry, this));
        arm_timer_  = this->create_wall_timer(200ms, [this]() {
            this->arm_motors();
            this->arm_timer_->cancel();
        });
        last_cmd_time_ = this->get_clock()->now();
    }

    // ---------------- Motor State Helpers ----------------
    void arm_motors() {
        RCLCPP_INFO(this->get_logger(), "Waiting for ODrive services to become available...");
        if (!left_axis_client_->wait_for_service(3s) || !right_axis_client_->wait_for_service(3s)) {
            RCLCPP_FATAL(this->get_logger(), "ODrive services not available after waiting. Shutting down.");
            g_shutdown_requested = true;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Services found. Arming motors (closed loop)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void disarm_motors() {
        RCLCPP_INFO(this->get_logger(), "Disarming motors (IDLE)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

private:
    // ---------------- Command Velocity ----------------
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_vel  = msg->linear.x;   // m/s
        double angular_vel = msg->angular.z;  // rad/s
        last_cmd_time_     = this->get_clock()->now();

        // Dead‑band -> zero torque stop
        if (std::abs(linear_vel) < velocity_deadband_ && std::abs(angular_vel) < velocity_deadband_) {
            send_zero_torque();
            return;
        }

        // Skid‑steer kinematics – wheel linear speeds (m/s)
        double right_wheel_mps = linear_vel + (angular_vel * wheel_base_ / 2.0);
        double left_wheel_mps  = linear_vel - (angular_vel * wheel_base_ / 2.0);

        // Apply turn speed multiplier to rotational component only
        double rot_term = (angular_vel * wheel_base_ / 2.0) * (turn_speed_multiplier_ - 1.0);
        right_wheel_mps += rot_term;
        left_wheel_mps  -= rot_term;

        // Convert to **wheel** turns/s
        double wheel_circumference = 2.0 * M_PI * wheel_radius_;
        double right_wheel_turns = right_wheel_mps / wheel_circumference;
        double left_wheel_turns  = left_wheel_mps  / wheel_circumference;

        // Convert to **motor** turns/s via gearbox
        double right_motor_turns_per_sec = right_wheel_turns * gear_ratio_ * velocity_multiplier_;
        double left_motor_turns_per_sec  = left_wheel_turns  * gear_ratio_ * velocity_multiplier_;

        // Publish velocity control
        odrive_can::msg::ControlMessage right_msg;
        right_msg.control_mode = 2; // VELOCITY_CONTROL
        right_msg.input_mode   = 2; // VEL_RAMP
        right_msg.input_vel    = right_motor_turns_per_sec;
        right_msg.input_torque = 0.0;

        odrive_can::msg::ControlMessage left_msg = right_msg;
        left_msg.input_vel = -left_motor_turns_per_sec;  // left motor inverted

        right_motor_pub_->publish(right_msg);
        left_motor_pub_ ->publish(left_msg);
    }

    // ---------------- Feedback Callbacks ----------------
    void left_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_left_vel_ = msg->vel_estimate;   // motor turns/s
    }
    void right_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_right_vel_ = msg->vel_estimate;  // motor turns/s (same sign as cmd)
    }

    // ---------------- Odometry ----------------
    void update_odometry() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        double wheel_circumference = 2.0 * M_PI * wheel_radius_;

        // Convert **motor** turns/s to **wheel** m/s
        double left_wheel_mps  = (current_left_vel_ / gear_ratio_)  * wheel_circumference;
        double right_wheel_mps = (current_right_vel_ / gear_ratio_) * wheel_circumference;

        double linear_vel  = (left_wheel_mps + right_wheel_mps) / 2.0;     // m/s
        double angular_vel = (right_wheel_mps - left_wheel_mps) / wheel_base_; // rad/s

        x_     += linear_vel * std::cos(theta_) * dt;
        y_     += linear_vel * std::sin(theta_) * dt;
        theta_ += angular_vel * dt;
        theta_  = std::atan2(std::sin(theta_), std::cos(theta_));           // wrap

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        tf2::Quaternion q; q.setRPY(0,0,theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x  = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header       = odom_msg.header;
        tf_msg.child_frame_id            = "base_link";
        tf_msg.transform.translation.x   = x_;
        tf_msg.transform.translation.y   = y_;
        tf_msg.transform.rotation        = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        last_time_ = current_time;
    }

    // ---------------- Helpers ----------------
    void send_zero_torque(){
        odrive_can::msg::ControlMessage msg;
        msg.control_mode = 1; // TORQUE_CONTROL
        msg.input_mode   = 1; // PASSTHROUGH
        msg.input_vel    = 0.0;
        msg.input_torque = 0.0;
        left_motor_pub_->publish(msg);
        right_motor_pub_->publish(msg);
    }

    // ---------------- Members ----------------
    double wheel_radius_, wheel_base_, gear_ratio_, velocity_multiplier_, turn_speed_multiplier_, velocity_deadband_;
    int stop_timeout_ms_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr left_motor_pub_, right_motor_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_, right_axis_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_status_sub_, right_status_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_, arm_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time last_time_, last_cmd_time_;
    double current_left_vel_  = 0.0;
    double current_right_vel_ = 0.0;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = std::make_shared<DiffDriveController>();
    while (!g_shutdown_requested) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    node->disarm_motors();
    std::this_thread::sleep_for(300ms);
    rclcpp::shutdown();
    return 0;
}
