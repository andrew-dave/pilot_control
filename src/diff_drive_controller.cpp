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
        this->declare_parameter<double>("wheel_radius", 0.063);
        this->declare_parameter<double>("wheel_base", 0.315);
        this->declare_parameter<double>("velocity_multiplier", 1.0);
        this->declare_parameter<double>("turn_speed_multiplier", 1.0);
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        velocity_multiplier_ = this->get_parameter("velocity_multiplier").as_double();
        turn_speed_multiplier_ = this->get_parameter("turn_speed_multiplier").as_double();
        RCLCPP_INFO(this->get_logger(), "Wheel Radius: %f m, Wheel Base: %f m", wheel_radius_, wheel_base_);
        RCLCPP_INFO(this->get_logger(), "Velocity Multiplier: %f", velocity_multiplier_);
        RCLCPP_INFO(this->get_logger(), "Turn Speed Multiplier: %f", turn_speed_multiplier_);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        left_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/left/control_message", 10);
        right_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/right/control_message", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        left_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));
        left_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/left/controller_status", 10, std::bind(&DiffDriveController::left_status_callback, this, std::placeholders::_1));
        right_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/right/controller_status", 10, std::bind(&DiffDriveController::right_status_callback, this, std::placeholders::_1));
        odom_timer_ = this->create_wall_timer(50ms, std::bind(&DiffDriveController::update_odometry, this));
        arm_timer_ = this->create_wall_timer(200ms, [this]() {
            this->arm_motors();
            this->arm_timer_->cancel();
        });
    }

    void arm_motors() {
        RCLCPP_INFO(this->get_logger(), "Waiting for ODrive services to become available...");
        if (!left_axis_client_->wait_for_service(3s) || !right_axis_client_->wait_for_service(3s)) {
            RCLCPP_FATAL(this->get_logger(), "ODrive services not available after waiting. Shutting down.");
            g_shutdown_requested = true;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Services found. Arming motors (requesting closed loop control)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // AXIS_STATE_CLOSED_LOOP_CONTROL
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void disarm_motors() {
        RCLCPP_INFO(this->get_logger(), "Sending disarm command (IDLE)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // AXIS_STATE_IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;
        
        // Calculate wheel velocities in m/s
        double right_wheel_mps = linear_vel + (angular_vel * wheel_base_ / 2.0);
        double left_wheel_mps = linear_vel - (angular_vel * wheel_base_ / 2.0);
        
        // Apply turn speed multiplier to the angular component only
        double angular_component_right = (angular_vel * wheel_base_ / 2.0) * turn_speed_multiplier_;
        double angular_component_left = -(angular_vel * wheel_base_ / 2.0) * turn_speed_multiplier_;
        
        // Recalculate with turn speed multiplier
        right_wheel_mps = linear_vel + angular_component_right;
        left_wheel_mps = linear_vel + angular_component_left;
        
        // Convert to motor turns per second
        double wheel_circumference = 2.0 * M_PI * wheel_radius_;
        double right_motor_turns_per_sec = right_wheel_mps / wheel_circumference;
        double left_motor_turns_per_sec = left_wheel_mps / wheel_circumference;
        
        // Apply overall velocity multiplier
        right_motor_turns_per_sec *= velocity_multiplier_;
        left_motor_turns_per_sec *= velocity_multiplier_;
        
        // Debug output for turning
        if (std::abs(angular_vel) > 0.1) {
            RCLCPP_INFO(this->get_logger(), 
                "Turn: ang_vel=%.2f, left_mps=%.2f, right_mps=%.2f, left_turns=%.2f, right_turns=%.2f", 
                angular_vel, left_wheel_mps, right_wheel_mps, left_motor_turns_per_sec, right_motor_turns_per_sec);
        }
        
        auto left_msg = odrive_can::msg::ControlMessage();
        left_msg.control_mode = 2; // VELOCITY_CONTROL
        left_msg.input_mode = 2;   // VEL_RAMP
        left_msg.input_vel = -left_motor_turns_per_sec;  // Left motor inverted
        
        auto right_msg = odrive_can::msg::ControlMessage();
        right_msg.control_mode = 2; // VELOCITY_CONTROL
        right_msg.input_mode = 2;   // VEL_RAMP
        right_msg.input_vel = right_motor_turns_per_sec;
        
        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }

    void left_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_left_vel_ = msg->vel_estimate;
    }

    void right_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_right_vel_ = -(msg->vel_estimate);
    }

    void update_odometry() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        double wheel_circumference = 2.0 * M_PI * wheel_radius_;
        
        // Convert motor turns/sec back to wheel m/s
        // Note: current_left_vel_ and current_right_vel_ are already in turns/sec from the motor feedback
        double left_wheel_mps = current_left_vel_ * wheel_circumference;
        double right_wheel_mps = current_right_vel_ * wheel_circumference;
        
        // Calculate robot linear and angular velocity
        double linear_vel = (left_wheel_mps + right_wheel_mps) / 2.0;
        double angular_vel = (right_wheel_mps - left_wheel_mps) / wheel_base_;
        
        // Update position
        x_ += linear_vel * cos(theta_) * dt;
        y_ += linear_vel * sin(theta_) * dt;
        theta_ += angular_vel * dt;
        
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;
        odom_pub_->publish(odom_msg);
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
        last_time_ = current_time;
    }

    double wheel_radius_, wheel_base_, velocity_multiplier_, turn_speed_multiplier_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr left_motor_pub_, right_motor_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_, right_axis_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_status_sub_, right_status_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_, arm_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time last_time_;
    double current_left_vel_ = 0.0, current_right_vel_ = 0.0;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
};

int main(int argc, char * argv[]) {
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