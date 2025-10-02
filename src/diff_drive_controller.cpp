#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
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
#include <memory>

using namespace std::chrono_literals;

std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_shutdown_requested = true;
    }
}

// Named ODrive modes to avoid magic numbers
namespace odrv {
  enum ControlMode { TORQUE_CONTROL = 1, VELOCITY_CONTROL = 2, POSITION_CONTROL = 3 };
  enum InputMode   { PASSTHROUGH    = 1, VEL_RAMP         = 2, TRAP_TRAJ       = 5 };
}

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController()
        : Node("diff_drive_controller"),
          last_time_(this->get_clock()->now())
    {
        // ---------------- Parameters ----------------
        // Drive geometry
        this->declare_parameter<double>("wheel_radius", 0.072); // 144 mm dia wheels -> 0.072 m radius
        this->declare_parameter<double>("wheel_base",   0.35);
        this->declare_parameter<double>("gear_ratio",   10.0);

        // Drive behavior
        this->declare_parameter<double>("velocity_multiplier",    1.0);
        this->declare_parameter<double>("turn_speed_multiplier",  1.0);
        this->declare_parameter<double>("velocity_deadband",      0.001); // m/s (and rad/s used comparably)
        this->declare_parameter<int>("stop_timeout_ms",           500);

        // Motor inversion (signs)
        this->declare_parameter<bool>("invert_left",   false);   // adjust to your wiring
        this->declare_parameter<bool>("invert_right",  true);
        this->declare_parameter<bool>("invert_third",  false);

        // GPR (node 2) kinematics (60 mm dia virtual wheel by default, direct 1:1)
        this->declare_parameter<double>("third_wheel_radius", 0.03); // 60 mm dia -> 0.03 m radius
        this->declare_parameter<double>("third_gear_ratio",   1.0);  // direct

        // GPR gating
        this->declare_parameter<bool>("gpr_forward_only", false);     // rotate only when moving forward
        this->declare_parameter<bool>("gpr_gate_turns",   false);    // optionally stop during turns
        this->declare_parameter<double>("gpr_turn_rate_thresh", 0.25); // rad/s
        // Allow GPR to continue during residual coast after cmd_vel stops
        this->declare_parameter<bool>("gpr_follow_coast", false);
        this->declare_parameter<int>("gpr_coast_timeout_ms", 1000);
        this->declare_parameter<double>("gpr_min_vx_follow", 0.02);

        // Fast-LIO odometry input (for GPR velocity drive)
        this->declare_parameter<bool>("gpr_use_fastlio_odom", true);
        this->declare_parameter<std::string>("fastlio_odom_topic", "/Odometry");
        this->declare_parameter<int>("fastlio_timeout_ms", 500);

        // Read params
        wheel_radius_          = this->get_parameter("wheel_radius").as_double();
        wheel_base_            = this->get_parameter("wheel_base").as_double();
        gear_ratio_            = this->get_parameter("gear_ratio").as_double();
        velocity_multiplier_   = this->get_parameter("velocity_multiplier").as_double();
        turn_speed_multiplier_ = this->get_parameter("turn_speed_multiplier").as_double();
        velocity_deadband_     = this->get_parameter("velocity_deadband").as_double();
        stop_timeout_ms_       = this->get_parameter("stop_timeout_ms").as_int();

        invert_left_           = this->get_parameter("invert_left").as_bool();
        invert_right_          = this->get_parameter("invert_right").as_bool();
        invert_third_          = this->get_parameter("invert_third").as_bool();

        third_wheel_radius_    = this->get_parameter("third_wheel_radius").as_double();
        third_gear_ratio_      = this->get_parameter("third_gear_ratio").as_double();

        gpr_forward_only_      = this->get_parameter("gpr_forward_only").as_bool();
        gpr_gate_turns_        = this->get_parameter("gpr_gate_turns").as_bool();
        gpr_turn_rate_thresh_  = this->get_parameter("gpr_turn_rate_thresh").as_double();
        gpr_follow_coast_      = this->get_parameter("gpr_follow_coast").as_bool();
        gpr_coast_timeout_ms_  = this->get_parameter("gpr_coast_timeout_ms").as_int();
        gpr_min_vx_follow_     = this->get_parameter("gpr_min_vx_follow").as_double();

        gpr_use_fastlio_odom_  = this->get_parameter("gpr_use_fastlio_odom").as_bool();
        fastlio_odom_topic_    = this->get_parameter("fastlio_odom_topic").as_string();
        fastlio_timeout_ms_    = this->get_parameter("fastlio_timeout_ms").as_int();

        RCLCPP_INFO(get_logger(),
            "Drive: r=%.3f m, base=%.3f m, gear=%.2f | GPR: r=%.3f m, gear=%.2f | invert L/R/3=%d/%d/%d",
            wheel_radius_, wheel_base_, gear_ratio_, third_wheel_radius_, third_gear_ratio_,
            invert_left_, invert_right_, invert_third_);

        // ---------------- Interfaces ----------------
        odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        left_motor_pub_  = this->create_publisher<odrive_can::msg::ControlMessage>("/left/control_message", 10);
        right_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/right/control_message", 10);
        third_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/gpr/control_message", 10);

        tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        left_axis_client_  = this->create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        third_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/gpr/request_axis_state");

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

        left_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/left/controller_status", 10, std::bind(&DiffDriveController::left_status_callback, this, std::placeholders::_1));
        right_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/right/controller_status", 10, std::bind(&DiffDriveController::right_status_callback, this, std::placeholders::_1));
        third_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/gpr/controller_status", 10, std::bind(&DiffDriveController::third_status_callback, this, std::placeholders::_1));

        // Optional: subscribe to Fast-LIO odometry for GPR speed estimation
        if (gpr_use_fastlio_odom_) {
            fastlio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                fastlio_odom_topic_, 10, std::bind(&DiffDriveController::fastlio_odom_callback, this, std::placeholders::_1));
        }

        // Timers
        odom_timer_ = this->create_wall_timer(100ms, std::bind(&DiffDriveController::update_odometry, this)); // 10 Hz
        gpr_timer_  = this->create_wall_timer(50ms,  std::bind(&DiffDriveController::update_gpr_motor, this)); // 20 Hz

        // Arm motors a moment after start
        arm_timer_  = this->create_wall_timer(200ms, [this]() {
            this->arm_motors();
            this->arm_timer_->cancel();
        });

        last_cmd_time_ = this->get_clock()->now();
    }

    // ---------------- Motor State Helpers ----------------
    void arm_motors() {
        RCLCPP_INFO(this->get_logger(), "Waiting for ODrive services to become available...");
        if (!left_axis_client_->wait_for_service(3s) ||
            !right_axis_client_->wait_for_service(3s) ||
            !third_axis_client_->wait_for_service(3s)) {
            RCLCPP_FATAL(this->get_logger(), "ODrive services not available after waiting. Shutting down.");
            g_shutdown_requested = true;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Services found. Arming motors (CLOSED_LOOP)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
        third_axis_client_->async_send_request(request);
    }

    void disarm_motors() {
        RCLCPP_INFO(this->get_logger(), "Disarming motors (IDLE)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
        third_axis_client_->async_send_request(request);
    }

private:
    // ---------------- Command Velocity ----------------
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double linear_vel  = msg->linear.x;   // m/s
        const double angular_vel = msg->angular.z;  // rad/s
        last_cmd_time_           = this->get_clock()->now();
        last_cmd_linear_x_       = linear_vel;

        // Dead-band -> zero torque stop (all motors)
        if (std::abs(linear_vel) < velocity_deadband_ && std::abs(angular_vel) < velocity_deadband_) {
            send_zero_torque();
            return;
        }

        // Skid-steer kinematics – wheel linear speeds (m/s)
        double right_wheel_mps = linear_vel + (angular_vel * wheel_base_ / 2.0);
        double left_wheel_mps  = linear_vel - (angular_vel * wheel_base_ / 2.0);

        // Apply turn speed multiplier to the rotational component only
        double rot_term = (angular_vel * wheel_base_ / 2.0) * (turn_speed_multiplier_ - 1.0);
        right_wheel_mps += rot_term;
        left_wheel_mps  -= rot_term;

        // Convert wheel m/s -> wheel turns/s
        const double wheel_circumference = 2.0 * M_PI * wheel_radius_;
        const double right_wheel_turns = right_wheel_mps / wheel_circumference;
        const double left_wheel_turns  = left_wheel_mps  / wheel_circumference;

        // Wheel turns/s -> motor turns/s via gearbox
        double right_motor_rps_cmd = right_wheel_turns * gear_ratio_ * velocity_multiplier_;
        double left_motor_rps_cmd  = left_wheel_turns  * gear_ratio_ * velocity_multiplier_;

        // Build messages
        odrive_can::msg::ControlMessage right_msg;
        right_msg.control_mode = odrv::VELOCITY_CONTROL;
        right_msg.input_mode   = odrv::VEL_RAMP;
        right_msg.input_torque = 0.0;

        odrive_can::msg::ControlMessage left_msg = right_msg;

        // Apply inversion consistently
        right_msg.input_vel = invert_right_ ? -right_motor_rps_cmd : right_motor_rps_cmd;
        left_msg .input_vel = invert_left_  ? -left_motor_rps_cmd  : left_motor_rps_cmd;

        // Publish to drive motors
        right_motor_pub_->publish(right_msg);
        left_motor_pub_ ->publish(left_msg);

        // NOTE: Do NOT command the third motor here. It's controlled by update_gpr_motor()
    }

    // ---------------- Feedback Callbacks ----------------
    void left_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_left_vel_ = msg->vel_estimate;   // motor turns/s (raw sign from ODrive)
    }
    void right_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_right_vel_ = msg->vel_estimate;  // motor turns/s
    }
    void third_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_third_vel_ = msg->vel_estimate;  // motor turns/s
    }

    // ---------------- Odometry ----------------
    void update_odometry() {
        const rclcpp::Time current_time = this->get_clock()->now();

        // Watchdog: if no cmd_vel recently, coast all motors
        const double ms_since_cmd = (current_time - last_cmd_time_).seconds() * 1000.0;
        if (ms_since_cmd > static_cast<double>(stop_timeout_ms_)) {
            send_zero_torque();
        }

        const double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) {
            return;
        }

        const double wheel_circumference = 2.0 * M_PI * wheel_radius_;

        // Convert **motor** turns/s -> **wheel** m/s, applying the same inversion as in command side
        const double left_motor_rps_meas  = invert_left_  ? -current_left_vel_  : current_left_vel_;
        const double right_motor_rps_meas = invert_right_ ? -current_right_vel_ : current_right_vel_;

        const double left_wheel_mps_meas  = (left_motor_rps_meas  / gear_ratio_) * wheel_circumference;
        const double right_wheel_mps_meas = (right_motor_rps_meas / gear_ratio_) * wheel_circumference;

        const double linear_vel  = (left_wheel_mps_meas + right_wheel_mps_meas) / 2.0;     // m/s
        const double angular_vel = (right_wheel_mps_meas - left_wheel_mps_meas) / wheel_base_; // rad/s

        // Integrate pose
        x_     += linear_vel * std::cos(theta_) * dt;
        y_     += linear_vel * std::sin(theta_) * dt;
        theta_ += angular_vel * dt;
        theta_  = std::atan2(std::sin(theta_), std::cos(theta_)); // wrap

        // Publish odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q; q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x  = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;

        odom_pub_->publish(odom_msg);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom_msg.header;
        tf_msg.child_frame_id              = "base_link";
        tf_msg.transform.translation.x     = x_;
        tf_msg.transform.translation.y     = y_;
        tf_msg.transform.translation.z     = 0.0;
        tf_msg.transform.rotation          = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        // Save measured body speeds for the GPR gate
        meas_vx_ = linear_vel;
        meas_wz_ = angular_vel;

        last_time_ = current_time;
    }

    // ---------------- GPR motor updater (forward-only gating) ----------------
    void update_gpr_motor() {
        // Watchdog for GPR control: if stale, stop third motor
        const double ms_since_cmd = (this->get_clock()->now() - last_cmd_time_).seconds() * 1000.0;
        // If command is stale, optionally follow measured coast for a short window
        if (ms_since_cmd > static_cast<double>(stop_timeout_ms_)) {
            if (!(gpr_follow_coast_ &&
                  ms_since_cmd <= static_cast<double>(gpr_coast_timeout_ms_) &&
                  std::abs(meas_vx_) > gpr_min_vx_follow_)) {
                send_zero_torque_third();
                return;
            }
            // else fall through to compute drive from measured velocities
        }

        const double wz = meas_wz_;  // yaw rate rad/s (from wheel-based odom)

        // Forward-only gating
        // if (gpr_forward_only_) {
        //     if (vx <= velocity_deadband_) {
        //         send_zero_torque_third();
        //         return;
        //     }
        // } else {
        //     if (std::abs(vx) <= velocity_deadband_) {
        //         send_zero_torque_third();
        //         return;
        //     }
        // }

        // Optional: gate out during turns
        if (gpr_gate_turns_ && std::abs(wz) > gpr_turn_rate_thresh_) {
            send_zero_torque_third();
            return;
        }

        // Compute robot linear speed to drive GPR. Prefer Fast-LIO odometry if enabled and fresh.
        double vx = 0.0;
        // Use ROS clock for Fast-LIO stamps to avoid time source mismatch
        const rclcpp::Time now_ros = ros_clock_.now();
        const double ms_since_fastlio = (now_ros - last_fastlio_time_).seconds() * 1000.0;
        bool have_fastlio = gpr_use_fastlio_odom_ && fastlio_speed_mps_ > 0.0 && ms_since_fastlio <= static_cast<double>(fastlio_timeout_ms_);
        if (have_fastlio) {
            vx = fastlio_speed_mps_;
        } else {
            // Fallback: estimate from drive motor speeds
            const double wheel_circumference = 2.0 * M_PI * wheel_radius_;
            const double left_motor_rps_meas  = invert_left_  ? -current_left_vel_  : current_left_vel_;
            const double right_motor_rps_meas = invert_right_ ? -current_right_vel_ : current_right_vel_;
            const double left_wheel_mps_meas  = (left_motor_rps_meas  / gear_ratio_) * wheel_circumference;
            const double right_wheel_mps_meas = (right_motor_rps_meas / gear_ratio_) * wheel_circumference;
            vx = (left_wheel_mps_meas + right_wheel_mps_meas) / 2.0;
        }

        // Map measured forward speed to GPR motor turns/s via kinematics
        const double third_circ = 2.0 * M_PI * third_wheel_radius_;
        double turns_per_sec = (vx / third_circ) * third_gear_ratio_ * velocity_multiplier_;

        // If forward-only, do not allow negative speed
        if (gpr_forward_only_ && turns_per_sec < 0.0) {
            turns_per_sec = 0.0;
        }

        odrive_can::msg::ControlMessage msg;
        msg.control_mode = odrv::VELOCITY_CONTROL;
        msg.input_mode   = odrv::VEL_RAMP;
        msg.input_torque = 0.0;
        msg.input_vel    = invert_third_ ? -turns_per_sec : turns_per_sec;

        third_motor_pub_->publish(msg);
    }

    // ---------------- Fast-LIO odometry callback ----------------
    void fastlio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const double px = msg->pose.pose.position.x;
        const double py = msg->pose.pose.position.y;
        const double pz = msg->pose.pose.position.z;
        // Force ROS time for incoming stamp to avoid mixing time sources
        const rclcpp::Time t(msg->header.stamp, RCL_ROS_TIME);
        if (!have_fastlio_prev_) {
            last_fastlio_x_ = px;
            last_fastlio_y_ = py;
            last_fastlio_z_ = pz;
            last_fastlio_time_ = t;
            have_fastlio_prev_ = true;
            fastlio_speed_mps_ = 0.0;
            return;
        }
        const double dt = (t - last_fastlio_time_).seconds();
        if (dt <= 0.0) {
            return;
        }
        const double dx = px - last_fastlio_x_;
        const double dy = py - last_fastlio_y_;
        const double dz = pz - last_fastlio_z_;
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        fastlio_speed_mps_ = dist / dt; // magnitude speed between updates
        last_fastlio_x_ = px;
        last_fastlio_y_ = py;
        last_fastlio_z_ = pz;
        last_fastlio_time_ = t;
    }

    // ---------------- Helpers ----------------
    void send_zero_torque(){
        odrive_can::msg::ControlMessage msg;
        msg.control_mode = odrv::TORQUE_CONTROL;
        msg.input_mode   = odrv::PASSTHROUGH;
        msg.input_vel    = 0.0; // ignored in torque mode
        msg.input_torque = 0.0;
        left_motor_pub_->publish(msg);
        right_motor_pub_->publish(msg);
        third_motor_pub_->publish(msg);
    }

    void send_zero_torque_third(){
        odrive_can::msg::ControlMessage msg;
        msg.control_mode = odrv::VELOCITY_CONTROL;
        msg.input_mode   = odrv::PASSTHROUGH;
        msg.input_vel    = 0.0;
        msg.input_torque = 0.0;
        third_motor_pub_->publish(msg);
    }

    // ---------------- Members ----------------
    // Params
    double wheel_radius_{0.057}, wheel_base_{0.35}, gear_ratio_{10.0};
    double velocity_multiplier_{1.0}, turn_speed_multiplier_{1.0}, velocity_deadband_{0.01};
    int    stop_timeout_ms_{500};

    bool   invert_left_{true}, invert_right_{false}, invert_third_{false};
    double third_wheel_radius_{0.03}, third_gear_ratio_{1.0};

    bool   gpr_forward_only_{true}, gpr_gate_turns_{false};
    double gpr_turn_rate_thresh_{0.25};
    bool   gpr_follow_coast_{true};
    int    gpr_coast_timeout_ms_{2000};
    double gpr_min_vx_follow_{0.02};

    // ROS I/O
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr left_motor_pub_, right_motor_pub_, third_motor_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_, right_axis_client_, third_axis_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_status_sub_, right_status_sub_, third_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_, arm_timer_, gpr_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // State
    rclcpp::Time last_time_, last_cmd_time_;
    double current_left_vel_  = 0.0; // motor turns/s (raw from ODrive)
    double current_right_vel_ = 0.0;
    double current_third_vel_ = 0.0;

    double meas_vx_ = 0.0; // measured linear x (m/s)
    double meas_wz_ = 0.0; // measured yaw (rad/s)

    double last_cmd_linear_x_ = 0.0; // last commanded forward m/s

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    // Fast-LIO state for GPR drive
    bool   gpr_use_fastlio_odom_{true};
    std::string fastlio_odom_topic_{"/Odometry"};
    int    fastlio_timeout_ms_{500};
    bool   have_fastlio_prev_{false};
    double last_fastlio_x_{0.0}, last_fastlio_y_{0.0}, last_fastlio_z_{0.0};
    rclcpp::Time last_fastlio_time_{};
    double fastlio_speed_mps_{0.0};
    // ROS time clock for Fast-LIO age computations
    rclcpp::Clock ros_clock_{RCL_ROS_TIME};
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
