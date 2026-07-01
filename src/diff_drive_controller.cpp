#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <deque>
#include <array>
#include <Eigen/Dense>

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
        this->declare_parameter<double>("gear_ratio",   1.0);

        // Drive behavior
        this->declare_parameter<double>("velocity_multiplier",    1.0);
        this->declare_parameter<double>("turn_speed_multiplier",  1.0);
        this->declare_parameter<double>("velocity_deadband",      0.001); // m/s (and rad/s used comparably)
        this->declare_parameter<int>("stop_timeout_ms",           500);

        // Motor inversion (signs)
        this->declare_parameter<bool>("invert_left",   false);   // adjust to your wiring
        this->declare_parameter<bool>("invert_right",  true);
        // this->declare_parameter<bool>("invert_third",  false);  // GPR: DISABLED

        // GPR (node 2) kinematics (60 mm dia virtual wheel by default, direct 1:1)
        // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
        // this->declare_parameter<double>("third_wheel_radius", 0.03); // 60 mm dia -> 0.03 m radius
        // this->declare_parameter<double>("third_gear_ratio",   1.0);  // direct

        // GPR gating
        // this->declare_parameter<bool>("gpr_forward_only", false);     // rotate only when moving forward
        // this->declare_parameter<bool>("gpr_gate_turns",   false);    // optionally stop during turns
        // this->declare_parameter<double>("gpr_turn_rate_thresh", 0.25); // rad/s
        // Allow GPR to continue during residual coast after cmd_vel stops
        // this->declare_parameter<bool>("gpr_follow_coast", false);
        // this->declare_parameter<int>("gpr_coast_timeout_ms", 1000);
        // this->declare_parameter<double>("gpr_min_vx_follow", 0.02);

        // Fast-LIO odometry input (for GPR velocity drive)
        this->declare_parameter<bool>("gpr_use_fastlio_odom", true);
        this->declare_parameter<std::string>("fastlio_odom_topic", "/Odometry");
        this->declare_parameter<int>("fastlio_timeout_ms", 500);
        this->declare_parameter<int>("fastlio_delta_window", 10);
        
        // Tilt correction parameters (matching pose_controller.py)
        this->declare_parameter<std::string>("accel_topic", "/livox/imu");
        this->declare_parameter<int>("accel_samples", 10);
        this->declare_parameter<std::string>("corrected_odom_topic", "/Odometry_tilt_corrected_diff_cpp");

        // Leveling-gated arming: wait for odom_tilt_corrector to latch a levelling
        // rotation (/leveling_ready) before requesting CLOSED_LOOP, so motors stay
        // idle (robot stationary) while the gravity window is captured. Falls back
        // to arming anyway after the timeout so a bad IMU can never brick arming.
        this->declare_parameter<bool>("arm_wait_for_leveling", true);
        this->declare_parameter<int>("arm_leveling_timeout_ms", 10000);

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
        // invert_third_          = this->get_parameter("invert_third").as_bool();  // GPR: DISABLED

        // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
        // third_wheel_radius_    = this->get_parameter("third_wheel_radius").as_double();
        // third_gear_ratio_      = this->get_parameter("third_gear_ratio").as_double();

        // gpr_forward_only_      = this->get_parameter("gpr_forward_only").as_bool();
        // gpr_gate_turns_        = this->get_parameter("gpr_gate_turns").as_bool();
        // gpr_turn_rate_thresh_  = this->get_parameter("gpr_turn_rate_thresh").as_double();
        // gpr_follow_coast_      = this->get_parameter("gpr_follow_coast").as_bool();
        // gpr_coast_timeout_ms_  = this->get_parameter("gpr_coast_timeout_ms").as_int();
        // gpr_min_vx_follow_     = this->get_parameter("gpr_min_vx_follow").as_double();

        gpr_use_fastlio_odom_  = this->get_parameter("gpr_use_fastlio_odom").as_bool();
        fastlio_odom_topic_    = this->get_parameter("fastlio_odom_topic").as_string();
        fastlio_timeout_ms_    = this->get_parameter("fastlio_timeout_ms").as_int();
        fastlio_delta_window_  = this->get_parameter("fastlio_delta_window").as_int();
        if (fastlio_delta_window_ < 1) fastlio_delta_window_ = 1;
        
        accel_topic_           = this->get_parameter("accel_topic").as_string();
        accel_samples_target_  = this->get_parameter("accel_samples").as_int();
        corrected_odom_topic_  = this->get_parameter("corrected_odom_topic").as_string();

        arm_wait_for_leveling_   = this->get_parameter("arm_wait_for_leveling").as_bool();
        arm_leveling_timeout_ms_ = this->get_parameter("arm_leveling_timeout_ms").as_int();

        RCLCPP_INFO(get_logger(),
            "Drive: r=%.3f m, base=%.3f m, gear=%.2f | invert L/R=%d/%d",
            wheel_radius_, wheel_base_, gear_ratio_,
            invert_left_, invert_right_);

        // ---------------- Interfaces ----------------
        odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        left_motor_pub_  = this->create_publisher<odrive_can::msg::ControlMessage>("/left/control_message", 10);
        right_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/right/control_message", 10);
        
        // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
        // third_motor_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>("/gpr/control_message", 10);
        // gpr_vx_pub_      = this->create_publisher<std_msgs::msg::Float32>("/gpr/velocity_mps", 10);
        // gpr_vx_raw_pub_  = this->create_publisher<std_msgs::msg::Float32>("/gpr/velocity_mps_raw", 10);
        // gpr_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gpr/travel_distance_m", 10);
        // gpr_fastlio_delta_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gpr/fastlio_delta_xyz", 10);
        
        // Tilt-corrected odometry republishing moved to odom_tilt_corrector.py
        // odom_corrected_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(corrected_odom_topic_, 10);

        tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        left_axis_client_  = this->create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        // third_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/gpr/request_axis_state");

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

        left_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/left/controller_status", 10, std::bind(&DiffDriveController::left_status_callback, this, std::placeholders::_1));
        right_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/right/controller_status", 10, std::bind(&DiffDriveController::right_status_callback, this, std::placeholders::_1));
        
        // MPC autonomy enable subscription - when active, ignore cmd_vel to let MPC control motors
        mpc_autonomy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mpc_autonomy_enable", 10, std::bind(&DiffDriveController::mpc_autonomy_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /mpc_autonomy_enable for MPC handoff");

        // Leveling-ready gate (latched by odom_tilt_corrector). TRANSIENT_LOCAL so
        // we still catch it if it was published before this subscription came up.
        {
            auto leveling_qos = rclcpp::QoS(1).transient_local().reliable();
            leveling_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/leveling_ready", leveling_qos,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (msg->data && !leveling_ready_) {
                        leveling_ready_ = true;
                        RCLCPP_INFO(this->get_logger(), "Leveling ready received; motor arming unblocked.");
                    }
                });
        }
        
        // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
        // third_status_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
        //     "/gpr/controller_status", 10, std::bind(&DiffDriveController::third_status_callback, this, std::placeholders::_1));

        // Optional: subscribe to Fast-LIO odometry for GPR speed estimation
        // Disable Fast-LIO odom subscription for tilt-corrected republishing (handled by Python node)
        // if (gpr_use_fastlio_odom_) {
        //     fastlio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //         fastlio_odom_topic_, 10, std::bind(&DiffDriveController::fastlio_odom_callback, this, std::placeholders::_1));
        // }
        
        // IMU subscription for gravity-based tilt correction
        // Disable IMU subscription for tilt-corrected republishing (handled by Python node)
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     accel_topic_, 10, std::bind(&DiffDriveController::imu_callback, this, std::placeholders::_1));

        // Timers
        odom_timer_ = this->create_wall_timer(100ms, std::bind(&DiffDriveController::update_odometry, this)); // 10 Hz
        
        // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
        // gpr_timer_  = this->create_wall_timer(50ms,  std::bind(&DiffDriveController::update_gpr_motor, this)); // 20 Hz

        // Arm motors once leveling is ready (or after a safety timeout). Polls at
        // 200 ms; only requests CLOSED_LOOP after the gate clears so the robot is
        // guaranteed stationary while odom_tilt_corrector captures gravity.
        arm_wait_start_ = this->get_clock()->now();
        arm_timer_  = this->create_wall_timer(200ms, [this]() {
            if (arm_dispatched_) { return; }
            if (!arm_wait_for_leveling_ || leveling_ready_) {
                this->arm_motors();
                arm_dispatched_ = true;
                this->arm_timer_->cancel();
                return;
            }
            const double waited_ms =
                (this->get_clock()->now() - arm_wait_start_).seconds() * 1000.0;
            if (waited_ms >= static_cast<double>(arm_leveling_timeout_ms_)) {
                RCLCPP_WARN(this->get_logger(),
                    "Leveling-ready not received after %d ms; arming anyway "
                    "(fallback tilt calibration in effect).",
                    arm_leveling_timeout_ms_);
                this->arm_motors();
                arm_dispatched_ = true;
                this->arm_timer_->cancel();
            }
        });

        last_cmd_time_ = this->get_clock()->now();
    }

    // ---------------- Motor State Helpers ----------------
    void arm_motors() {
        RCLCPP_INFO(this->get_logger(), "Waiting for ODrive services to become available...");
        // if (!left_axis_client_->wait_for_service(3s) ||
        //     !right_axis_client_->wait_for_service(3s) ||
        //     !third_axis_client_->wait_for_service(3s)) { // UNDO-GPR
        if (!left_axis_client_->wait_for_service(3s) ||
            !right_axis_client_->wait_for_service(3s)) {
            RCLCPP_FATAL(this->get_logger(), "ODrive services not available after waiting. Shutting down.");
            g_shutdown_requested = true;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Services found. Arming motors (CLOSED_LOOP)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
        // third_axis_client_->async_send_request(request); // UNDO-GPR
    }

    void disarm_motors() {
        RCLCPP_INFO(this->get_logger(), "Disarming motors (IDLE)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
        // third_axis_client_->async_send_request(request); // UNDO-GPR
    }

private:
    // -------------------- Quaternion/Matrix Helper Methods (matching pose_controller.py) --------------------
    static std::array<double, 4> quat_normalize(const std::array<double, 4>& q) {
        double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm < 1e-12) {
            return {0.0, 0.0, 0.0, 1.0};
        }
        return {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
    }

    static std::array<double, 4> quat_from_axis_angle(const std::array<double, 3>& axis, double angle) {
        double half = angle * 0.5;
        double s = std::sin(half);
        return {axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half)};
    }

    static std::array<double, 4> quat_multiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
        return {
            q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1],
            q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0],
            q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3],
            q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
        };
    }

    static std::array<double, 4> quat_conjugate(const std::array<double, 4>& q) {
        return {-q[0], -q[1], -q[2], q[3]};
    }

    static std::array<double, 3> rotate_vector_by_quat(const std::array<double, 4>& q, const std::array<double, 3>& v) {
        std::array<double, 4> v_quat = {v[0], v[1], v[2], 0.0};
        auto temp = quat_multiply(q, v_quat);
        auto result = quat_multiply(temp, quat_conjugate(q));
        return {result[0], result[1], result[2]};
    }

    static std::array<double, 4> compute_alignment_quat_between(const std::array<double, 3>& from_vec, 
                                                                 const std::array<double, 3>& to_vec) {
        // Normalize input vectors
        double fx = from_vec[0], fy = from_vec[1], fz = from_vec[2];
        double norm_from = std::sqrt(fx*fx + fy*fy + fz*fz);
        if (norm_from < 1e-6) {
            return {0.0, 0.0, 0.0, 1.0};
        }
        fx /= norm_from; fy /= norm_from; fz /= norm_from;

        double tx = to_vec[0], ty = to_vec[1], tz = to_vec[2];
        double norm_to = std::sqrt(tx*tx + ty*ty + tz*tz);
        if (norm_to < 1e-6) {
            return {0.0, 0.0, 0.0, 1.0};
        }
        tx /= norm_to; ty /= norm_to; tz /= norm_to;

        // Compute rotation axis (cross product)
        std::array<double, 3> axis = {
            fy*tz - fz*ty,
            fz*tx - fx*tz,
            fx*ty - fy*tx
        };
        double axis_norm = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        double dot = fx*tx + fy*ty + fz*tz;

        // Handle special cases
        if (axis_norm < 1e-6) {
            if (dot > 0) {
                // Vectors are parallel
                return {0.0, 0.0, 0.0, 1.0};
            } else {
                // Vectors are opposite - 180 degree rotation
                // Find a perpendicular axis
                std::array<double, 3> perp_axis = {1.0, 0.0, 0.0};
                if (std::abs(fx) > 0.9) {
                    perp_axis = {0.0, 1.0, 0.0};
                }
                return quat_from_axis_angle(perp_axis, M_PI);
            }
        }

        axis[0] /= axis_norm; axis[1] /= axis_norm; axis[2] /= axis_norm;
        double angle = std::atan2(axis_norm, dot);
        return quat_from_axis_angle(axis, angle);
    }

    static std::array<double, 3> quat_to_rpy(const std::array<double, 4>& q) {
        // Convert quaternion to roll, pitch, yaw (radians)
        double x = q[0], y = q[1], z = q[2], w = q[3];
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1.0) {
            pitch = std::copysign(M_PI / 2.0, sinp);  // Use 90 degrees if out of range
        } else {
            pitch = std::asin(sinp);
        }
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        return {roll, pitch, yaw};
    }

    static Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw) {
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        double cr = std::cos(roll),  sr = std::sin(roll);
        double cp = std::cos(pitch), sp = std::sin(pitch);
        double cy = std::cos(yaw),   sy = std::sin(yaw);
        
        Eigen::Matrix3d Rz, Ry, Rx;
        Rz << cy, -sy, 0.0,
              sy,  cy, 0.0,
             0.0, 0.0, 1.0;
        
        Ry << cp, 0.0,  sp,
             0.0, 1.0, 0.0,
             -sp, 0.0,  cp;
        
        Rx << 1.0, 0.0, 0.0,
             0.0,  cr, -sr,
             0.0,  sr,  cr;
        
        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d quat_to_matrix(const std::array<double, 4>& q) {
        double x = q[0], y = q[1], z = q[2], w = q[3];
        Eigen::Matrix3d R;
        R(0,0) = 1 - 2*(y*y + z*z);
        R(0,1) = 2*(x*y - w*z);
        R(0,2) = 2*(x*z + w*y);
        R(1,0) = 2*(x*y + w*z);
        R(1,1) = 1 - 2*(x*x + z*z);
        R(1,2) = 2*(y*z - w*x);
        R(2,0) = 2*(x*z - w*y);
        R(2,1) = 2*(y*z + w*x);
        R(2,2) = 1 - 2*(x*x + y*y);
        return R;
    }

    static std::array<double, 4> matrix_to_quat(const Eigen::Matrix3d& R) {
        double trace = R.trace();
        std::array<double, 4> q;
        if (trace > 0.0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            q[3] = 0.25 / s;
            q[0] = (R(2,1) - R(1,2)) * s;
            q[1] = (R(0,2) - R(2,0)) * s;
            q[2] = (R(1,0) - R(0,1)) * s;
        } else {
            if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
                double s = 2.0 * std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
                q[3] = (R(2,1) - R(1,2)) / s;
                q[0] = 0.25 * s;
                q[1] = (R(0,1) + R(1,0)) / s;
                q[2] = (R(0,2) + R(2,0)) / s;
            } else if (R(1,1) > R(2,2)) {
                double s = 2.0 * std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
                q[3] = (R(0,2) - R(2,0)) / s;
                q[0] = (R(0,1) + R(1,0)) / s;
                q[1] = 0.25 * s;
                q[2] = (R(1,2) + R(2,1)) / s;
            } else {
                double s = 2.0 * std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
                q[3] = (R(1,0) - R(0,1)) / s;
                q[0] = (R(0,2) + R(2,0)) / s;
                q[1] = (R(1,2) + R(2,1)) / s;
                q[2] = 0.25 * s;
            }
        }
        return quat_normalize(q);
    }


    // ---------------- MPC Autonomy Callback ----------------
    void mpc_autonomy_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        bool new_state = msg->data;
        if (new_state != mpc_autonomy_active_) {
            mpc_autonomy_active_ = new_state;
            if (mpc_autonomy_active_) {
                RCLCPP_INFO(this->get_logger(), "🤖 MPC AUTONOMY ENABLED - diff_drive_controller yielding motor control");
                // Don't send any command - just yield control to MPC immediately
            } else {
                RCLCPP_INFO(this->get_logger(), "🎮 MANUAL TELEOP ENABLED - diff_drive_controller resuming motor control");
            }
        }
    }

    // ---------------- Command Velocity ----------------
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // SAFETY: When MPC autonomy is active, ignore all cmd_vel commands
        // This prevents fighting for motor control between teleop and MPC
        if (mpc_autonomy_active_) {
            // Don't log every ignored message to avoid spam, but update time
            last_cmd_time_ = this->get_clock()->now();
            return;
        }
        
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
        double rot_term = (angular_vel * wheel_base_ / 2.0);
        right_wheel_mps += rot_term;
        left_wheel_mps  -= rot_term;

        // Convert wheel m/s -> wheel turns/s
        const double wheel_circumference = 2.0 * M_PI * wheel_radius_;
        const double right_wheel_turns = right_wheel_mps / wheel_circumference;
        const double left_wheel_turns  = left_wheel_mps  / wheel_circumference;

        // Wheel turns/s -> motor turns/s via gearbox
        double right_motor_rps_cmd = right_wheel_turns;
        double left_motor_rps_cmd  = left_wheel_turns;

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

        // NOTE: GPR motor control moved to gpr_scan_controller.py
    }

    // ---------------- Feedback Callbacks ----------------
    void left_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_left_vel_ = msg->vel_estimate;   // motor turns/s (raw sign from ODrive)
    }
    void right_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
        current_right_vel_ = msg->vel_estimate;  // motor turns/s
    }
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // void third_status_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
    //     current_third_vel_ = msg->vel_estimate;  // motor turns/s
    // }

    // ---------------- Odometry ----------------
    void update_odometry() {
        const rclcpp::Time current_time = this->get_clock()->now();

        // Watchdog: if no cmd_vel recently, coast all motors
        const double ms_since_cmd = (current_time - last_cmd_time_).seconds() * 1000.0;
        if (mpc_autonomy_active_) {
            // When MPC is active we intentionally stay silent - do not send zero torque packets.
            // Keep last_cmd_time_ fresh so that teleop resumes cleanly once MPC is disabled.
            last_cmd_time_ = current_time;
        } else if (ms_since_cmd > static_cast<double>(stop_timeout_ms_)) {
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

    // ---------------- GPR motor updater (absolute position from travel distance) ----------------
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // void update_gpr_motor() {
    //     // Watchdog for GPR control: if stale, stop third motor
    //     const double ms_since_cmd = (this->get_clock()->now() - last_cmd_time_).seconds() * 1000.0;
    //     // If command is stale, optionally follow measured coast for a short window
    //     if (ms_since_cmd > static_cast<double>(stop_timeout_ms_)) {
    //         if (!(gpr_follow_coast_ &&
    //               ms_since_cmd <= static_cast<double>(gpr_coast_timeout_ms_) &&
    //               std::abs(meas_vx_) > gpr_min_vx_follow_)) {
    //             send_zero_torque_third();
    //             return;
    //         }
    //         // else fall through to compute drive from measured velocities
    //     }

    //     const double wz = meas_wz_;  // yaw rate rad/s (from wheel-based odom)

    //     // Forward-only gating
    //     // if (gpr_forward_only_) {
    //     //     if (vx <= velocity_deadband_) {
    //     //         send_zero_torque_third();
    //     //         return;
    //     //     }
    //     // } else {
    //     //     if (std::abs(vx) <= velocity_deadband_) {
    //     //         send_zero_torque_third();
    //     //         return;
    //     //     }
    //     // }

    //     // Optional: gate out during turns
    //     if (gpr_gate_turns_ && std::abs(wz) > gpr_turn_rate_thresh_) {
    //         send_zero_torque_third();
    //         return;
    //     }

    //     // Compute desired absolute GPR motor position (turns) from total travel distance
    //     // turns = (distance_m / circumference_m) * gear_ratio * velocity_multiplier
    //     const double third_circ = 2.0 * M_PI * third_wheel_radius_;
    //     double turns_target = (total_travel_m_ / third_circ) * third_gear_ratio_ * velocity_multiplier_;

    //     // Apply inversion/sign convention consistent with previous velocity command path
    //     double pos_cmd_turns = invert_third_ ? -turns_target : turns_target;

    //     // Send absolute position command using trap trajectory for smooth motion
    //     odrive_can::msg::ControlMessage msg;
    //     msg.control_mode = odrv::POSITION_CONTROL;
    //     msg.input_mode   = odrv::TRAP_TRAJ;
    //     msg.input_torque = 0.0;
    //     msg.input_vel    = 0.0;
    //     msg.input_pos    = pos_cmd_turns;

    //     //third_motor_pub_->publish(msg);
    // }

    // ---------------- IMU callback for gravity-based tilt correction ----------------
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (accel_initialized_) {
            return;  // Already have enough samples
        }

        // Collect linear acceleration samples
        accel_samples_.push_back(msg->linear_acceleration.x);
        accel_samples_.push_back(msg->linear_acceleration.y);
        accel_samples_.push_back(msg->linear_acceleration.z);

        // Check if we have enough samples (3 values per sample)
        if (static_cast<int>(accel_samples_.size()) >= accel_samples_target_ * 3) {
            // Compute average
            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            int num_samples = accel_samples_target_;
            for (int i = 0; i < num_samples; i++) {
                sum_x += accel_samples_[i * 3 + 0];
                sum_y += accel_samples_[i * 3 + 1];
                sum_z += accel_samples_[i * 3 + 2];
            }
            accel_avg_[0] = sum_x / num_samples;
            accel_avg_[1] = sum_y / num_samples;
            accel_avg_[2] = sum_z / num_samples;

            accel_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU acceleration averaged: accel_avg=[%.3f, %.3f, %.3f] (body frame)",
                        accel_avg_[0], accel_avg_[1], accel_avg_[2]);
            RCLCPP_INFO(this->get_logger(), "  Waiting for first odometry to compute alignment using body orientation...");
        }
    }

    // ---------------- Fast-LIO odometry callback (with tilt correction) ----------------
    void fastlio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Wait for IMU-based tilt correction to be initialized
        if (!accel_initialized_) {
            return;
        }

        // Extract raw pose
        double raw_px = msg->pose.pose.position.x;
        double raw_py = msg->pose.pose.position.y;
        double raw_pz = msg->pose.pose.position.z;
        
        std::array<double, 4> raw_quat = {
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        };

        // On first odometry, establish the map frame transformation
        if (!odom_initialized_) {
            // Store world origin
            p0_world_[0] = raw_px;
            p0_world_[1] = raw_py;
            p0_world_[2] = raw_pz;

            // Rotate acceleration from body frame to world frame using current body orientation
            Eigen::Matrix3d R_wb = quat_to_matrix(raw_quat);
            Eigen::Vector3d accel_body(accel_avg_[0], accel_avg_[1], accel_avg_[2]);
            accel_body.normalize();
            Eigen::Vector3d accel_world = R_wb * accel_body;

            // Compute alignment quaternion to align world-frame acceleration to [0, 0, -1]
            std::array<double, 3> a_world = {accel_world[0], accel_world[1], accel_world[2]};
            std::array<double, 3> z_down = {0.0, 0.0, -1.0};
            align_quat_ = compute_alignment_quat_between(a_world, z_down);

            // Compute R_flip with correct axis flips: X flipped, Y unchanged, Z flipped
            Eigen::Matrix3d R_flip;
            R_flip << -1,  0,  0,
                       0,  1,  0,
                       0,  0, -1;

            // Compute R_align from alignment quaternion via RPY (matches pose_controller.py)
            std::array<double, 3> rpy = quat_to_rpy(align_quat_);
            double rx = rpy[0], ry = rpy[1], rz = rpy[2];
            Eigen::Matrix3d R_align = rpy_to_matrix(rx, ry, rz);

            // Compute combined map rotation: R_map = R_flip @ R_align
            R_map_ = R_flip * R_align;

            odom_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Fast-LIO odometry tilt correction initialized at origin [%.3f, %.3f, %.3f]",
                        p0_world_[0], p0_world_[1], p0_world_[2]);
            RCLCPP_INFO(this->get_logger(), "  accel_world=[%.3f, %.3f, %.3f]",
                        accel_world[0], accel_world[1], accel_world[2]);
            RCLCPP_INFO(this->get_logger(), "  align_quat=[%.4f, %.4f, %.4f, %.4f]",
                        align_quat_[0], align_quat_[1], align_quat_[2], align_quat_[3]);
            RCLCPP_INFO(this->get_logger(), "  align_rpy(deg)=[%.2f, %.2f, %.2f]",
                        rx * 180.0 / M_PI, ry * 180.0 / M_PI, rz * 180.0 / M_PI);
        }

        // Transform position: p_local = R_map @ (p_raw - p0_world)
        Eigen::Vector3d p_raw(raw_px - p0_world_[0], raw_py - p0_world_[1], raw_pz - p0_world_[2]);
        Eigen::Vector3d p_local = R_map_ * p_raw;

        // Transform orientation: R_local = R_map @ R_raw
        Eigen::Matrix3d R_raw = quat_to_matrix(raw_quat);
        Eigen::Matrix3d R_local = R_map_ * R_raw;
        std::array<double, 4> local_quat = matrix_to_quat(R_local);

        // Transform linear velocity: vel_local = R_map @ vel_raw
        Eigen::Vector3d vel_raw(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        Eigen::Vector3d vel_local = R_map_ * vel_raw;

        // Angular velocity (copy full vector as-is, planar control uses z)

        // Publish corrected odometry (mirror pose_controller.py)
        nav_msgs::msg::Odometry odom_corrected;
        odom_corrected.header = msg->header;
        odom_corrected.child_frame_id = msg->child_frame_id;

        odom_corrected.pose.pose.position.x = p_local[0];
        odom_corrected.pose.pose.position.y = p_local[1];
        odom_corrected.pose.pose.position.z = p_local[2];

        odom_corrected.pose.pose.orientation.x = local_quat[0];
        odom_corrected.pose.pose.orientation.y = local_quat[1];
        odom_corrected.pose.pose.orientation.z = local_quat[2];
        odom_corrected.pose.pose.orientation.w = local_quat[3];

        odom_corrected.twist.twist.linear.x = vel_local[0];
        odom_corrected.twist.twist.linear.y = vel_local[1];
        odom_corrected.twist.twist.linear.z = 0.0;

        // Keep angular twist as-is (planar control uses z)
        odom_corrected.twist.twist.angular = msg->twist.twist.angular;

        // Do not set covariances to match pose_controller.py defaults

        odom_corrected_pub_->publish(odom_corrected);
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
        // third_motor_pub_->publish(msg); // UNDO-GPR
    }

    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // void send_zero_torque_third(){
    //     odrive_can::msg::ControlMessage msg;
    //     msg.control_mode = odrv::VELOCITY_CONTROL;
    //     msg.input_mode   = odrv::PASSTHROUGH;
    //     msg.input_vel    = 0.0;
    //     msg.input_torque = 0.0;
    //     // third_motor_pub_->publish(msg); // UNDO-GPR
    // }

    // ---------------- Members ----------------
    // Params
    double wheel_radius_{0.057}, wheel_base_{0.35}, gear_ratio_{10.0};
    double velocity_multiplier_{1.0}, turn_speed_multiplier_{1.0}, velocity_deadband_{0.01};
    int    stop_timeout_ms_{500};

    bool   invert_left_{true}, invert_right_{false};
    
    // MPC autonomy state - when true, ignore cmd_vel and let MPC control motors
    bool   mpc_autonomy_active_{false};
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // bool   invert_third_{false};
    // double third_wheel_radius_{0.03}, third_gear_ratio_{1.0};
    // bool   gpr_forward_only_{true}, gpr_gate_turns_{false};
    // double gpr_turn_rate_thresh_{0.25};
    // bool   gpr_follow_coast_{true};
    // int    gpr_coast_timeout_ms_{2000};
    // double gpr_min_vx_follow_{0.02};

    // ROS I/O
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_corrected_pub_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr left_motor_pub_, right_motor_pub_;
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr third_motor_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gpr_vx_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gpr_vx_raw_pub_;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gpr_fastlio_delta_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gpr_distance_pub_;
    
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_, right_axis_client_;
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr third_axis_client_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr left_status_sub_, right_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mpc_autonomy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leveling_ready_sub_;
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr third_status_sub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_, arm_timer_;
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // rclcpp::TimerBase::SharedPtr gpr_timer_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // State
    rclcpp::Time last_time_, last_cmd_time_;
    double current_left_vel_  = 0.0; // motor turns/s (raw from ODrive)
    double current_right_vel_ = 0.0;
    
    // GPR CONTROL DISABLED - Moved to gpr_scan_controller.py
    // double current_third_vel_ = 0.0;

    double meas_vx_ = 0.0; // measured linear x (m/s)
    double meas_wz_ = 0.0; // measured yaw (rad/s)

    double last_cmd_linear_x_ = 0.0; // last commanded forward m/s

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    // Fast-LIO state for GPR drive (removed unused members)
    bool   gpr_use_fastlio_odom_{true};
    std::string fastlio_odom_topic_{"/Odometry"};
    int    fastlio_timeout_ms_{500};
    int    fastlio_delta_window_{5};

    // Tilt correction state (matching pose_controller.py)
    std::string accel_topic_{"/livox/imu"};
    std::string corrected_odom_topic_{"/Odometry_tilt_corrected_diff"};
    int    accel_samples_target_{10};
    bool   accel_initialized_{false};
    bool   odom_initialized_{false};

    // Leveling-gated arming state
    bool        arm_wait_for_leveling_{true};
    int         arm_leveling_timeout_ms_{10000};
    bool        leveling_ready_{false};
    bool        arm_dispatched_{false};
    rclcpp::Time arm_wait_start_;
    std::vector<double> accel_samples_;
    std::array<double, 3> accel_avg_{0.0, 0.0, 0.0};
    std::array<double, 4> align_quat_{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> p0_world_{0.0, 0.0, 0.0};
    Eigen::Matrix3d R_map_{Eigen::Matrix3d::Identity()};
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
