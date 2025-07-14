#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("host_teleop") {
        // Declare parameters to prevent Foxglove bridge errors
        this->declare_parameter("teleop_active", true);
        this->declare_parameter("teleop_mode", "keyboard");
        this->declare_parameter("max_linear_velocity", 1.0);
        this->declare_parameter("max_angular_velocity", 4.5);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        left_axis_client_ = create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        save_raw_map_client_ = create_client<std_srvs::srv::Trigger>("/save_raw_map");
        shutdown_mapping_client_ = create_client<std_srvs::srv::Trigger>("/shutdown_mapping");
        process_map_client_ = create_client<std_srvs::srv::Trigger>("/process_and_save_map");
        
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopNode::update, this));
        
        // Setup terminal for non-blocking input
        setup_terminal();
        
        RCLCPP_INFO(get_logger(), "Teleop started. Controls:");
        RCLCPP_INFO(get_logger(), "  WASD - Move robot");
        RCLCPP_INFO(get_logger(), "  E - Arm motors");
        RCLCPP_INFO(get_logger(), "  Q - Disarm motors");
        RCLCPP_INFO(get_logger(), "  M - Save map, shutdown Fast-LIO2, then process map");
        RCLCPP_INFO(get_logger(), "  Ctrl+C - Exit");
        
        // Check if services are available (non-blocking)
        RCLCPP_INFO(get_logger(), "Checking service availability...");
        
        // Check services without blocking
        if (save_raw_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ save_raw_map service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ save_raw_map service is NOT available (will be checked when M is pressed)");
        }
        
        if (shutdown_mapping_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ shutdown_mapping service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ shutdown_mapping service is NOT available (will be checked when M is pressed)");
        }
        
        if (process_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ process_and_save_map service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ process_and_save_map service is NOT available (will be checked when M is pressed)");
        }
        
        RCLCPP_INFO(get_logger(), "✓ Teleop ready for robot control");
        RCLCPP_INFO(get_logger(), "✓ Press M when ready to save map and shutdown mapping");
    }

    ~TeleopNode() {
        restore_terminal();
    }

    void setup_terminal() {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &old_termios_);
        
        // Create new terminal settings
        new_termios_ = old_termios_;
        new_termios_.c_lflag &= ~(ICANON | ECHO);
        new_termios_.c_cc[VMIN] = 0;
        new_termios_.c_cc[VTIME] = 0;
        
        // Apply new settings
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
        
        // Set stdin to non-blocking
        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    }

    void restore_terminal() {
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }

    char get_key() {
        char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1) {
            return ch;
        }
        return 0;
    }

    void arm_motors() {
        RCLCPP_INFO(get_logger(), "Arming motors (CLOSED_LOOP_CONTROL)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP_CONTROL
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void disarm_motors() {
        RCLCPP_INFO(get_logger(), "Disarming motors (IDLE)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void save_map_and_shutdown() {
        RCLCPP_INFO(get_logger(), "=== STARTING MAP SAVE AND SHUTDOWN SEQUENCE ===");
        
        // Check if services are available before proceeding
        RCLCPP_INFO(get_logger(), "Checking service availability before proceeding...");
        
        if (!save_raw_map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "✗ save_raw_map service is not available!");
            RCLCPP_ERROR(get_logger(), "Make sure the robot is running mapping_only.launch.py");
            return;
        }
        
        if (!shutdown_mapping_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "✗ shutdown_mapping service is not available!");
            RCLCPP_ERROR(get_logger(), "Make sure the robot is running mapping_only.launch.py");
            return;
        }
        
        if (!process_map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "✗ process_and_save_map service is not available!");
            RCLCPP_ERROR(get_logger(), "Make sure the laptop teleop is running with pcd_processor");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "✓ All services are available, proceeding...");
        
        // Step 1: Save raw map from Fast-LIO2
        RCLCPP_INFO(get_logger(), "Step 1: Saving raw map from Fast-LIO2...");
        auto save_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto save_future = save_raw_map_client_->async_send_request(save_request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), save_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto save_response = save_future.get();
            if (save_response->success) {
                RCLCPP_INFO(get_logger(), "✓ Raw map saved: %s", save_response->message.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "✗ Failed to save raw map: %s", save_response->message.c_str());
                return;
            }
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to call save_raw_map service");
            return;
        }
        
        // Step 2: Shutdown Fast-LIO2 and mapping nodes
        RCLCPP_INFO(get_logger(), "Step 2: Shutting down Fast-LIO2 and mapping nodes...");
        auto shutdown_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto shutdown_future = shutdown_mapping_client_->async_send_request(shutdown_request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), shutdown_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto shutdown_response = shutdown_future.get();
            if (shutdown_response->success) {
                RCLCPP_INFO(get_logger(), "✓ %s", shutdown_response->message.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "✗ Failed to shutdown mapping: %s", shutdown_response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to call shutdown service");
        }
        
        // Step 2.5: Copy raw map from robot to laptop
        RCLCPP_INFO(get_logger(), "Step 2.5: Copying raw map from robot to laptop...");
        system("./src/roofus_pilot1/scripts/copy_latest_map.sh");
        RCLCPP_INFO(get_logger(), "✓ Raw map copied to laptop");
        
        // Step 3: Process the saved raw map
        RCLCPP_INFO(get_logger(), "Step 3: Processing saved raw map...");
        auto process_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto process_future = process_map_client_->async_send_request(process_request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), process_future, std::chrono::seconds(30)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto process_response = process_future.get();
            if (process_response->success) {
                RCLCPP_INFO(get_logger(), "✓ %s", process_response->message.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "✗ Failed to process map: %s", process_response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to call process_and_save_map service");
        }
        
        RCLCPP_INFO(get_logger(), "=== MAP SAVE AND PROCESSING COMPLETE ===");
        RCLCPP_INFO(get_logger(), "✓ Fast-LIO2 and mapping nodes are shutdown");
        RCLCPP_INFO(get_logger(), "✓ Raw map has been processed and saved");
        RCLCPP_INFO(get_logger(), "✓ Robot control remains active");
        RCLCPP_INFO(get_logger(), "✓ Use WASD keys to control robot movement");
        RCLCPP_INFO(get_logger(), "✓ Press E to arm motors, Q to disarm");
    }

    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        // Handle keyboard input
        char key = get_key();
        if (key != 0) {
            switch (key) {
                case 'e':
                case 'E':
                    arm_motors();
                    break;
                case 'q':
                case 'Q':
                    disarm_motors();
                    break;
                case 'm':
                case 'M':
                    save_map_and_shutdown();
                    break;
            }
        }

        // Robot teleoperation (WASD) - check if keys are pressed
        if (key == 'w' || key == 'W') {
            cmd_vel_msg.linear.x = 0.5;  // Forward
        } else if (key == 's' || key == 'S') {
            cmd_vel_msg.linear.x = -0.5; // Backward
        }
        if (key == 'a' || key == 'A') {
            cmd_vel_msg.angular.z = -4.5; // Left
        } else if (key == 'd' || key == 'D') {
            cmd_vel_msg.angular.z = 4.5; // Right
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr right_axis_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_raw_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_mapping_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr process_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_termios_, new_termios_;
    int old_flags_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}