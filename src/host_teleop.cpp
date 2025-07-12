#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <SDL2/SDL.h>
#include <chrono>

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("host_teleop") {
        // Declare a parameter to prevent Foxglove bridge errors
        this->declare_parameter("teleop_active", true);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        left_axis_client_ = create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        save_map_client_ = create_client<std_srvs::srv::Trigger>("/manual_save_map");
        shutdown_client_ = create_client<std_srvs::srv::Trigger>("/shutdown_mapping");
        
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopNode::update, this));
        
        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow("Teleop", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 300, 300, 0);
        
        RCLCPP_INFO(get_logger(), "Teleop started. Controls:");
        RCLCPP_INFO(get_logger(), "  WASD - Move robot");
        RCLCPP_INFO(get_logger(), "  E - Arm motors");
        RCLCPP_INFO(get_logger(), "  Q - Disarm motors");
        RCLCPP_INFO(get_logger(), "  M - Save map once and shutdown mapping");
        
        // Check if services are available
        RCLCPP_INFO(get_logger(), "Checking service availability...");
        if (save_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ Manual save map service is available");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Manual save map service is NOT available");
        }
        
        if (shutdown_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ Shutdown service is available");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Shutdown service is NOT available");
        }
    }

    ~TeleopNode() {
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

private:
    void arm_motors() {
        RCLCPP_INFO(get_logger(), "Arming motors by keypress (E)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // AXIS_STATE_CLOSED_LOOP
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void disarm_motors() {
        RCLCPP_INFO(get_logger(), "Disarming motors by keypress (Q)...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // AXIS_STATE_IDLE
        left_axis_client_->async_send_request(request);
        right_axis_client_->async_send_request(request);
    }

    void save_map_and_shutdown() {
        RCLCPP_INFO(get_logger(), "=== M KEY PRESSED: SAVING MAP ONCE AND SHUTTING DOWN MAPPING ===");
        
        // Check if services are still available before calling them
        RCLCPP_INFO(get_logger(), "Checking service availability before calling...");
        if (!save_map_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_ERROR(get_logger(), "✗ Manual save map service is not available!");
            return;
        }
        
        if (!shutdown_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_ERROR(get_logger(), "✗ Shutdown service is not available!");
            return;
        }
        
        // First, save the map with custom filename
        RCLCPP_INFO(get_logger(), "Step 1: Calling manual save map service...");
        auto save_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto save_future = save_map_client_->async_send_request(save_request);
        
        RCLCPP_INFO(get_logger(), "Waiting for manual save map response...");
        // Wait for the save to complete with shorter timeout
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), save_future, std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto save_response = save_future.get();
            if (save_response->success) {
                RCLCPP_INFO(get_logger(), "✓ Map saved successfully: %s", save_response->message.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "✗ Failed to save map: %s", save_response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to call manual save map service - service may not be available");
        }
        
        // Shutdown mapping nodes via service call
        RCLCPP_INFO(get_logger(), "Step 2: Calling shutdown service...");
        auto shutdown_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto shutdown_future = shutdown_client_->async_send_request(shutdown_request);
        
        RCLCPP_INFO(get_logger(), "Waiting for shutdown response...");
        // Wait for the shutdown to complete with shorter timeout
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), shutdown_future, std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto shutdown_response = shutdown_future.get();
            if (shutdown_response->success) {
                RCLCPP_INFO(get_logger(), "✓ %s", shutdown_response->message.c_str());
                RCLCPP_INFO(get_logger(), "✓ Teleop window will remain open for robot control");
                RCLCPP_INFO(get_logger(), "✓ Use WASD keys to control robot movement");
                RCLCPP_INFO(get_logger(), "✓ Press E to arm motors, Q to disarm");
            } else {
                RCLCPP_WARN(get_logger(), "✗ Failed to shutdown mapping nodes: %s", shutdown_response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to call shutdown service - service may not be available");
        }
        
        RCLCPP_INFO(get_logger(), "=== MAPPING SHUTDOWN COMPLETE - TELEOP REMAINS ACTIVE ===");
        
        // Don't close the teleop window - keep it open for robot control
        RCLCPP_INFO(get_logger(), "Teleop window remains open for robot control...");
    }

    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                rclcpp::shutdown();
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_e) {
                    arm_motors();
                } else if (event.key.keysym.sym == SDLK_q) {
                    disarm_motors();
                } else if (event.key.keysym.sym == SDLK_m) {
                    save_map_and_shutdown();
                }
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);
        // Robot teleoperation (WASD)
        if (keys[SDL_SCANCODE_W]) {
            cmd_vel_msg.linear.x = 1.0;  // Forward
        } else if (keys[SDL_SCANCODE_S]) {
            cmd_vel_msg.linear.x = -1.0; // Backward
        }
        if (keys[SDL_SCANCODE_A]) {
            cmd_vel_msg.angular.z = -4.5; // Left
        } else if (keys[SDL_SCANCODE_D]) {
            cmd_vel_msg.angular.z = 4.5; // Right
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr right_axis_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}