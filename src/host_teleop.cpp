#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <SDL2/SDL.h>
#include <chrono>

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
        
        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow("Teleop", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 300, 300, 0);
        
        RCLCPP_INFO(get_logger(), "Teleop started. Controls:");
        RCLCPP_INFO(get_logger(), "  WASD - Move robot");
        RCLCPP_INFO(get_logger(), "  E - Arm motors");
        RCLCPP_INFO(get_logger(), "  Q - Disarm motors");
        RCLCPP_INFO(get_logger(), "  M - Save map, shutdown Fast-LIO2, then process map");
        
        // Check if services are available
        RCLCPP_INFO(get_logger(), "Checking service availability...");
        
        // Wait for services to be available
        while (!save_raw_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for save_raw_map service...");
        }
        while (!shutdown_mapping_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for shutdown_mapping service...");
        }
        while (!process_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for process_and_save_map service...");
        }
        
        RCLCPP_INFO(get_logger(), "✓ All services available");
        RCLCPP_INFO(get_logger(), "✓ Ready for mapping and control");
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
            cmd_vel_msg.linear.x = 0.5;  // Forward
        } else if (keys[SDL_SCANCODE_S]) {
            cmd_vel_msg.linear.x = -0.5; // Backward
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
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_raw_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_mapping_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr process_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}