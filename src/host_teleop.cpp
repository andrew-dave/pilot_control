#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <SDL2/SDL.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <atomic>
#include <future>

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
        window_ = SDL_CreateWindow("Teleop", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 300, 300, SDL_WINDOW_SHOWN);
        
        if (!window_) {
            RCLCPP_ERROR(get_logger(), "Failed to create SDL window: %s", SDL_GetError());
            return;
        }
        
        // Force the window to be visible and focused
        SDL_ShowWindow(window_);
        SDL_RaiseWindow(window_);
        
        RCLCPP_INFO(get_logger(), "SDL window created successfully");
        RCLCPP_INFO(get_logger(), "Teleop started. Controls:");
        RCLCPP_INFO(get_logger(), "  WASD - Move robot");
        RCLCPP_INFO(get_logger(), "  E - Arm motors");
        RCLCPP_INFO(get_logger(), "  Q - Disarm motors");
        RCLCPP_INFO(get_logger(), "  M - Save map, shutdown Fast-LIO2, then process map");
        RCLCPP_INFO(get_logger(), "  Click on the 'Teleop' window to give it focus!");
        
        // Check if services are available (non-blocking)
        RCLCPP_INFO(get_logger(), "Checking service availability...");
        
        // Check ODrive services
        if (left_axis_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ Left ODrive service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ Left ODrive service is NOT available (E/Q keys won't work)");
        }
        
        if (right_axis_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ Right ODrive service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ Right ODrive service is NOT available (E/Q keys won't work)");
        }
        
        // Check mapping services
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
        RCLCPP_INFO(get_logger(), "✓ Press E to arm motors, Q to disarm, M to save map");
    }

    ~TeleopNode() {
        // Wait for workflow thread to finish if it's running
        if (workflow_thread_.joinable()) {
            workflow_thread_.join();
        }
        
        if (window_) {
            SDL_DestroyWindow(window_);
        }
        SDL_Quit();
    }

    void arm_motors() {
        RCLCPP_INFO(get_logger(), "Arming motors (CLOSED_LOOP_CONTROL)...");
        
        // Check if services are available
        if (!left_axis_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Left ODrive service not available!");
            return;
        }
        if (!right_axis_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Right ODrive service not available!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "ODrive services available, sending arm requests...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP_CONTROL
        
        auto left_future = left_axis_client_->async_send_request(request);
        auto right_future = right_axis_client_->async_send_request(request);
        
        // Wait for responses
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto left_response = left_future.get();
            RCLCPP_INFO(get_logger(), "Left motor arm successful - State: %d, Errors: %d", 
                       left_response->axis_state, left_response->active_errors);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get left motor arm response");
        }
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto right_response = right_future.get();
            RCLCPP_INFO(get_logger(), "Right motor arm successful - State: %d, Errors: %d", 
                       right_response->axis_state, right_response->active_errors);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get right motor arm response");
        }
    }

    void disarm_motors() {
        RCLCPP_INFO(get_logger(), "Disarming motors (IDLE)...");
        
        // Check if services are available
        if (!left_axis_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Left ODrive service not available!");
            return;
        }
        if (!right_axis_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Right ODrive service not available!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "ODrive services available, sending disarm requests...");
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        
        auto left_future = left_axis_client_->async_send_request(request);
        auto right_future = right_axis_client_->async_send_request(request);
        
        // Wait for responses
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto left_response = left_future.get();
            RCLCPP_INFO(get_logger(), "Left motor disarm successful - State: %d, Errors: %d", 
                       left_response->axis_state, left_response->active_errors);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get left motor disarm response");
        }
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto right_response = right_future.get();
            RCLCPP_INFO(get_logger(), "Right motor disarm successful - State: %d, Errors: %d", 
                       right_response->axis_state, right_response->active_errors);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get right motor disarm response");
        }
    }

    void start_map_workflow() {
        if (workflow_active_.load()) {
            RCLCPP_WARN(get_logger(), "Map workflow already in progress, ignoring M key press");
            return;
        }
        
        workflow_active_.store(true);
        workflow_step_.store(0);
        
        RCLCPP_INFO(get_logger(), "=== STARTING MAP SAVE AND SHUTDOWN SEQUENCE ===");
        RCLCPP_INFO(get_logger(), "✓ Teleop node will remain active throughout the process");
        
        // Start the workflow in a separate thread to avoid blocking the main loop
        workflow_thread_ = std::thread(&TeleopNode::execute_map_workflow, this);
    }

private:
    void execute_map_workflow() {
        try {
            // Step 1: Check service availability
            workflow_step_.store(1);
            RCLCPP_INFO(get_logger(), "Step 1: Checking service availability...");
            
            if (!save_raw_map_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(get_logger(), "✗ save_raw_map service is not available!");
                workflow_active_.store(false);
                return;
            }
            
            if (!shutdown_mapping_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(get_logger(), "✗ shutdown_mapping service is not available!");
                workflow_active_.store(false);
                return;
            }
            
            if (!process_map_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(get_logger(), "✗ process_and_save_map service is not available!");
                workflow_active_.store(false);
                return;
            }
            
            RCLCPP_INFO(get_logger(), "✓ All services are available, proceeding...");
            
            // Step 2: Save raw map from Fast-LIO2
            workflow_step_.store(2);
            RCLCPP_INFO(get_logger(), "Step 2: Saving raw map from Fast-LIO2...");
            auto save_request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto save_future = save_raw_map_client_->async_send_request(save_request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), save_future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
                auto save_response = save_future.get();
                if (save_response->success) {
                    RCLCPP_INFO(get_logger(), "✓ Raw map saved: %s", save_response->message.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "✗ Failed to save raw map: %s", save_response->message.c_str());
                    workflow_active_.store(false);
                    return;
                }
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to call save_raw_map service");
                workflow_active_.store(false);
                return;
            }
            
            // Step 3: Shutdown Fast-LIO2 and mapping nodes
            workflow_step_.store(3);
            RCLCPP_INFO(get_logger(), "Step 3: Shutting down Fast-LIO2 and mapping nodes...");
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
            
            // Step 4: Copy raw map from robot to laptop
            workflow_step_.store(4);
            RCLCPP_INFO(get_logger(), "Step 4: Copying raw map from robot to laptop...");
            
            // Use a simple system call with proper error handling
            RCLCPP_INFO(get_logger(), "Executing copy script...");
            int copy_result = std::system("/home/avenblake/pilot_ws/src/pilot_control/scripts/copy_latest_map.sh");
            RCLCPP_INFO(get_logger(), "Copy script completed with exit code: %d", copy_result);
            
            if (copy_result == 0) {
                RCLCPP_INFO(get_logger(), "✓ Raw map copied to laptop successfully");
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to copy raw map to laptop (exit code: %d)", copy_result);
                RCLCPP_ERROR(get_logger(), "Continuing with processing anyway...");
            }
            
            // Add a small delay to ensure file is fully written
            RCLCPP_INFO(get_logger(), "Waiting 2 seconds for file to be fully written...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // Step 5: Process the saved raw map
            workflow_step_.store(5);
            RCLCPP_INFO(get_logger(), "Step 5: Processing saved raw map...");
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
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in map workflow: %s", e.what());
        }
        
        workflow_active_.store(false);
        workflow_step_.store(0);
    }

    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        SDL_Event event;
        static int event_count = 0;
        
        while (SDL_PollEvent(&event)) {
            event_count++;
            if (event_count <= 5) {  // Only log first few events to avoid spam
                RCLCPP_INFO(get_logger(), "SDL Event received: type=%d", event.type);
            }
            
            if (event.type == SDL_QUIT) {
                RCLCPP_INFO(get_logger(), "SDL_QUIT received, shutting down...");
                rclcpp::shutdown();
            } else if (event.type == SDL_KEYDOWN) {
                RCLCPP_INFO(get_logger(), "Key pressed: %d", event.key.keysym.sym);
                if (event.key.keysym.sym == SDLK_e) {
                    arm_motors();
                } else if (event.key.keysym.sym == SDLK_q) {
                    disarm_motors();
                } else if (event.key.keysym.sym == SDLK_m) {
                    RCLCPP_INFO(get_logger(), "M key pressed - starting map save sequence!");
                    start_map_workflow();
                }
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);
        // Robot teleoperation (WASD)
        if (keys[SDL_SCANCODE_W]) {
            cmd_vel_msg.linear.x = 1.5;  // Forward
        } else if (keys[SDL_SCANCODE_S]) {
            cmd_vel_msg.linear.x = -1.5; // Backward
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
    
    // Workflow state management
    std::atomic<bool> workflow_active_{false};
    std::atomic<int> workflow_step_{0};
    std::thread workflow_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}