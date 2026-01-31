#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
        // Load parameter values
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        // Initialize angular magnitude used for A/D turns
        ang_mag_ = std::min(1.0, std::max(0.0, max_angular_velocity_));
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        mpc_autonomy_pub_ = create_publisher<std_msgs::msg::Bool>("/mpc_autonomy_enable", 10);
        left_axis_client_ = create_client<odrive_can::srv::AxisState>("/left/request_axis_state");
        right_axis_client_ = create_client<odrive_can::srv::AxisState>("/right/request_axis_state");
        gpr_axis_client_ = create_client<odrive_can::srv::AxisState>("/gpr/request_axis_state");
        save_raw_map_client_ = create_client<std_srvs::srv::Trigger>("/save_raw_map");
        // shutdown_mapping_client_ - REMOVED (not shutting down Fast-LIO2 anymore)
        // process_map_client_ - REMOVED (not using pcd_processor anymore)
        video_record_set_client_ = create_client<std_srvs::srv::SetBool>("/video_record_set");
        // GPR line control services (Arduino)
        gpr_line_start_client_ = create_client<std_srvs::srv::Trigger>("/gpr_line_start");
        gpr_line_stop_client_  = create_client<std_srvs::srv::Trigger>("/gpr_line_stop");
        // GPR scan controller service
        gpr_scan_toggle_client_ = create_client<std_srvs::srv::Trigger>("/gpr_scan/toggle");
        // GPR power off service
        gpr_power_off_client_ = create_client<std_srvs::srv::Trigger>("/gpr_power_off");
        // Rosbag recording toggle service
        rosbag_toggle_client_ = create_client<std_srvs::srv::Trigger>("/rosbag/toggle");
        
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
        RCLCPP_INFO(get_logger(), "  WASD - Move robot (disabled when MPC active)");
        RCLCPP_INFO(get_logger(), "  X - Toggle MPC autonomous control");
        RCLCPP_INFO(get_logger(), "  E - Arm motors");
        RCLCPP_INFO(get_logger(), "  Q - Disarm motors");
        RCLCPP_INFO(get_logger(), "  L - Start GPR line (linear actuator)");
        RCLCPP_INFO(get_logger(), "  K - Stop GPR line (linear actuator)");
        RCLCPP_INFO(get_logger(), "  G - Toggle GPR scan (line + motor + logging)");
        RCLCPP_INFO(get_logger(), "  O - GPR power off");
        RCLCPP_INFO(get_logger(), "  M - Save map checkpoint (Fast-LIO2 continues running)");
        RCLCPP_INFO(get_logger(), "  R - Start recording (both cams)");
        RCLCPP_INFO(get_logger(), "  T - Stop recording (both cams)");
        RCLCPP_INFO(get_logger(), "  B - Toggle rosbag recording");
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
        
        if (gpr_axis_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ GPR ODrive service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ GPR ODrive service is NOT available (E/Q won't affect 3rd motor)");
        }
        
        // Check mapping services
        if (save_raw_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ save_raw_map service is available");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ save_raw_map service is NOT available (will be checked when M is pressed)");
        }

        // Check video record service
        if (video_record_set_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_INFO(get_logger(), "✓ video_record_set service is available (R/T keys)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ video_record_set service is NOT available (R/T will do nothing)");
        }

        // Check GPR line services
        if (gpr_line_start_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ gpr_line_start service is available (L key)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ gpr_line_start service is NOT available (L key will do nothing)");
        }

        if (gpr_line_stop_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ gpr_line_stop service is available (K key)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ gpr_line_stop service is NOT available (K key will do nothing)");
        }

        if (gpr_scan_toggle_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ gpr_scan/toggle service is available (G key)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ gpr_scan/toggle service is NOT available (G key will do nothing)");
        }

        if (gpr_power_off_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ gpr_power_off service is available (O key)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ gpr_power_off service is NOT available (O key will do nothing)");
        }

        if (rosbag_toggle_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "✓ rosbag/toggle service is available (B key)");
        } else {
            RCLCPP_WARN(get_logger(), "⚠ rosbag/toggle service is NOT available (B key will do nothing)");
        }
        
        RCLCPP_INFO(get_logger(), "✓ Teleop ready for robot control");
        RCLCPP_INFO(get_logger(), "✓ Press E to arm motors, Q to disarm, M to save map, G for GPR scan, B for rosbag");
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
        
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 8; // CLOSED_LOOP_CONTROL
        
        // Send async requests with callbacks - no blocking!
        auto left_future = left_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ Left motor armed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ Left motor arm failed: %s", e.what());
                }
            });
        
        auto right_future = right_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ Right motor armed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ Right motor arm failed: %s", e.what());
                }
            });
        
        auto gpr_future = gpr_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ GPR motor armed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ GPR motor arm failed: %s", e.what());
                }
            });
        
        // Suppress unused variable warnings
        (void)left_future;
        (void)right_future;
        (void)gpr_future;
    }

    void disarm_motors() {
        RCLCPP_INFO(get_logger(), "Disarming motors (IDLE)...");
        
        auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
        request->axis_requested_state = 1; // IDLE
        
        // Send async requests with callbacks - no blocking!
        auto left_future = left_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ Left motor disarmed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ Left motor disarm failed: %s", e.what());
                }
            });
        
        auto right_future = right_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ Right motor disarmed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ Right motor disarm failed: %s", e.what());
                }
            });
        
        auto gpr_future = gpr_axis_client_->async_send_request(request,
            [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture resp) {
                try {
                    auto result = resp.get();
                    RCLCPP_INFO(this->get_logger(), "✓ GPR motor disarmed - State: %d, Errors: %d", 
                               result->axis_state, result->active_errors);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "✗ GPR motor disarm failed: %s", e.what());
                }
            });
        
        // Suppress unused variable warnings
        (void)left_future;
        (void)right_future;
        (void)gpr_future;
    }

    // ---------------- GPR line control ----------------
    void trigger_gpr_line_start() {
        if (!gpr_line_start_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "gpr_line_start service not available");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = gpr_line_start_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ Line-UP command acknowledged by Arduino");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Line-UP failed: %s", resp.get()->message.c_str());
                }
            });
        (void)future; // ignore – handled via callback
    }

    void trigger_gpr_line_stop() {
        if (!gpr_line_stop_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "gpr_line_stop service not available");
            return;
        }
        auto req2 = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future2 = gpr_line_stop_client_->async_send_request(req2,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ Line-DOWN command acknowledged by Arduino");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Line-DOWN failed: %s", resp.get()->message.c_str());
                }
            });
        (void)future2;
    }

    void trigger_gpr_scan_toggle() {
        if (!gpr_scan_toggle_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "gpr_scan/toggle service not available");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = gpr_scan_toggle_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ GPR Scan Toggle: %s", resp.get()->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ GPR scan toggle failed: %s", resp.get()->message.c_str());
                }
            });
        (void)future;
    }

    void trigger_gpr_power_off() {
        if (!gpr_power_off_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "gpr_power_off service not available");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = gpr_power_off_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ GPR Power Off: %s", resp.get()->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ GPR power off failed: %s", resp.get()->message.c_str());
                }
            });
        (void)future;
    }

    void trigger_rosbag_toggle() {
        if (!rosbag_toggle_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "rosbag/toggle service not available");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = rosbag_toggle_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ Rosbag Toggle: %s", resp.get()->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ Rosbag toggle failed: %s", resp.get()->message.c_str());
                }
            });
        (void)future;
    }

    void start_map_workflow() {
        if (workflow_active_.load()) {
            RCLCPP_WARN(get_logger(), "Map workflow already in progress, ignoring M key press");
            return;
        }
        
        workflow_active_.store(true);
        workflow_step_.store(0);
        
        RCLCPP_INFO(get_logger(), "=== SAVING MAP CHECKPOINT ===");
        RCLCPP_INFO(get_logger(), "✓ Fast-LIO2 will continue running after save");
        
        // Start the workflow in a separate thread to avoid blocking the main loop
        workflow_thread_ = std::thread(&TeleopNode::execute_map_workflow, this);
    }

private:
    void execute_map_workflow() {
        try {
            // Step 1: Check service availability
            workflow_step_.store(1);
            RCLCPP_INFO(get_logger(), "Checking save_raw_map service availability...");
            
            if (!save_raw_map_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(get_logger(), "✗ save_raw_map service is not available!");
                workflow_active_.store(false);
                return;
            }
            
            RCLCPP_INFO(get_logger(), "✓ Service available, saving map...");
            
            // Step 2: Save raw map from Fast-LIO2
            workflow_step_.store(2);
            RCLCPP_INFO(get_logger(), "Saving raw map from Fast-LIO2...");
            auto save_request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto save_future = save_raw_map_client_->async_send_request(save_request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), save_future, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS) {
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
            
            RCLCPP_INFO(get_logger(), "=== MAP SAVE COMPLETE ===");
            RCLCPP_INFO(get_logger(), "✓ Raw map saved to session folder");
            RCLCPP_INFO(get_logger(), "✓ Fast-LIO2 continues running - you can keep mapping");
            RCLCPP_INFO(get_logger(), "✓ Press M again to save another checkpoint");
            RCLCPP_INFO(get_logger(), "✓ Use WASD keys to control robot movement");
            
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
            } else if (event.type == SDL_KEYDOWN && !event.key.repeat) {
                // Ignore key repeat events to prevent double triggering
                RCLCPP_INFO(get_logger(), "Key pressed: %d", event.key.keysym.sym);
                if (event.key.keysym.sym == SDLK_e) {
                    arm_motors();
                } else if (event.key.keysym.sym == SDLK_q) {
                    disarm_motors();
                } else if (event.key.keysym.sym == SDLK_m) {
                    RCLCPP_INFO(get_logger(), "M key pressed - starting map save sequence!");
                    start_map_workflow();
                } else if (event.key.keysym.sym == SDLK_l) {
                    RCLCPP_INFO(get_logger(), "L key pressed - Line UP");
                    trigger_gpr_line_start();
                } else if (event.key.keysym.sym == SDLK_k) {
                    RCLCPP_INFO(get_logger(), "K key pressed - Line DOWN");
                    trigger_gpr_line_stop();
                } else if (event.key.keysym.sym == SDLK_g) {
                    RCLCPP_INFO(get_logger(), "G key pressed - Toggle GPR Scan");
                    trigger_gpr_scan_toggle();
                } else if (event.key.keysym.sym == SDLK_o) {
                    RCLCPP_INFO(get_logger(), "O key pressed - GPR Power Off");
                    trigger_gpr_power_off();
                } else if (event.key.keysym.sym == SDLK_r) {
                    RCLCPP_INFO(get_logger(), "R key pressed - Start recording (both cams)");
                    send_video_record_set(true);
                } else if (event.key.keysym.sym == SDLK_t) {
                    RCLCPP_INFO(get_logger(), "T key pressed - Stop recording (both cams)");
                    send_video_record_set(false);
                } else if (event.key.keysym.sym == SDLK_b) {
                    RCLCPP_INFO(get_logger(), "B key pressed - Toggle Rosbag Recording");
                    trigger_rosbag_toggle();
                } else if (event.key.keysym.sym == SDLK_0) {
                    // Increase angular velocity magnitude by 0.1 (clamped to max_angular_velocity_)
                    double old = ang_mag_;
                    ang_mag_ = std::min(max_angular_velocity_, ang_mag_ + 0.1);
                    RCLCPP_INFO(get_logger(), "Angular magnitude increased: %.2f -> %.2f rad/s", old, ang_mag_);
                } else if (event.key.keysym.sym == SDLK_9) {
                    // Decrease angular velocity magnitude by 0.1 (clamped to >= 0)
                    double old = ang_mag_;
                    ang_mag_ = std::max(0.0, ang_mag_ - 0.1);
                    RCLCPP_INFO(get_logger(), "Angular magnitude decreased: %.2f -> %.2f rad/s", old, ang_mag_);
                } else if (event.key.keysym.sym == SDLK_x) {
                    // Toggle MPC autonomous control
                    mpc_autonomy_enabled_ = !mpc_autonomy_enabled_;
                    auto msg = std_msgs::msg::Bool();
                    msg.data = mpc_autonomy_enabled_;
                    mpc_autonomy_pub_->publish(msg);
                    if (mpc_autonomy_enabled_) {
                        RCLCPP_INFO(get_logger(), "");
                        RCLCPP_INFO(get_logger(), "╔═══════════════════════════════════════════════════════╗");
                        RCLCPP_INFO(get_logger(), "║  🤖 MPC AUTONOMOUS CONTROL: ENABLED                   ║");
                        RCLCPP_INFO(get_logger(), "║  • WASD teleop DISABLED - MPC controls motors         ║");
                        RCLCPP_INFO(get_logger(), "║  • Other keys (M,R,T,B,G,E,Q) still work              ║");
                        RCLCPP_INFO(get_logger(), "║  • Press X again to return to manual control          ║");
                        RCLCPP_INFO(get_logger(), "╚═══════════════════════════════════════════════════════╝");
                        RCLCPP_INFO(get_logger(), "");
                    } else {
                        RCLCPP_INFO(get_logger(), "");
                        RCLCPP_INFO(get_logger(), "╔═══════════════════════════════════════════════════════╗");
                        RCLCPP_INFO(get_logger(), "║  🎮 MANUAL TELEOP CONTROL: ENABLED                    ║");
                        RCLCPP_INFO(get_logger(), "║  • WASD teleop ACTIVE - manual motor control          ║");
                        RCLCPP_INFO(get_logger(), "║  • MPC autonomous control DISABLED                    ║");
                        RCLCPP_INFO(get_logger(), "║  • Press X to enable MPC autonomous mode              ║");
                        RCLCPP_INFO(get_logger(), "╚═══════════════════════════════════════════════════════╝");
                        RCLCPP_INFO(get_logger(), "");
                    }
                }
            }
        }

        // Only process WASD teleop when MPC autonomous control is DISABLED
        // When MPC is active, we don't send any cmd_vel to avoid fighting for motor control
        if (!mpc_autonomy_enabled_) {
            const Uint8* keys = SDL_GetKeyboardState(NULL);
            // Robot teleoperation (WASD)
            if (keys[SDL_SCANCODE_W]) {
                cmd_vel_msg.linear.x = 0.4;  // Forward
            } else if (keys[SDL_SCANCODE_S]) {
                cmd_vel_msg.linear.x = -0.4; // Backward
            }
            if (keys[SDL_SCANCODE_A]) {
                cmd_vel_msg.angular.z = ang_mag_; // Left (adjustable)
            } else if (keys[SDL_SCANCODE_D]) {
                cmd_vel_msg.angular.z = -ang_mag_; // Right (adjustable)
            }

            cmd_vel_pub_->publish(cmd_vel_msg);
        }
        // Note: When MPC is enabled, we intentionally don't publish cmd_vel
        // This ensures diff_drive_controller doesn't interfere with MPC motor commands
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_autonomy_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr left_axis_client_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr right_axis_client_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr gpr_axis_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_raw_map_client_;
    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_mapping_client_; // REMOVED - not shutting down Fast-LIO2
    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr process_map_client_; // REMOVED - not using pcd_processor
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr video_record_set_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gpr_line_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gpr_line_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gpr_scan_toggle_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gpr_power_off_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rosbag_toggle_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
    
    // Workflow state management
    std::atomic<bool> workflow_active_{false};
    std::atomic<int> workflow_step_{0};
    std::thread workflow_thread_;

    void send_video_record_set(bool enable) {
        if (!video_record_set_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(get_logger(), "video_record_set service not available");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = enable;
        auto future = video_record_set_client_->async_send_request(req,
            [this, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture resp) {
                if (resp.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ Recording %s", enable ? "ON" : "OFF");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Failed to set recording: %s", resp.get()->message.c_str());
                }
            });
        (void)future;
    }
    
    // Teleop tuning
    double max_linear_velocity_{1.5};
    double max_angular_velocity_{3.5};
    double ang_mag_{0.5};
    
    // MPC autonomy state - when true, WASD teleop is disabled and MPC controls motors
    bool mpc_autonomy_enabled_{false};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}