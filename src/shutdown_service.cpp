#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cstdlib>
#include <chrono>
#include <signal.h>
#include <unistd.h>

class ShutdownService : public rclcpp::Node {
public:
    ShutdownService() : Node("shutdown_service") {
        // Create service for remote shutdown
        shutdown_service_ = this->create_service<std_srvs::srv::Trigger>(
            "shutdown_mapping",
            std::bind(&ShutdownService::shutdown_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "Shutdown service started - available at /shutdown_mapping");
    }

private:
    void shutdown_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request; // Unused parameter
        
        RCLCPP_INFO(this->get_logger(), "Received shutdown request - killing mapping nodes and signaling launch shutdown...");
        
        // Kill Fast-LIO2 node
        int result1 = system("ros2 node kill /laserMapping");
        if (result1 == 0) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed /laserMapping");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to kill /laserMapping (exit code: %d)", result1);
        }
        
        // Kill Livox driver node
        int result2 = system("ros2 node kill /livox_lidar_publisher");
        if (result2 == 0) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed /livox_lidar_publisher");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to kill /livox_lidar_publisher (exit code: %d)", result2);
        }
        
        // Kill Foxglove bridge
        int result3 = system("ros2 node kill /foxglove_bridge");
        if (result3 == 0) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed /foxglove_bridge");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to kill /foxglove_bridge (exit code: %d)", result3);
        }
        
        // Kill PCD saver if it's running on robot
        int result4 = system("ros2 node kill /pcd_saver");
        if (result4 == 0) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed /pcd_saver");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "PCD saver not running on robot (expected if running on laptop)");
        }
        
        // Wait a moment for nodes to shutdown
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Signal the launch process to shutdown by sending SIGINT to the parent process
        // This will gracefully shutdown the entire launch
        pid_t parent_pid = getppid();
        if (parent_pid > 1) {
            RCLCPP_INFO(this->get_logger(), "Signaling launch process (PID: %d) to shutdown...", parent_pid);
            kill(parent_pid, SIGINT);
        } else {
            // Fallback: try to kill the entire process group
            RCLCPP_INFO(this->get_logger(), "Killing entire process group...");
            killpg(0, SIGINT);
        }
        
        response->success = true;
        response->message = "Mapping nodes killed and launch shutdown signaled. Differential drive controller remains active.";
        
        RCLCPP_INFO(this->get_logger(), "Shutdown request completed - only differential drive controller should remain active");
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShutdownService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 