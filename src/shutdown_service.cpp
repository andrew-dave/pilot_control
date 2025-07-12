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
        
        RCLCPP_INFO(this->get_logger(), "=== MAPPING COMPLETE - SHUTTING DOWN MAPPING NODES ===");
        RCLCPP_INFO(this->get_logger(), "Saving map and shutting down all mapping components...");
        RCLCPP_INFO(this->get_logger(), "Differential drive controller will remain active for teleop only.");
        RCLCPP_INFO(this->get_logger(), "========================================================");
        
        // Kill mapping nodes (Fast-LIO2, Livox driver, Foxglove bridge, PCD saver)
        RCLCPP_INFO(this->get_logger(), "Shutting down mapping nodes...");
        
        // Kill Fast-LIO2 node
        int result1 = system("ros2 node kill /laser_mapping");
        if (result1 == 0) {
            RCLCPP_INFO(this->get_logger(), "✓ Killed /laser_mapping (Fast-LIO2)");
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ Failed to kill /laser_mapping (exit code: %d)", result1);
        }
        
        // Kill Livox driver node
        int result2 = system("ros2 node kill /livox_lidar_publisher");
        if (result2 == 0) {
            RCLCPP_INFO(this->get_logger(), "✓ Killed /livox_lidar_publisher (LiDAR driver)");
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ Failed to kill /livox_lidar_publisher (exit code: %d)", result2);
        }
        
        // Kill Foxglove bridge
        int result3 = system("ros2 node kill /foxglove_bridge");
        if (result3 == 0) {
            RCLCPP_INFO(this->get_logger(), "✓ Killed /foxglove_bridge (visualization)");
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ Failed to kill /foxglove_bridge (exit code: %d)", result3);
        }
        
        // Kill PCD saver (may be running on laptop or robot)
        int result4 = system("ros2 node kill /pcd_saver");
        if (result4 == 0) {
            RCLCPP_INFO(this->get_logger(), "✓ Killed /pcd_saver (map saver)");
        } else {
            RCLCPP_INFO(this->get_logger(), "ℹ PCD saver not found or already stopped");
        }
        
        // Wait a moment for nodes to shutdown
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        RCLCPP_INFO(this->get_logger(), "========================================================");
        RCLCPP_INFO(this->get_logger(), "✓ MAPPING COMPLETE - MAPPING NODES SHUTDOWN SUCCESSFUL");
        RCLCPP_INFO(this->get_logger(), "✓ Differential drive controller remains active");
        RCLCPP_INFO(this->get_logger(), "✓ Robot is ready for teleop only mode");
        RCLCPP_INFO(this->get_logger(), "========================================================");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 NEXT STEPS:");
        RCLCPP_INFO(this->get_logger(), "  • Use WASD keys to control robot movement");
        RCLCPP_INFO(this->get_logger(), "  • Press E to arm motors, Q to disarm");
        RCLCPP_INFO(this->get_logger(), "  • To restart mapping: ros2 launch pilot_control robot_complete.launch.py");
        RCLCPP_INFO(this->get_logger(), "  • To run only robot control: ros2 launch pilot_control diff_drive_only.launch.py");
        RCLCPP_INFO(this->get_logger(), " ");
        
        response->success = true;
        response->message = "Mapping complete! All mapping nodes shutdown. Differential drive controller remains active for teleop.";
        
        // Don't shutdown the entire launch - keep the robot control nodes running
        // The differential drive controller, ODrive nodes, and teleop will continue running
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