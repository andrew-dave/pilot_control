#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class TorqueTestNode : public rclcpp::Node {
public:
    TorqueTestNode() : Node("torque_test") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TorqueTestNode::update, this));
        
        RCLCPP_INFO(get_logger(), "Torque Test Node Started");
        RCLCPP_INFO(get_logger(), "This will test turning with different angular velocities");
        RCLCPP_INFO(get_logger(), "Watch the robot's turning performance and adjust torque_feedforward parameter");
    }

private:
    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        // Test sequence: different angular velocities
        auto now = this->get_clock()->now();
        double elapsed = (now - start_time_).seconds();
        
        if (elapsed < 3.0) {
            // Gentle turn left
            cmd_vel_msg.angular.z = 0.5; // 0.5 rad/s = ~29 degrees/s
            RCLCPP_INFO(get_logger(), "Gentle LEFT turn: angular_vel = %.1f rad/s", cmd_vel_msg.angular.z);
        } else if (elapsed < 6.0) {
            // Medium turn right
            cmd_vel_msg.angular.z = -1.0; // 1.0 rad/s = ~57 degrees/s
            RCLCPP_INFO(get_logger(), "Medium RIGHT turn: angular_vel = %.1f rad/s", cmd_vel_msg.angular.z);
        } else if (elapsed < 9.0) {
            // Fast turn left
            cmd_vel_msg.angular.z = 2.0; // 2.0 rad/s = ~115 degrees/s
            RCLCPP_INFO(get_logger(), "Fast LEFT turn: angular_vel = %.1f rad/s", cmd_vel_msg.angular.z);
        } else if (elapsed < 12.0) {
            // Stop
            cmd_vel_msg.angular.z = 0.0;
            RCLCPP_INFO(get_logger(), "STOPPED");
        } else {
            // End test
            RCLCPP_INFO(get_logger(), "Test complete. Shutting down...");
            rclcpp::shutdown();
            return;
        }
        
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_ = this->get_clock()->now();
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueTestNode>());
    rclcpp::shutdown();
    return 0;
} 