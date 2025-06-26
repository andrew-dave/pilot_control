#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class TurnTestNode : public rclcpp::Node {
public:
    TurnTestNode() : Node("turn_test") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurnTestNode::update, this));
        
        RCLCPP_INFO(get_logger(), "Turn Test Node Started");
        RCLCPP_INFO(get_logger(), "Press 'A' for left turn, 'D' for right turn, 'S' to stop");
        RCLCPP_INFO(get_logger(), "Press 'Q' to quit");
    }

private:
    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        // Simple test: turn left for 2 seconds, then right for 2 seconds, then stop
        auto now = this->get_clock()->now();
        double elapsed = (now - start_time_).seconds();
        
        if (elapsed < 2.0) {
            // Turn left
            cmd_vel_msg.angular.z = 1.0; // 1 rad/s = ~57 degrees/s
            RCLCPP_INFO(get_logger(), "Turning LEFT: angular_vel = %.1f rad/s", cmd_vel_msg.angular.z);
        } else if (elapsed < 4.0) {
            // Turn right
            cmd_vel_msg.angular.z = -1.0;
            RCLCPP_INFO(get_logger(), "Turning RIGHT: angular_vel = %.1f rad/s", cmd_vel_msg.angular.z);
        } else if (elapsed < 6.0) {
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
    rclcpp::spin(std::make_shared<TurnTestNode>());
    rclcpp::shutdown();
    return 0;
} 