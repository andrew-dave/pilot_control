#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>

class NavigationTestNode : public rclcpp::Node {
public:
    NavigationTestNode() : Node("navigation_test") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationTestNode::odom_callback, this, std::placeholders::_1));
        
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&NavigationTestNode::update, this));
        
        // Initialize state
        start_x_ = 0.0;
        start_y_ = 0.0;
        start_theta_ = 0.0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;
        state_ = State::WAITING_FOR_ODOM;
        
        RCLCPP_INFO(get_logger(), "Navigation Test Node Started");
        RCLCPP_INFO(get_logger(), "Path: 1m straight -> stop -> left turn -> 0.5m -> left turn -> 1m straight");
    }

private:
    enum class State {
        WAITING_FOR_ODOM,
        GOING_STRAIGHT_1M,
        STOPPING_1,
        TURNING_LEFT_1,
        GOING_STRAIGHT_0_5M,
        STOPPING_2,
        TURNING_LEFT_2,
        GOING_STRAIGHT_1M_FINAL,
        STOPPING_FINAL,
        COMPLETE
    };
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_theta_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        if (state_ == State::WAITING_FOR_ODOM) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_theta_ = current_theta_;
            state_ = State::GOING_STRAIGHT_1M;
            RCLCPP_INFO(get_logger(), "Starting navigation test");
        }
    }
    
    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        switch (state_) {
            case State::WAITING_FOR_ODOM:
                // Wait for first odometry message
                break;
                
            case State::GOING_STRAIGHT_1M: {
                double distance_traveled = sqrt(pow(current_x_ - start_x_, 2) + pow(current_y_ - start_y_, 2));
                if (distance_traveled >= 1.0) {
                    state_ = State::STOPPING_1;
                    RCLCPP_INFO(get_logger(), "Reached 1m, stopping...");
                } else {
                    cmd_vel_msg.linear.x = 0.5; // 0.5 m/s forward
                    RCLCPP_INFO(get_logger(), "Going straight: %.2f/1.00m", distance_traveled);
                }
                break;
            }
            
            case State::STOPPING_1: {
                static int stop_counter = 0;
                stop_counter++;
                if (stop_counter > 20) { // Stop for 1 second (20 * 50ms)
                    state_ = State::TURNING_LEFT_1;
                    start_theta_ = current_theta_;
                    stop_counter = 0;
                    RCLCPP_INFO(get_logger(), "Starting left turn 1...");
                }
                break;
            }
            
            case State::TURNING_LEFT_1: {
                double angle_turned = current_theta_ - start_theta_;
                // Normalize angle to [-π, π]
                while (angle_turned > M_PI) angle_turned -= 2 * M_PI;
                while (angle_turned < -M_PI) angle_turned += 2 * M_PI;
                
                if (angle_turned >= M_PI/2) { // 90 degrees = π/2 radians
                    state_ = State::GOING_STRAIGHT_0_5M;
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    RCLCPP_INFO(get_logger(), "Turn 1 complete, going 0.5m...");
                } else {
                    cmd_vel_msg.angular.z = 1.0; // 1 rad/s left turn
                    RCLCPP_INFO(get_logger(), "Turning left 1: %.1f/90 degrees", angle_turned * 180 / M_PI);
                }
                break;
            }
            
            case State::GOING_STRAIGHT_0_5M: {
                double distance_traveled = sqrt(pow(current_x_ - start_x_, 2) + pow(current_y_ - start_y_, 2));
                if (distance_traveled >= 0.5) {
                    state_ = State::STOPPING_2;
                    RCLCPP_INFO(get_logger(), "Reached 0.5m, stopping...");
                } else {
                    cmd_vel_msg.linear.x = 0.5; // 0.5 m/s forward
                    RCLCPP_INFO(get_logger(), "Going straight: %.2f/0.50m", distance_traveled);
                }
                break;
            }
            
            case State::STOPPING_2: {
                static int stop_counter = 0;
                stop_counter++;
                if (stop_counter > 20) { // Stop for 1 second
                    state_ = State::TURNING_LEFT_2;
                    start_theta_ = current_theta_;
                    stop_counter = 0;
                    RCLCPP_INFO(get_logger(), "Starting left turn 2...");
                }
                break;
            }
            
            case State::TURNING_LEFT_2: {
                double angle_turned = current_theta_ - start_theta_;
                // Normalize angle to [-π, π]
                while (angle_turned > M_PI) angle_turned -= 2 * M_PI;
                while (angle_turned < -M_PI) angle_turned += 2 * M_PI;
                
                if (angle_turned >= M_PI/2) { // 90 degrees
                    state_ = State::GOING_STRAIGHT_1M_FINAL;
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    RCLCPP_INFO(get_logger(), "Turn 2 complete, going final 1m...");
                } else {
                    cmd_vel_msg.angular.z = 1.0; // 1 rad/s left turn
                    RCLCPP_INFO(get_logger(), "Turning left 2: %.1f/90 degrees", angle_turned * 180 / M_PI);
                }
                break;
            }
            
            case State::GOING_STRAIGHT_1M_FINAL: {
                double distance_traveled = sqrt(pow(current_x_ - start_x_, 2) + pow(current_y_ - start_y_, 2));
                if (distance_traveled >= 1.0) {
                    state_ = State::STOPPING_FINAL;
                    RCLCPP_INFO(get_logger(), "Reached final 1m, stopping...");
                } else {
                    cmd_vel_msg.linear.x = 0.5; // 0.5 m/s forward
                    RCLCPP_INFO(get_logger(), "Going straight final: %.2f/1.00m", distance_traveled);
                }
                break;
            }
            
            case State::STOPPING_FINAL: {
                static int stop_counter = 0;
                stop_counter++;
                if (stop_counter > 20) { // Stop for 1 second
                    state_ = State::COMPLETE;
                    stop_counter = 0;
                    RCLCPP_INFO(get_logger(), "Navigation test complete!");
                }
                break;
            }
            
            case State::COMPLETE:
                // Test complete, do nothing
                break;
        }
        
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    State state_;
    double start_x_, start_y_, start_theta_;
    double current_x_, current_y_, current_theta_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationTestNode>());
    rclcpp::shutdown();
    return 0;
} 