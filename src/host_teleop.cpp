#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

using namespace std::chrono_literals;

class TeleopKeyNode : public rclcpp::Node {
public:
    TeleopKeyNode()
        : Node("teleop_keys") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&TeleopKeyNode::checkKey, this));
        configureTerminal();
    }

    ~TeleopKeyNode() {
        restoreTerminal();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios orig_term_;
    bool key_active_ = false;

    void configureTerminal() {
        tcgetattr(STDIN_FILENO, &orig_term_);
        struct termios new_term = orig_term_;
        new_term.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
    }

    void checkKey() {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            geometry_msgs::msg::Twist msg;
            switch (c) {
                case 'w': msg.linear.x = 0.2; break;
                case 's': msg.linear.x = -0.2; break;
                case 'a': msg.angular.z = 0.5; break;
                case 'd': msg.angular.z = -0.5; break;
                case 'q': rclcpp::shutdown(); return;
                default: return;
            }
            pub_->publish(msg);
            key_active_ = true;
        } else {
            if (key_active_) {
                geometry_msgs::msg::Twist stop;
                pub_->publish(stop);
                key_active_ = false;
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopKeyNode>());
    rclcpp::shutdown();
    return 0;
}
