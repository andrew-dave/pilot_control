#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <chrono>

class CmdVelToSerial : public rclcpp::Node {
public:
    CmdVelToSerial() : Node("serial_cmd_bridge") {
        std::string port = declare_parameter<std::string>("port", "/dev/ttyACM1");
        int baud = declare_parameter<int>("baud_rate", 9600);
        std::string topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        try {
            serial_.setPort(port);
            serial_.setBaudrate(baud);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(timeout);
            serial_.open();
            rclcpp::sleep_for(std::chrono::milliseconds(500));  // Allow Arduino to reset
            serial_.flush();

            RCLCPP_INFO(this->get_logger(), "Serial connected on %s at %d baud", port.c_str(), baud);
        } catch (serial::IOException &e) {
            RCLCPP_FATAL(this->get_logger(), "Unable to open serial port %s", port.c_str());
            rclcpp::shutdown();
        }

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            topic, 10,
            std::bind(&CmdVelToSerial::twistCallback, this, std::placeholders::_1));
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int fwd_pwm = 60;
        int turn_pwm = 30;
        int pwmL = 0;
        int pwmR = 0;

        double x = msg->linear.x;
        double z = msg->angular.z;

        if (x > 0.05) {
            pwmL = fwd_pwm;
            pwmR = fwd_pwm;
        } else if (x < -0.05) {
            pwmL = -fwd_pwm;
            pwmR = -fwd_pwm;
        } else if (z > 0.05) {
            pwmL = -turn_pwm;
            pwmR = turn_pwm;
        } else if (z < -0.05) {
            pwmL = turn_pwm;
            pwmR = -turn_pwm;
        } else {
            pwmL = 0;
            pwmR = 0;
        }

        std::string cmd = std::to_string(pwmL) + "," + std::to_string(pwmR) + "\n";

        if (serial_.isOpen()) {
            serial_.write(cmd);
            RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Serial port not open");
        }
    }

    serial::Serial serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToSerial>());
    rclcpp::shutdown();
    return 0;
}
