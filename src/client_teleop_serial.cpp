#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <string>
#include <algorithm>

class CmdVelToSerial : public rclcpp::Node {
public:
    CmdVelToSerial() : Node("serial_cmd_bridge") {
        declare_parameter<std::string>("port", "/dev/ttyACM0");
        declare_parameter<int>("baud_rate", 9600);
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<double>("wheel_separation", 0.28);  // meters
        declare_parameter<double>("wheel_radius", 0.0508);    // meters
        declare_parameter<double>("max_angular_speed", 62.83); // rad/s

        std::string port = get_parameter("port").as_string();
        int baud = get_parameter("baud_rate").as_int();
        topic_ = get_parameter("cmd_vel_topic").as_string();
        wheel_separation_ = get_parameter("wheel_separation").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        max_omega_ = get_parameter("max_angular_speed").as_double();

        try {
            serial_.setPort(port);
            serial_.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "Serial connected on %s at %d baud", port.c_str(), baud);
        } catch (serial::IOException &e) {
            RCLCPP_FATAL(this->get_logger(), "Unable to open serial port %s", port.c_str());
            rclcpp::shutdown();
        }

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            topic_, 10, std::bind(&CmdVelToSerial::twistCallback, this, std::placeholders::_1));
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int forward_pwm = 150;
        int turn_pwm = 120;

        int pwmL = 0;
        int pwmR = 0;

        double lin = msg->linear.x;
        double ang = msg->angular.z;

        if (lin > 0.05) {
            pwmL = forward_pwm;
            pwmR = forward_pwm;
        } else if (lin < -0.05) {
            pwmL = -forward_pwm;
            pwmR = -forward_pwm;
        } else if (ang > 0.05) {
            pwmL = -turn_pwm;
            pwmR = turn_pwm;
        } else if (ang < -0.05) {
            pwmL = turn_pwm;
            pwmR = -turn_pwm;
        } else {
            pwmL = 0;
            pwmR = 0;
        }

        std::string data = std::to_string(pwmL) + "," + std::to_string(pwmR) + "\n";
        serial_.write(data);
        RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
    }

    serial::Serial serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    std::string topic_;
    double wheel_separation_, wheel_radius_, max_omega_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToSerial>());
    rclcpp::shutdown();
    return 0;
}
