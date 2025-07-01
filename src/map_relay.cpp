#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapRelay : public rclcpp::Node
{
public:
    MapRelay() : Node("map_relay")
    {
        // Create publisher for /map with transient local QoS
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", 
            rclcpp::QoS(10).transient_local()
        );
        
        // Create subscriber for /Laser_map with volatile durability to match Fast-LIO2
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/Laser_map",
            rclcpp::QoS(10).durability_volatile(),
            std::bind(&MapRelay::map_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Map relay started - relaying /Laser_map to /map");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Republish the map with updated header
        auto relayed_msg = *msg;
        relayed_msg.header.stamp = this->get_clock()->now();
        map_pub_->publish(relayed_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 