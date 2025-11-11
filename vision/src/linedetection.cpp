// Libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "hough.hpp"

// Entry point
int main(int argc, char *argv[]) {
    // Run ROS2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoughLineFollower>());
    rclcpp::shutdown();
    return 0;
}