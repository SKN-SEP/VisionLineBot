#ifndef HOUGH_H
#define HOUGH_H

// Libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <string>

// Constants
const int CAMERA_ID = 16;
const int TIMER_DURATION_MS = 1000;
inline const std::string SPEED_TOPIC = "/cv_speed";
inline const std::string MOVE_TOPIC = "/cv_move";
inline const std::string IMG_DIR = "/home/pi/VisionLineBotSrc/VisionLineBot/vision/images";

// Hough based line follower algorithm
class HoughLineFollower : public rclcpp::Node {
    private:
    // OpenCV variables
    cv::VideoCapture camera;
    cv::Mat img;
    bool is_captured;

    // ROS2 variables
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_publisher;

    public:
    // Default constructor
    HoughLineFollower();

    // Default destructor
    ~HoughLineFollower();

    // To take an image
    void caputreImage();

    // To save an image
    void saveImageToFile();

    // Control car
    void move();
};

#endif 