// Libraries
#include <uuid/uuid.h>
#include <chrono>
#include "hough.hpp"

// Default constructor
HoughLineFollower::HoughLineFollower() : Node("hough_line_follower") {
    // Initialize opencv
    this->camera.open(CAMERA_ID);
    this->is_captured = false;

    // Initialize ros2
    this->timer = this->create_wall_timer(std::chrono::milliseconds(TIMER_DURATION_MS), std::bind(&HoughLineFollower::move, this));
    this->speed_publisher = this->create_publisher<std_msgs::msg::Int16>(SPEED_TOPIC, 10);
    this->move_publisher = this->create_publisher<std_msgs::msg::String>(MOVE_TOPIC, 10);
}

// Default destructor
HoughLineFollower::~HoughLineFollower() { }

// To take an image
void HoughLineFollower::caputreImage() {
    this->is_captured = this->camera.read(this->img);
}

// To save an image
void HoughLineFollower::saveImageToFile() {
    // Safety check
    if (this->is_captured == false) return;
    
    // Generate uuid
    uuid_t uuid;
    char uuid_str[37];
    uuid_generate_random(uuid);

    // Create file name
    uuid_unparse_lower(uuid, uuid_str);
    std::string filename = IMG_DIR + "/" + std::string(uuid_str) + ".png";

    // Save image
    cv::imwrite(filename, this->img);
}

// Control car
void HoughLineFollower::move() {
    caputreImage();
    saveImageToFile();
}