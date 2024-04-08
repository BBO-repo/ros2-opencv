#include "data_processor/ImageDataProvider.hpp"

#include <chrono>
#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

CameraMockerNode::CameraMockerNode() : Node("camera_mocker_publisher"), count_(0), rng_(cv::theRNG())
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_provider", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&CameraMockerNode::timer_callback, this));

    // start at a random position
    x_ = rng_.uniform(0,cols_);
    y_ = this->rng_.uniform(0,rows_);
}

void CameraMockerNode::timer_callback() {
    
    // build a new 640x480 image
    cv::Mat img(cv::Size(cols_, rows_), CV_8UC3, cv::Scalar(255, 255, 255));

    // move ball randomly
    int dx = this->rng_.uniform(0,17) - 5;
    int dy = this->rng_.uniform(0,17) - 5;

    x_ = x_ + dx < 0 ? 0 : (x_ + dx > cols_ ? cols_ : x_ + dx ); 
    y_ = y_ + dy < 0 ? 0 : (y_ + dy > rows_ ? rows_ : y_ + dy ); 

    // draw ball with updated position
    cv::circle(img, cv::Point(x_, y_), 30, CV_RGB(255,0,0), cv::FILLED, 10);

    // build converts a openCV into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
}
