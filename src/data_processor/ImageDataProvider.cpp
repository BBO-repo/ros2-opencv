#include "data_processor/ImageDataProvider.hpp"

#include <chrono>
#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

CameraMockerNode::CameraMockerNode() :
    Node("camera_mocker_publisher"), rng_(cv::theRNG())
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_provider", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&CameraMockerNode::timer_callback, this));

    // retrieve vidoe file path through parameter
    declare_parameter("video_path", "");
    video_path_ = get_parameter("video_path").as_string();

    // open video file
    capture_.open(video_path_);
}

void CameraMockerNode::timer_callback() {
    cv::Mat frame, resized_frame;
    capture_.read(frame);
    if (frame.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Media %s finished", video_path_.c_str());
        return;
    }

    cv::resize(frame, resized_frame, cv::Size{cols_, rows_}, 0, 0, cv::INTER_CUBIC);
    // build converts a openCV into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toImageMsg();
    publisher_->publish(*msg_.get());

    RCLCPP_INFO(this->get_logger(), "Image %d published", count_);
    count_++;
}
