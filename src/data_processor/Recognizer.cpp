#include "data_processor/Recognizer.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

DataProcessorNode::DataProcessorNode() : Node("opencv_image_processor"), count_(0)
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_provider", 10, std::bind(&DataProcessorNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
}

void DataProcessorNode::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image: %ld", count_);
    count_++;

    // convert ROS image to openCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    // image processing
    cv::Mat image_raw = cv_ptr->image;
    int x, y;
    std::tie(x, y) = find_ball_position(image_raw);
    update_trajectory(x, y);
    draw_trajectory(image_raw);

    // append some descriptive text
    cv::putText(image_raw,
        "basic OpenCV image processing of ball tracking", //text
        cv::Point(10, 30), //top-left position
        cv::FONT_HERSHEY_SIMPLEX,
        0.75, cv::Scalar(0, 0, 0)/*font color*/, 2);
    cv::putText(image_raw,
        "from an image published through ROS2", //text
        cv::Point(10, 60), //top-left position
        cv::FONT_HERSHEY_SIMPLEX,
        0.75, cv::Scalar(0, 0, 0)/*font color*/, 2);
    cv::putText(image_raw,
        "bridged to OpenCV and republished through ROS2", //text
        cv::Point(10, 90), //top-left position
        cv::FONT_HERSHEY_SIMPLEX,
        0.75, cv::Scalar(0, 0, 0)/*font color*/, 2);

    // convert openCV image to ROS image
    cv_bridge::CvImage img_bridge =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image_raw);

    // message to be sent
    sensor_msgs::msg::Image out_image;

    // from cv_bridge to sensor_msgs::Image
    img_bridge.toImageMsg(out_image);

    // publish image
    publisher_->publish(out_image);
}

std::tuple<int, int> DataProcessorNode::find_ball_position(const cv::Mat &image)
{
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
    gray.rows/16, // change this value to detect circles with different distances to each other
    100, 30, 10, 50 // change the last two parameters
    // (min_radius & max_radius) to detect larger circles
    );    
    return circles.size() > 0 ? std::make_tuple((int)circles[0][0], (int)circles[0][1]) : std::make_tuple(0, 0);
}

void DataProcessorNode::update_trajectory(int x, int y)
{
    trajectory_[0][count_-1] = x;
    trajectory_[1][count_-1] = y;
}

void DataProcessorNode::draw_trajectory(cv::Mat &image)
{
    // draw a single cross at every point
    for(size_t ic = 0; ic < count_; ++ic)
    {
        // current coordinate
        int xc = trajectory_[0][ic];
        int yc = trajectory_[1][ic];
        // drawing the cross
        cv::line(image, cv::Point(xc-3, yc-3), cv::Point(xc+3, yc+3), cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(xc-3, yc+3), cv::Point(xc+3, yc-3), cv::Scalar(0, 255, 0), 1);
    }

    // fill between cross to draw trajectory if more than one point in trajectory
    if(count_ >= 2)
    {
        for(size_t ic = 1; ic < count_; ++ic)
        {
            // drawing the line between two point
            cv::line(image,
                     cv::Point(trajectory_[0][ic-1], trajectory_[1][ic-1]),
                     cv::Point(trajectory_[0][ic], trajectory_[1][ic]),
                     cv::Scalar(0, 255, 0), 2);
        }
    }
}
