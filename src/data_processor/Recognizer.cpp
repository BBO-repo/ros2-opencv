#include "data_processor/Recognizer.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

DataProcessorNode::DataProcessorNode() : 
    Node("opencv_image_processor")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_provider", 10, std::bind(&DataProcessorNode::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
    declare_parameter("yolov5_file_path", "");
    yolov5_file_path_ = get_parameter("yolov5_file_path").as_string();
    inf_ptr = std::make_unique<Inference>(yolov5_file_path_);
    RCLCPP_INFO(get_logger(), "Using yolov5 file: %s", yolov5_file_path_.c_str());
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
    // Process image
    std::vector<Detection> output = inf_ptr->runInference(image_raw);
    for (size_t i = 0; i < output.size(); ++i)
    {
        Detection detection = output[i];

        cv::Rect box = detection.box;
        cv::Scalar color = detection.color;

        // Detection box
        cv::rectangle(image_raw, box, color, 2);

        // Detection box text
        std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(image_raw, textBox, color, cv::FILLED);
        cv::putText(image_raw, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }

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
