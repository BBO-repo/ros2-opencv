#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#include "data_processor/Inference.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

class DataProcessorNode : public rclcpp::Node {
public:
  DataProcessorNode();
 
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t count_{0};
  std::string yolov5_file_path_{};
  std::unique_ptr<Inference> inf_ptr;
};

#endif // RECOGNIZER_H