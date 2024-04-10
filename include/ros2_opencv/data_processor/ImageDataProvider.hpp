#ifndef IMAGE_DATA_PROVIDER_H
#define IMAGE_DATA_PROVIDER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>

class CameraMockerNode : public rclcpp::Node {
public:
  CameraMockerNode();
 
private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  int cols_{640}, rows_{480}; // image size
  int count_{0}; // number of published image
  cv::RNG rng_{}; // random generator
  std::string video_path_{};
  cv::VideoCapture capture_{};
};

#endif // IMAGE_DATA_PROVIDER_H
