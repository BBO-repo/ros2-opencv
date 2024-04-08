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
  int cols_ = 640, rows_ = 480; // image size
  size_t count_; // number of published image
  int x_, y_; // position state
  cv::RNG rng_; // random generator
};