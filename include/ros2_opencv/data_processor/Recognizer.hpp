#include <tuple>

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
  size_t count_;
  // keep the last 1000 position
  int trajectory_[2][1000];
  // utils to update and plot trajectory in published image
  std::tuple<int, int> find_ball_position(const cv::Mat &image);
  void update_trajectory(int x, int y);
  void draw_trajectory(cv::Mat &image);
};