#include <memory>

#include "data_processor/ImageDataProvider.hpp"
#include "data_processor/Recognizer.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto cam_mock_node = std::make_shared<CameraMockerNode>();
    auto text_recon_node = std::make_shared<DataProcessorNode>();

    exec.add_node(cam_mock_node);
    exec.add_node(text_recon_node);
    
    exec.spin();
    rclcpp::shutdown();
    return 0;
}