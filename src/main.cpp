#include <rclcpp/rclcpp.hpp>
#include "rknn_yolo11_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RknnYolo11Node>();
        node->initialize();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}