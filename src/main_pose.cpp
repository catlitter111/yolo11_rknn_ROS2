#include <rclcpp/rclcpp.hpp>
#include "rknn_yolov8_pose_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RknnYolov8PoseNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting YOLOv8 pose detection node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 