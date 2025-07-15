#include <rclcpp/rclcpp.hpp>
#include "integrated_person_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        rclcpp::NodeOptions options;
        auto node = std::make_shared<IntegratedPersonDetectionNode>(options);
        
        RCLCPP_INFO(node->get_logger(), "集成人员检测节点启动");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点运行时异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
} 