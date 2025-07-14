#ifndef RKNN_YOLO11_NODE_HPP_
#define RKNN_YOLO11_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "yolo11.h"
#include "postprocess.h"

class RknnYolo11Node : public rclcpp::Node
{
public:
    explicit RknnYolo11Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RknnYolo11Node();
    
    void initialize();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void initializeModel();
    void cleanupModel();
    vision_msgs::msg::Detection2DArray processImage(const cv::Mat& image);
    
    // ROS2 publishers and subscribers
    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    image_transport::Publisher debug_image_pub_;
    
    // RKNN context
    rknn_app_context_t rknn_app_ctx_;
    bool model_initialized_;
    
    // Parameters
    std::string model_path_;
    std::string input_topic_;
    std::string output_topic_;
    std::string debug_image_topic_;
    float confidence_threshold_;
    float nms_threshold_;
    bool enable_debug_image_;
    
    // Image transport
    std::shared_ptr<image_transport::ImageTransport> it_;
};

#endif // RKNN_YOLO11_NODE_HPP_