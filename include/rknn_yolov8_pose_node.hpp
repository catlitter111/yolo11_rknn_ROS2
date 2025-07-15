#ifndef RKNN_YOLOV8_POSE_NODE_HPP
#define RKNN_YOLOV8_POSE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <std_msgs/msg/header.hpp>

#include "yolov8_pose.h"
#include "common.h"
#include "image_utils.h"

class RknnYolov8PoseNode : public rclcpp::Node
{
public:
    RknnYolov8PoseNode();
    ~RknnYolov8PoseNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void process_image(const cv::Mat& image);
    void create_detection_messages(const object_detect_result_list& results, 
                                   const std_msgs::msg::Header& header);
    void draw_keypoints(cv::Mat& image, const object_detect_result& result);
    void draw_skeleton(cv::Mat& image, const object_detect_result& result);
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    // RKNN components
    rknn_app_context_t rknn_app_ctx_;
    
    // Parameters
    std::string model_path_;
    std::string input_topic_;
    std::string output_topic_;
    std::string detection_topic_;
    bool show_fps_;
    bool enable_visualization_;
    double keypoint_confidence_threshold_;
    bool debug_keypoints_;
    
    // Skeleton connections (COCO format)
    std::vector<std::pair<int, int>> skeleton_connections_;
    
    // Performance monitoring
    int frame_count_;
    double total_time_;
    std::chrono::steady_clock::time_point last_time_;
};

#endif // RKNN_YOLOV8_POSE_NODE_HPP 