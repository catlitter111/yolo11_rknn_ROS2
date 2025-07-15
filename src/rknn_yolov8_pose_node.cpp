#include "rknn_yolov8_pose_node.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// 骨架连接定义 (COCO format)
static const std::vector<std::pair<int, int>> SKELETON_CONNECTIONS = {
    {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13},
    {6, 12}, {7, 13}, {6, 7}, {6, 8}, {7, 9},
    {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3},
    {2, 4}, {3, 5}, {4, 6}, {5, 7}
};

// 颜色定义
static const cv::Scalar COLORS[] = {
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(255, 255, 0),   // 青色
    cv::Scalar(255, 0, 255),   // 品红
    cv::Scalar(0, 255, 255),   // 黄色
    cv::Scalar(128, 0, 128),   // 紫色
    cv::Scalar(255, 165, 0),   // 橙色
    cv::Scalar(0, 128, 128),   // 青绿
    cv::Scalar(128, 128, 0),   // 橄榄绿
};

RknnYolov8PoseNode::RknnYolov8PoseNode() : Node("rknn_yolov8_pose_node"), frame_count_(0), total_time_(0.0)
{
    // 声明参数
    this->declare_parameter<std::string>("model_path", "./model/yolov8_pose.rknn");
    this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("output_topic", "/yolov8_pose/image");
    this->declare_parameter<std::string>("detection_topic", "/yolov8_pose/detections");
    // this->declare_parameter<std::string>("marker_topic", "/yolov8_pose/markers");  // 已移除
    this->declare_parameter<bool>("show_fps", true);
    this->declare_parameter<bool>("enable_visualization", true);
    this->declare_parameter<double>("keypoint_confidence_threshold", 0.3);
    this->declare_parameter<bool>("debug_keypoints", false);
    
    // 获取参数
    this->get_parameter("model_path", model_path_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("detection_topic", detection_topic_);
    // this->get_parameter("marker_topic", marker_topic_);  // 已移除
    this->get_parameter("show_fps", show_fps_);
    this->get_parameter("enable_visualization", enable_visualization_);
    this->get_parameter("keypoint_confidence_threshold", keypoint_confidence_threshold_);
    this->get_parameter("debug_keypoints", debug_keypoints_);
    
    // 初始化RKNN模型
    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
    
    // 初始化后处理
    if (init_pose_post_process() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init post process");
        return;
    }
    
    // 初始化YOLOv8 pose模型
    int ret = init_yolov8_pose_model(model_path_.c_str(), &rknn_app_ctx_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init yolov8 pose model! ret=%d model_path=%s", ret, model_path_.c_str());
        return;
    }
    
    // 初始化骨架连接
    skeleton_connections_ = SKELETON_CONNECTIONS;
    
    // 创建订阅者和发布者
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic_, 10, std::bind(&RknnYolov8PoseNode::image_callback, this, std::placeholders::_1));
    
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(detection_topic_, 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    
    last_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "YOLOv8 pose node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Detection topic: %s", detection_topic_.c_str());
}

RknnYolov8PoseNode::~RknnYolov8PoseNode()
{
    // 释放模型资源
    release_yolov8_pose_model(&rknn_app_ctx_);
    deinit_pose_post_process();
    
    RCLCPP_INFO(this->get_logger(), "YOLOv8 pose node destroyed");
}

void RknnYolov8PoseNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // 转换ROS图像消息到OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // 处理图像
        process_image(cv_ptr->image);
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void RknnYolov8PoseNode::process_image(const cv::Mat& image)
{
    auto start_time = std::chrono::steady_clock::now();
    
    // 创建image_buffer_t结构
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    
    // 设置图像数据
    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = image.rows * image.cols * 3;
    
    // 转换BGR到RGB
    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    src_image.virt_addr = rgb_image.data;
    
    // 执行推理
    pose_object_detect_result_list od_results;
    int ret = inference_yolov8_pose_model(&rknn_app_ctx_, &src_image, &od_results);
    
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "YOLOv8 pose inference failed! ret=%d", ret);
        return;
    }
    
    // 创建输出图像
    cv::Mat output_image = image.clone();
    
    // 创建检测结果消息
    vision_msgs::msg::Detection2DArray detection_msg;
    detection_msg.header.stamp = this->get_clock()->now();
    detection_msg.header.frame_id = "camera_frame";
    
    // 处理检测结果
    for (int i = 0; i < od_results.count; i++) {
        const pose_object_detect_result& result = od_results.results[i];
        
        // 创建检测对象消息
        vision_msgs::msg::Detection2D detection;
        detection.header.stamp = this->get_clock()->now();
        detection.header.frame_id = "camera_frame";
        
        // 设置边界框
        detection.bbox.center.position.x = (result.box.left + result.box.right) / 2.0;
        detection.bbox.center.position.y = (result.box.top + result.box.bottom) / 2.0;
        detection.bbox.size_x = result.box.right - result.box.left;
        detection.bbox.size_y = result.box.bottom - result.box.top;
        
        // 设置假设
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = std::to_string(result.cls_id);
        hypothesis.hypothesis.score = result.prop;
        detection.results.push_back(hypothesis);
        
        detection_msg.detections.push_back(detection);
        
        // 在图像上绘制检测结果
        if (enable_visualization_) {
            // 绘制边界框
            cv::rectangle(output_image, 
                         cv::Point(result.box.left, result.box.top),
                         cv::Point(result.box.right, result.box.bottom),
                         cv::Scalar(0, 255, 0), 2);
            
            // 绘制类别和置信度
            std::string text = std::string(coco_cls_to_name(result.cls_id)) + " " + 
                              std::to_string(static_cast<int>(result.prop * 100)) + "%";
            cv::putText(output_image, text, 
                       cv::Point(result.box.left, result.box.top - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            
            // 绘制关键点和骨架
            draw_keypoints(output_image, result);
            draw_skeleton(output_image, result);
        }
    }
    
    // 发布检测结果
    detection_pub_->publish(detection_msg);
    
    // 创建检测消息
    if (enable_visualization_) {
        create_detection_messages(od_results, detection_msg.header);
    }
    
    // 计算和显示FPS
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    frame_count_++;
    total_time_ += duration.count();
    
    if (show_fps_) {
        double avg_fps = 1000.0 / (total_time_ / frame_count_);
        double current_fps = 1000.0 / duration.count();
        
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps)) + 
                               " (Avg: " + std::to_string(static_cast<int>(avg_fps)) + ")";
        cv::putText(output_image, fps_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    
    // 发布带有检测结果的图像
    if (image_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", output_image).toImageMsg();
        output_msg->header.stamp = this->get_clock()->now();
        output_msg->header.frame_id = "camera_frame";
        image_pub_->publish(*output_msg);
    }
    
    // 打印检测结果
    if (od_results.count > 0) {
        RCLCPP_INFO(this->get_logger(), "Detected %d objects", od_results.count);
        for (int i = 0; i < od_results.count; i++) {
            const pose_object_detect_result& result = od_results.results[i];
            RCLCPP_INFO(this->get_logger(), "  %s @ (%d %d %d %d) %.3f",
                       coco_cls_to_name(result.cls_id),
                       result.box.left, result.box.top,
                       result.box.right, result.box.bottom,
                       result.prop);
        }
    }
}

void RknnYolov8PoseNode::draw_keypoints(cv::Mat& image, const pose_object_detect_result& result)
{
    // 绘制关键点
    for (int k = 0; k < 17; k++) {
        float x = result.keypoints[k][0];
        float y = result.keypoints[k][1];
        float conf = result.keypoints[k][2];
        
        // 调试输出关键点信息
        if (debug_keypoints_) {
            RCLCPP_INFO(this->get_logger(), "Keypoint %d: x=%.1f, y=%.1f, conf=%.3f", k, x, y, conf);
        }
        
        // 检查关键点是否在图像范围内且置信度足够
        if (conf > keypoint_confidence_threshold_ && x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            // 根据置信度调整颜色
            cv::Scalar color;
            if (conf > 0.7) {
                color = cv::Scalar(0, 255, 0);  // 绿色 - 高置信度
            } else if (conf > 0.5) {
                color = cv::Scalar(0, 255, 255); // 黄色 - 中等置信度
            } else {
                color = cv::Scalar(0, 165, 255); // 橙色 - 低置信度
            }
            
            cv::circle(image, cv::Point(static_cast<int>(x), static_cast<int>(y)), 
                      4, color, -1);
            
            // 绘制关键点编号
            cv::putText(image, std::to_string(k), 
                       cv::Point(static_cast<int>(x)+5, static_cast<int>(y)-5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
        }
    }
}

void RknnYolov8PoseNode::draw_skeleton(cv::Mat& image, const pose_object_detect_result& result)
{
    // 绘制骨架连接
    for (const auto& connection : skeleton_connections_) {
        int idx1 = connection.first - 1;  // 转换为0-based索引
        int idx2 = connection.second - 1;
        
        if (idx1 >= 0 && idx1 < 17 && idx2 >= 0 && idx2 < 17) {
            float x1 = result.keypoints[idx1][0];
            float y1 = result.keypoints[idx1][1];
            float conf1 = result.keypoints[idx1][2];
            
            float x2 = result.keypoints[idx2][0];
            float y2 = result.keypoints[idx2][1];
            float conf2 = result.keypoints[idx2][2];
            
            // 检查坐标是否在图像范围内
            bool valid1 = conf1 > keypoint_confidence_threshold_ && x1 >= 0 && x1 < image.cols && y1 >= 0 && y1 < image.rows;
            bool valid2 = conf2 > keypoint_confidence_threshold_ && x2 >= 0 && x2 < image.cols && y2 >= 0 && y2 < image.rows;
            
            // 只绘制两个关键点都有效的连接
            if (valid1 && valid2) {
                // 根据置信度调整线条颜色和粗细
                cv::Scalar color;
                int thickness;
                float avg_conf = (conf1 + conf2) / 2.0;
                
                if (avg_conf > 0.7) {
                    color = cv::Scalar(0, 255, 0);  // 绿色 - 高置信度
                    thickness = 3;
                } else if (avg_conf > 0.5) {
                    color = cv::Scalar(255, 165, 0); // 橙色 - 中等置信度
                    thickness = 2;
                } else {
                    color = cv::Scalar(0, 165, 255); // 橙色 - 低置信度
                    thickness = 1;
                }
                
                cv::line(image, 
                        cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                        cv::Point(static_cast<int>(x2), static_cast<int>(y2)),
                        color, thickness);
            }
        }
    }
}

void RknnYolov8PoseNode::create_detection_messages(const pose_object_detect_result_list& results, 
                                                   const std_msgs::msg::Header& header)
{
    // 这里可以添加更多的检测结果处理逻辑
    // 目前简化处理，仅打印关键点信息
    for (int i = 0; i < results.count; i++) {
        const pose_object_detect_result& result = results.results[i];
        
        std::string keypoint_info = "Keypoints: ";
        for (int k = 0; k < 17; k++) {
            if (result.keypoints[k][2] > 0.5) {  // 只显示置信度高的关键点
                keypoint_info += "[" + std::to_string(k) + ": " + 
                                std::to_string(result.keypoints[k][0]) + "," +
                                std::to_string(result.keypoints[k][1]) + "] ";
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Person %d: %s", i, keypoint_info.c_str());
    }
} 