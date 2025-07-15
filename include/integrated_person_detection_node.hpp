#ifndef INTEGRATED_PERSON_DETECTION_NODE_HPP_
#define INTEGRATED_PERSON_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <rmw/qos_profiles.h>

#include "yolo11.h"
#include "yolov8_pose.h"
#include "postprocess_pose.h"
#include "common.h"
#include "image_utils.h"
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <chrono>

// 服装检测结果结构
struct ClothingDetection {
    cv::Rect bbox;
    float confidence;
    int class_id;
    std::string category;  // "upper" or "lower"
    std::string color;     // 主要颜色名称
    cv::Scalar color_rgb;  // RGB颜色值
    bool has_upper;
    bool has_lower;
    
    ClothingDetection() : color("unknown"), color_rgb(cv::Scalar(128, 128, 128)), has_upper(false), has_lower(false) {}
};

// 服装配对结构
struct ClothingPair {
    ClothingDetection upper;
    ClothingDetection lower;
    bool has_upper;
    bool has_lower;
    
    ClothingPair() : has_upper(false), has_lower(false) {}
};

// 人体信息结构
struct PersonInfo {
    std::string person_id;
    cv::Rect person_bbox;
    cv::Point2f center;
    ClothingPair clothing;
    float distance;
    bool valid_distance;
    std::chrono::steady_clock::time_point last_update;
    
    // 关键点信息 (17个COCO关键点，每个3个值：x, y, confidence)
    float keypoints[17][3];
    bool has_keypoints;
    
    PersonInfo() : distance(-1.0f), valid_distance(false), has_keypoints(false) {
        // 初始化关键点
        for (int i = 0; i < 17; i++) {
            keypoints[i][0] = 0.0f; // x
            keypoints[i][1] = 0.0f; // y 
            keypoints[i][2] = 0.0f; // confidence
        }
    }
};

// 距离查询请求
struct DistanceQuery {
    std::string person_id;
    cv::Point2f query_point;
    std::chrono::steady_clock::time_point request_time;
};

class IntegratedPersonDetectionNode : public rclcpp::Node
{
public:
    explicit IntegratedPersonDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~IntegratedPersonDetectionNode();
    
    void initialize();

private:
    // 回调函数
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void depthResultCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    
    // 核心处理函数
    std::vector<ClothingDetection> detectClothing(const cv::Mat& image);
    std::vector<ClothingPair> matchClothingItems(const std::vector<ClothingDetection>& detections);
    std::vector<PersonInfo> determinePersonPositions(const std::vector<ClothingPair>& pairs, const cv::Mat& image);
    
    // 工具函数
    bool isUpperClothing(int class_id);
    bool isLowerClothing(int class_id);
    float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    cv::Point2f calculateCenter(const cv::Rect& bbox);
    
    // 颜色检测函数
    void detectClothingColor(const cv::Mat& image, ClothingDetection& detection);
    std::string getColorName(const cv::Scalar& hsv_color);
    cv::Scalar getMainColor(const cv::Mat& roi);
    
    // 距离查询
    void queryDistance(const cv::Point2f& point, const std::string& person_id);
    
    // YOLOv8 Pose关键点检测
    bool initializePoseModel();
    void cleanupPoseModel();
    void detectPersonKeypoints(const cv::Mat& image, PersonInfo& person);
    cv::Mat extractPersonROI(const cv::Mat& image, const cv::Rect& person_bbox);
    
    // 关键点可视化
    void drawKeypoints(cv::Mat& image, const PersonInfo& person);
    void drawSkeleton(cv::Mat& image, const PersonInfo& person);
    
    // 可视化和发布
    cv::Mat publishVisualization(const cv::Mat& image, const std::vector<PersonInfo>& persons);
    void publishPersonPositions(const std::vector<PersonInfo>& persons);
    void displayDebugImage(const cv::Mat& image);
    
    // 模型初始化
    bool initializeModel();
    void cleanupModel();
    void initializeClothingCategories();  // 添加这个方法声明
    
    // ROS2组件
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_result_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;  // 添加初始化定时器
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr person_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr depth_query_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    
    // YOLO11模型相关
    rknn_app_context_t rknn_app_ctx_;
    bool models_initialized_;
    
    // YOLOv8 Pose模型相关
    rknn_app_context_t pose_rknn_app_ctx_;
    bool pose_models_initialized_;
    std::string pose_model_path_;
    float keypoint_confidence_threshold_;
    
    // 参数
    std::string model_path_;
    std::string input_topic_;
    std::string person_topic_;
    std::string distance_query_topic_;
    std::string distance_result_topic_;
    std::string debug_image_topic_;
    float confidence_threshold_;
    float nms_threshold_;
    bool enable_debug_display_;
    
    // 服装类别映射
    std::unordered_map<int, std::string> class_names_;
    std::vector<int> upper_clothing_classes_;
    std::vector<int> lower_clothing_classes_;
    
    // 距离查询管理
    std::vector<DistanceQuery> pending_queries_;
    std::unordered_map<std::string, PersonInfo> person_cache_;
    
    // 性能监控
    int frame_count_;
    std::chrono::steady_clock::time_point last_fps_time_;
    
    // 骨骼连接关系 (COCO格式)
    std::vector<std::pair<int, int>> skeleton_connections_;
    
    // OpenCV显示窗口
    bool display_enabled_;
    std::string window_name_;
};

#endif  // INTEGRATED_PERSON_DETECTION_NODE_HPP_ 