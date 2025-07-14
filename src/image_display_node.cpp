#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "stereo_camera_cpp/srv/get_distance.hpp"
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>

class ImageDisplayNode : public rclcpp::Node
{
public:
    ImageDisplayNode() : Node("image_display_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "/stereo/left/image_raw");
        this->declare_parameter("detection_topic", "/detections");
        this->declare_parameter("distance_service", "/stereo/get_distance");
        this->declare_parameter("window_name", "YOLO11 + Distance Detection");
        this->declare_parameter("enable_distance", true);
        this->declare_parameter("enable_debug", true);
        this->declare_parameter("font_scale", 0.7);
        this->declare_parameter("line_thickness", 2);
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string detection_topic = this->get_parameter("detection_topic").as_string();
        distance_service_name_ = this->get_parameter("distance_service").as_string();
        window_name_ = this->get_parameter("window_name").as_string();
        enable_distance_ = this->get_parameter("enable_distance").as_bool();
        enable_debug_ = this->get_parameter("enable_debug").as_bool();
        font_scale_ = this->get_parameter("font_scale").as_double();
        line_thickness_ = this->get_parameter("line_thickness").as_int();
        
        // 初始化变量
        frame_count_ = 0;
        detection_count_ = 0;
        current_image_valid_ = false;
        current_fps_ = 0.0;
        fps_buffer_size_ = 30;
        fps_timestamps_.resize(fps_buffer_size_);
        
        // 创建图像订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageDisplayNode::imageCallback, this, std::placeholders::_1));
        
        // 创建检测结果订阅者
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            detection_topic, 10,
            std::bind(&ImageDisplayNode::detectionCallback, this, std::placeholders::_1));
        
        // 创建距离服务客户端
        if (enable_distance_) {
            distance_client_ = this->create_client<stereo_camera_cpp::srv::GetDistance>(
                distance_service_name_);
        }
        
        // 创建定时器用于显示更新
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&ImageDisplayNode::updateDisplay, this));
        
        RCLCPP_INFO(this->get_logger(), "YOLO11距离检测显示节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅图像话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅检测话题: %s", detection_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "距离服务: %s", distance_service_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "窗口名称: %s", window_name_.c_str());
        
        // 等待距离服务可用
        if (enable_distance_) {
            wait_for_service_thread_ = std::thread(&ImageDisplayNode::waitForDistanceService, this);
        }
        
        // 窗口创建标志
        window_created_ = false;
    }
    
    ~ImageDisplayNode()
    {
        cv::destroyAllWindows();
        if (wait_for_service_thread_.joinable()) {
            wait_for_service_thread_.join();
        }
    }

private:
    void waitForDistanceService()
    {
        RCLCPP_INFO(this->get_logger(), "等待距离服务可用...");
        
        int retry_count = 0;
        while (rclcpp::ok() && !distance_client_->service_is_ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retry_count++;
            
            if (retry_count % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "距离服务尚未可用，继续等待... (尝试 %d)", retry_count);
            }
            
            if (retry_count > 40) { // 20秒超时
                RCLCPP_WARN(this->get_logger(), "距离服务等待超时，距离功能将被禁用");
                distance_service_available_ = false;
                return;
            }
        }
        
        if (rclcpp::ok() && distance_client_->service_is_ready()) {
            distance_service_available_ = true;
            RCLCPP_INFO(this->get_logger(), "距离服务已连接");
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                current_image_ = cv_ptr->image.clone();
                current_image_header_ = msg->header;
                current_image_valid_ = true;
                frame_count_++;
                
                // 更新FPS计算
                updateFPS();
            }
            
            // 每30帧打印一次日志
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "接收图像帧 #%d, 尺寸: %dx%d", 
                           frame_count_, msg->width, msg->height);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void detectionCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            current_detections_ = *msg;
            detection_count_++;
        }
        
        // 如果启用距离检测，为每个检测结果请求距离
        if (enable_distance_ && distance_service_available_) {
            requestDistancesForDetections(msg);
        }
        
        // 每10次检测打印一次日志
        if (detection_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "接收检测结果 #%d, 检测到 %zu 个目标", 
                       detection_count_, msg->detections.size());
        }
    }
    
    void requestDistancesForDetections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr detections)
    {
        for (size_t i = 0; i < detections->detections.size(); ++i) {
            const auto& detection = detections->detections[i];
            
            // 计算检测框中心点
            double center_x = detection.bbox.center.position.x;
            double center_y = detection.bbox.center.position.y;
            
            // 创建距离请求
            auto request = std::make_shared<stereo_camera_cpp::srv::GetDistance::Request>();
            request->center_x = center_x;
            request->center_y = center_y;
            request->radius = 5; // 5像素半径
            
            // 异步调用距离服务
            distance_client_->async_send_request(
                request,
                [this, i, center_x, center_y](rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedFuture future) {
                    this->handleDistanceResponse(future, i, center_x, center_y);
                }
            );
        }
    }
    
    void handleDistanceResponse(
        rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedFuture future,
        size_t detection_index, double center_x, double center_y)
    {
        try {
            auto response = future.get();
            
            std::lock_guard<std::mutex> lock(distance_mutex_);
            
            DetectionDistance dist_info;
            dist_info.center_x = center_x;
            dist_info.center_y = center_y;
            dist_info.detection_index = detection_index;
            dist_info.timestamp = this->get_clock()->now();
            
            if (response->success) {
                dist_info.distance = response->distance;
                dist_info.valid = true;
                dist_info.error_message = "";
                
                RCLCPP_DEBUG(this->get_logger(), "检测目标 %zu 距离: %.3f m", 
                           detection_index, response->distance);
            } else {
                dist_info.distance = 0.0;
                dist_info.valid = false;
                dist_info.error_message = response->error_message;
                
                RCLCPP_DEBUG(this->get_logger(), "检测目标 %zu 距离获取失败: %s", 
                           detection_index, response->error_message.c_str());
            }
            
            detection_distances_[detection_index] = dist_info;
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "距离响应处理异常: %s", e.what());
        }
    }
    
    void updateDisplay()
    {
        cv::Mat display_image;
        vision_msgs::msg::Detection2DArray detections;
        bool image_valid = false;
        
        // 获取当前图像和检测结果
        {
            std::lock_guard<std::mutex> lock1(image_mutex_);
            std::lock_guard<std::mutex> lock2(detection_mutex_);
            
            if (current_image_valid_) {
                display_image = current_image_.clone();
                detections = current_detections_;
                image_valid = true;
            }
        }
        
        if (!image_valid) {
            return;
        }
        
        // 第一次创建窗口
        if (!window_created_) {
            cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
            cv::moveWindow(window_name_, 100, 100);
            window_created_ = true;
            RCLCPP_INFO(this->get_logger(), "创建显示窗口: %s", window_name_.c_str());
        }
        
        // 绘制检测结果
        drawDetections(display_image, detections);
        
        // 添加统计信息
        drawStatistics(display_image);
        
        // 显示图像
        cv::imshow(window_name_, display_image);
        
        // 处理按键事件
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') { // ESC或q键退出
            RCLCPP_INFO(this->get_logger(), "用户按下退出键，停止节点");
            rclcpp::shutdown();
        }
    }
    
    void drawDetections(cv::Mat& image, const vision_msgs::msg::Detection2DArray& detections)
    {
        for (size_t i = 0; i < detections.detections.size(); ++i) {
            const auto& detection = detections.detections[i];
            
            if (detection.results.empty()) {
                continue;
            }
            
            // 计算边界框坐标
            double center_x = detection.bbox.center.position.x;
            double center_y = detection.bbox.center.position.y;
            double width = detection.bbox.size_x;
            double height = detection.bbox.size_y;
            
            int x1 = static_cast<int>(center_x - width / 2);
            int y1 = static_cast<int>(center_y - height / 2);
            int x2 = static_cast<int>(center_x + width / 2);
            int y2 = static_cast<int>(center_y + height / 2);
            
            // 确保坐标在图像范围内
            x1 = std::max(0, std::min(x1, image.cols - 1));
            y1 = std::max(0, std::min(y1, image.rows - 1));
            x2 = std::max(0, std::min(x2, image.cols - 1));
            y2 = std::max(0, std::min(y2, image.rows - 1));
            
            // 获取检测结果信息
            const auto& result = detection.results[0];
            std::string class_id = result.hypothesis.class_id;
            double confidence = result.hypothesis.score;
            
            // 选择颜色（根据类别ID）
            cv::Scalar color = getColorForClass(class_id);
            
            // 绘制边界框
            cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, line_thickness_);
            
            // 绘制中心点
            cv::circle(image, cv::Point(static_cast<int>(center_x), static_cast<int>(center_y)), 
                      3, color, -1);
            
            // 准备标签文本
            std::ostringstream label_stream;
            label_stream << class_id << " " << std::fixed << std::setprecision(2) << confidence;
            
            // 添加距离信息
            std::string distance_text = getDistanceText(i);
            if (!distance_text.empty()) {
                label_stream << " | " << distance_text;
            }
            
            std::string label = label_stream.str();
            
            // 计算文本尺寸
            int baseline;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                               font_scale_, line_thickness_, &baseline);
            
            // 绘制文本背景
            cv::rectangle(image, 
                         cv::Point(x1, y1 - text_size.height - 10),
                         cv::Point(x1 + text_size.width, y1),
                         color, -1);
            
            // 绘制文本
            cv::putText(image, label, cv::Point(x1, y1 - 5), 
                       cv::FONT_HERSHEY_SIMPLEX, font_scale_, 
                       cv::Scalar(255, 255, 255), line_thickness_);
        }
    }
    
    void updateFPS()
    {
        auto now = std::chrono::steady_clock::now();
        
        // 更新时间戳缓冲区
        fps_timestamps_[frame_count_ % fps_buffer_size_] = now;
        
        // 计算FPS（当有足够的帧时）
        if (frame_count_ >= static_cast<uint64_t>(fps_buffer_size_)) {
            auto oldest_time = fps_timestamps_[(frame_count_ + 1) % fps_buffer_size_];
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldest_time);
            
            if (duration.count() > 0) {
                current_fps_ = (fps_buffer_size_ - 1) * 1000.0 / duration.count();
            }
        }
    }

    void drawStatistics(cv::Mat& image)
    {
        // 显示FPS信息
        std::string fps_info;
        if (frame_count_ >= static_cast<uint64_t>(fps_buffer_size_)) {
            fps_info = "FPS: " + std::to_string(static_cast<int>(current_fps_));
        } else {
            fps_info = "FPS: calculating...";
        }
        
        // 绘制统计信息
        cv::Scalar bg_color(0, 0, 0);      // 黑色背景
        cv::Scalar text_color(0, 255, 0);  // 绿色文字
        int margin = 10;
        int y = margin + 25;
        
        // 计算文本尺寸
        int baseline;
        cv::Size text_size = cv::getTextSize(fps_info, cv::FONT_HERSHEY_SIMPLEX, 
                                           font_scale_, 1, &baseline);
        
        // 绘制文本背景
        cv::rectangle(image, 
                     cv::Point(margin - 5, y - text_size.height - 5),
                     cv::Point(margin + text_size.width + 5, y + baseline + 5),
                     bg_color, -1);
        
        // 绘制文本
        cv::putText(image, fps_info, cv::Point(margin, y), 
                   cv::FONT_HERSHEY_SIMPLEX, font_scale_, text_color, 1);
    }
    
    cv::Scalar getColorForClass(const std::string& class_id)
    {
        // 根据类别ID生成颜色
        std::hash<std::string> hasher;
        size_t hash = hasher(class_id);
        
        // 生成鲜艳的颜色
        int r = (hash & 0xFF0000) >> 16;
        int g = (hash & 0x00FF00) >> 8;
        int b = hash & 0x0000FF;
        
        // 确保颜色足够亮
        r = std::max(r, 100);
        g = std::max(g, 100);
        b = std::max(b, 100);
        
        return cv::Scalar(b, g, r); // OpenCV使用BGR顺序
    }
    
    std::string getDistanceText(size_t detection_index)
    {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        
        auto it = detection_distances_.find(detection_index);
        if (it == detection_distances_.end()) {
            return "";
        }
        
        const auto& dist_info = it->second;
        auto now = this->get_clock()->now();
        auto age = (now - dist_info.timestamp).seconds();
        
        // 如果距离数据太旧（超过2秒），忽略
        if (age > 2.0) {
            return "";
        }
        
        if (dist_info.valid) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << dist_info.distance << "m";
            return oss.str();
        } else {
            return "N/A";
        }
    }
    
    // 距离信息结构
    struct DetectionDistance {
        double center_x;
        double center_y;
        size_t detection_index;
        double distance;
        bool valid;
        std::string error_message;
        rclcpp::Time timestamp;
    };
    
    // 成员变量
    // ROS2订阅者和客户端
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedPtr distance_client_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    
    // 配置参数
    std::string window_name_;
    std::string distance_service_name_;
    bool enable_distance_;
    bool enable_debug_;
    double font_scale_;
    int line_thickness_;
    
    // 状态变量
    std::mutex image_mutex_;
    std::mutex detection_mutex_;
    std::mutex distance_mutex_;
    
    cv::Mat current_image_;
    std_msgs::msg::Header current_image_header_;
    bool current_image_valid_;
    
    vision_msgs::msg::Detection2DArray current_detections_;
    std::map<size_t, DetectionDistance> detection_distances_;
    
    int frame_count_;
    int detection_count_;
    bool window_created_;
    
    // FPS计算相关
    double current_fps_;
    int fps_buffer_size_;
    std::vector<std::chrono::steady_clock::time_point> fps_timestamps_;
    
    // 距离服务状态
    std::atomic<bool> distance_service_available_{false};
    std::thread wait_for_service_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImageDisplayNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("image_display"), "异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}