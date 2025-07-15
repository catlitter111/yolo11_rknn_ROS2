#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <jsoncpp/json/json.h>
#include <cmath> // For std::sqrt and std::pow
#include <limits> // For std::numeric_limits

class ImageDisplayNode : public rclcpp::Node
{
public:
    ImageDisplayNode() : Node("image_display_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "/camera/color/image_raw");
        this->declare_parameter("detection_topic", "/detections");
        this->declare_parameter("distance_request_topic", "/depth_reader/get_depth_at");
        this->declare_parameter("distance_response_topic", "/depth_reader/depth_value");
        this->declare_parameter("window_name", "YOLO11 + Distance Detection");
        this->declare_parameter("enable_distance", true);
        this->declare_parameter("enable_debug", true);
        this->declare_parameter("font_scale", 0.7);
        this->declare_parameter("line_thickness", 2);
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string detection_topic = this->get_parameter("detection_topic").as_string();
        distance_request_topic_ = this->get_parameter("distance_request_topic").as_string();
        distance_response_topic_ = this->get_parameter("distance_response_topic").as_string();
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
        
        // 创建距离请求发布者和响应订阅者
        if (enable_distance_) {
            distance_request_pub_ = this->create_publisher<std_msgs::msg::String>(
                distance_request_topic_, 10);
            distance_response_sub_ = this->create_subscription<std_msgs::msg::String>(
                distance_response_topic_, 10,
                std::bind(&ImageDisplayNode::distanceResponseCallback, this, std::placeholders::_1));
        }
        
        // 创建定时器用于显示更新
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&ImageDisplayNode::updateDisplay, this));
        
        RCLCPP_INFO(this->get_logger(), "YOLO11距离检测显示节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅图像话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅检测话题: %s", detection_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "距离请求话题: %s", distance_request_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "距离响应话题: %s", distance_response_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "窗口名称: %s", window_name_.c_str());
        
        // 窗口创建标志
        window_created_ = false;
    }
    
    ~ImageDisplayNode()
    {
        cv::destroyAllWindows();
    }

private:
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
        if (enable_distance_) {
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
        // 清理过期的距离信息
        cleanupExpiredDistances();
        
        for (size_t i = 0; i < detections->detections.size(); ++i) {
            const auto& detection = detections->detections[i];
            
            // 计算检测框中心点
            double center_x = detection.bbox.center.position.x;
            double center_y = detection.bbox.center.position.y;
            
            // 检查是否已经有最近的距离信息
            if (hasRecentDistanceInfo(i, center_x, center_y)) {
                continue;
            }
            
            // 创建JSON格式的距离请求
            Json::Value request_json;
            request_json["x"] = static_cast<int>(center_x);
            request_json["y"] = static_cast<int>(center_y);
            request_json["detection_index"] = static_cast<int>(i);
            request_json["timestamp"] = this->get_clock()->now().nanoseconds();
            
            Json::StreamWriterBuilder builder;
            std::string json_string = Json::writeString(builder, request_json);
            
            // 发布距离请求
            auto msg = std_msgs::msg::String();
            msg.data = json_string;
            distance_request_pub_->publish(msg);
            
            RCLCPP_DEBUG(this->get_logger(), "请求检测目标 %zu 的距离，坐标: (%.0f, %.0f)", 
                        i, center_x, center_y);
        }
    }
    
    // 新增：检查是否有最近的距离信息
    bool hasRecentDistanceInfo(size_t detection_index, double center_x, double center_y) {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        
        auto it = detection_distances_.find(detection_index);
        if (it == detection_distances_.end()) {
            return false;
        }
        
        const auto& dist_info = it->second;
        auto now = this->get_clock()->now();
        auto age = (now - dist_info.timestamp).seconds();
        
        // 如果距离信息不到1秒，并且坐标相近，则认为不需要重新请求
        if (age < 1.0) {
            double coord_diff = std::sqrt(std::pow(center_x - dist_info.center_x, 2) + 
                                        std::pow(center_y - dist_info.center_y, 2));
            if (coord_diff < 20.0) { // 20像素内认为是同一个目标
                return true;
            }
        }
        
        return false;
    }
    
    // 新增：清理过期的距离信息
    void cleanupExpiredDistances() {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        
        auto now = this->get_clock()->now();
        auto it = detection_distances_.begin();
        
        while (it != detection_distances_.end()) {
            auto age = (now - it->second.timestamp).seconds();
            if (age > 10.0) { // 10秒后清理
                RCLCPP_DEBUG(this->get_logger(), "清理过期的距离信息，检测目标: %zu", it->first);
                it = detection_distances_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void distanceResponseCallback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        try {
            Json::Reader reader;
            Json::Value response_json;
            
            if (!reader.parse(msg->data, response_json)) {
                RCLCPP_WARN(this->get_logger(), "解析距离响应JSON失败");
                return;
            }
            
            std::lock_guard<std::mutex> lock(distance_mutex_);
            
            DetectionDistance dist_info;
            dist_info.center_x = response_json["x"].asDouble();
            dist_info.center_y = response_json["y"].asDouble();
            dist_info.timestamp = this->get_clock()->now();
            
            // 从响应中获取检测索引（如果有的话）
            size_t detection_index = 0;
            if (response_json.isMember("detection_index")) {
                detection_index = response_json["detection_index"].asUInt();
            } else {
                // 如果没有detection_index，通过坐标匹配最近的检测
                detection_index = findDetectionByCoordinate(dist_info.center_x, dist_info.center_y);
            }
            
            if (response_json["valid"].asBool()) {
                dist_info.distance = response_json["depth_m"].asDouble();
                dist_info.valid = true;
                dist_info.error_message = "";
                
                RCLCPP_INFO(this->get_logger(), "接收到检测目标 %zu 距离: %.3f m (坐标: %.0f, %.0f)", 
                           detection_index, dist_info.distance, dist_info.center_x, dist_info.center_y);
            } else {
                dist_info.distance = 0.0;
                dist_info.valid = false;
                dist_info.error_message = response_json.get("error", "Unknown error").asString();
                
                RCLCPP_WARN(this->get_logger(), "检测目标 %zu 距离获取失败: %s (坐标: %.0f, %.0f)", 
                           detection_index, dist_info.error_message.c_str(), dist_info.center_x, dist_info.center_y);
            }
            
            dist_info.detection_index = detection_index;
            detection_distances_[detection_index] = dist_info;
            
            // 调试信息：显示当前存储的距离信息数量
            RCLCPP_DEBUG(this->get_logger(), "当前存储的距离信息数量: %zu", detection_distances_.size());
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "距离响应处理异常: %s", e.what());
        }
    }
    
    // 新增：通过坐标查找最近的检测目标
    size_t findDetectionByCoordinate(double x, double y) {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        
        size_t best_index = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < current_detections_.detections.size(); ++i) {
            const auto& detection = current_detections_.detections[i];
            double center_x = detection.bbox.center.position.x;
            double center_y = detection.bbox.center.position.y;
            
            double dist = std::sqrt(std::pow(x - center_x, 2) + std::pow(y - center_y, 2));
            if (dist < min_distance) {
                min_distance = dist;
                best_index = i;
            }
        }
        
        return best_index;
    }

    void updateDisplay()
    {
        cv::Mat display_image;
        vision_msgs::msg::Detection2DArray detections;
        
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!current_image_valid_) {
                return;
            }
            display_image = current_image_.clone();
        }
        
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            detections = current_detections_;
        }
        
        // 绘制检测结果和距离信息
        for (size_t i = 0; i < detections.detections.size(); ++i) {
            const auto& detection = detections.detections[i];
            
            if (detection.results.empty()) continue;
            
            // 获取边界框信息
            auto bbox = detection.bbox;
            cv::Point2f center(bbox.center.position.x, bbox.center.position.y);
            cv::Size2f size(bbox.size_x, bbox.size_y);
            
            // 计算边界框坐标
            int x1 = static_cast<int>(center.x - size.width / 2);
            int y1 = static_cast<int>(center.y - size.height / 2);
            int x2 = static_cast<int>(center.x + size.width / 2);
            int y2 = static_cast<int>(center.y + size.height / 2);
            
            // 绘制边界框
            cv::rectangle(display_image, cv::Point(x1, y1), cv::Point(x2, y2), 
                        cv::Scalar(0, 255, 0), line_thickness_);
            
            // 绘制标签和置信度
            const auto& hypothesis = detection.results[0].hypothesis;
            std::string label = hypothesis.class_id + " " + 
                              std::to_string(static_cast<int>(hypothesis.score * 100)) + "%";
            
            // 添加距离信息
            std::string distance_text = getDistanceText(i);
            if (!distance_text.empty()) {
                label += " " + distance_text;
            }
            
            // 绘制标签背景
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                                font_scale_, line_thickness_, &baseline);
            cv::rectangle(display_image, 
                        cv::Point(x1, y1 - text_size.height - baseline),
                        cv::Point(x1 + text_size.width, y1),
                        cv::Scalar(0, 255, 0), -1);
            
            // 绘制标签文本
            cv::putText(display_image, label, cv::Point(x1, y1 - baseline),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale_, cv::Scalar(0, 0, 0), line_thickness_);
            
            // 绘制中心点
            cv::circle(display_image, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
                     3, cv::Scalar(0, 0, 255), -1);
        }
        
        // 绘制状态信息
        if (enable_debug_) {
            drawStatusInfo(display_image);
        }
        
        // 显示图像
        cv::imshow(window_name_, display_image);
        
        // 处理按键事件
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' 或 ESC
            RCLCPP_INFO(this->get_logger(), "用户请求退出");
            rclcpp::shutdown();
        }
        
        // 标记窗口已创建
        if (!window_created_) {
            window_created_ = true;
            RCLCPP_INFO(this->get_logger(), "显示窗口已创建，按 'q' 或 ESC 退出");
        }
    }
    
    void drawStatusInfo(cv::Mat& image)
    {
        int y_offset = 30;
        int line_height = 25;
        
        // 基本信息
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps_));
        cv::putText(image, fps_text, cv::Point(10, y_offset), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        y_offset += line_height;
        
        std::string frame_text = "Frame: " + std::to_string(frame_count_);
        cv::putText(image, frame_text, cv::Point(10, y_offset),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        y_offset += line_height;
        
        std::string detection_text = "Detections: " + std::to_string(detection_count_);
        cv::putText(image, detection_text, cv::Point(10, y_offset),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        
        // 距离服务状态
        if (enable_distance_) {
            y_offset += line_height;
            std::string distance_status = "Distance: Active";
            cv::putText(image, distance_status, cv::Point(10, y_offset),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            // 显示距离缓存数量
            std::lock_guard<std::mutex> lock(distance_mutex_);
            y_offset += line_height;
            std::string cache_text = "Distance Cache: " + std::to_string(detection_distances_.size());
            cv::putText(image, cache_text, cv::Point(10, y_offset),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }
    
    void updateFPS()
    {
        auto now = std::chrono::steady_clock::now();
        fps_timestamps_[frame_count_ % fps_buffer_size_] = now;
        
        if (frame_count_ >= fps_buffer_size_) {
            auto oldest = fps_timestamps_[(frame_count_ + 1) % fps_buffer_size_];
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldest);
            current_fps_ = (fps_buffer_size_ - 1) * 1000.0 / duration.count();
        }
    }
    
    std::string getDistanceText(size_t detection_index)
    {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        
        auto it = detection_distances_.find(detection_index);
        if (it == detection_distances_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "未找到检测目标 %zu 的距离信息", detection_index);
            return "";
        }
        
        const auto& dist_info = it->second;
        auto now = this->get_clock()->now();
        auto age = (now - dist_info.timestamp).seconds();
        
        // 增加时间窗口到5秒，减少数据过期问题
        if (age > 5.0) {
            RCLCPP_DEBUG(this->get_logger(), "检测目标 %zu 的距离信息过期 (%.1f秒)", detection_index, age);
            return "";
        }
        
        if (dist_info.valid) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << dist_info.distance << "m";
            RCLCPP_DEBUG(this->get_logger(), "显示检测目标 %zu 距离: %s", detection_index, oss.str().c_str());
            return oss.str();
        } else {
            RCLCPP_DEBUG(this->get_logger(), "检测目标 %zu 距离无效: %s", detection_index, dist_info.error_message.c_str());
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
    // ROS2订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr distance_request_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr distance_response_sub_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    
    // 配置参数
    std::string window_name_;
    std::string distance_request_topic_;
    std::string distance_response_topic_;
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