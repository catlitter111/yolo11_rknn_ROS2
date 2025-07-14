#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

class ImageDisplayNode : public rclcpp::Node
{
public:
    ImageDisplayNode() : Node("image_display_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "/stereo/left/image_raw");
        this->declare_parameter("window_name", "Camera View");
        this->declare_parameter("enable_debug", true);
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        window_name_ = this->get_parameter("window_name").as_string();
        enable_debug_ = this->get_parameter("enable_debug").as_bool();
        
        // 创建订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageDisplayNode::imageCallback, this, std::placeholders::_1));
        
        // 如果启用调试模式，同时订阅YOLO调试图像
        if (enable_debug_) {
            debug_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/yolo_debug_image", 10,
                std::bind(&ImageDisplayNode::debugImageCallback, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(this->get_logger(), "图像显示节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "窗口名称: %s", window_name_.c_str());
        
        frame_count_ = 0;
        debug_frame_count_ = 0;
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
            
            frame_count_++;
            
            // 添加帧信息文本
            std::string frame_info = "Frame: " + std::to_string(frame_count_) + 
                                   " Size: " + std::to_string(msg->width) + "x" + std::to_string(msg->height);
            cv::putText(cv_ptr->image, frame_info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            
            // 显示图像
            cv::imshow(window_name_, cv_ptr->image);
            cv::waitKey(1);
            
            // 每30帧打印一次日志
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "显示帧 #%d, 尺寸: %dx%d", 
                           frame_count_, msg->width, msg->height);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void debugImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            debug_frame_count_++;
            
            // 添加调试信息文本
            std::string debug_info = "YOLO Debug Frame: " + std::to_string(debug_frame_count_);
            cv::putText(cv_ptr->image, debug_info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
            
            // 显示调试图像
            cv::imshow("YOLO11 Detection Results", cv_ptr->image);
            cv::waitKey(1);
            
            // 每10帧打印一次日志
            if (debug_frame_count_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "YOLO调试帧 #%d", debug_frame_count_);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "调试图像cv_bridge异常: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
    std::string window_name_;
    bool enable_debug_;
    int frame_count_;
    int debug_frame_count_;
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