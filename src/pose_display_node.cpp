#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>

class PoseDisplayNode : public rclcpp::Node
{
public:
    PoseDisplayNode() : Node("pose_display_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("input_topic", "/yolov8_pose/image");
        this->declare_parameter<std::string>("window_name", "YOLOv8 Pose Detection");
        this->declare_parameter<bool>("enable_display", true);
        
        // 获取参数
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("window_name", window_name_);
        this->get_parameter("enable_display", enable_display_);
        
        if (enable_display_) {
            // 创建OpenCV窗口
            cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
            
            // 创建图像订阅者
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_topic, 10, 
                std::bind(&PoseDisplayNode::image_callback, this, std::placeholders::_1));
            
            RCLCPP_INFO(this->get_logger(), "Pose display node started");
            RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Window name: %s", window_name_.c_str());
            RCLCPP_INFO(this->get_logger(), "Press 'q' to quit");
        } else {
            RCLCPP_INFO(this->get_logger(), "Display disabled");
        }
    }
    
    ~PoseDisplayNode()
    {
        if (enable_display_) {
            cv::destroyWindow(window_name_);
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!enable_display_) return;
        
        try {
            // 转换ROS图像消息到OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // 显示图像
            cv::imshow(window_name_, cv_ptr->image);
            
            // 检查按键退出
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27) { // 'q' 或 ESC 键退出
                RCLCPP_INFO(this->get_logger(), "Quit key pressed, shutting down...");
                rclcpp::shutdown();
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::string window_name_;
    bool enable_display_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PoseDisplayNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 