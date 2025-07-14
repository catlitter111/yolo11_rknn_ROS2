#include "rknn_yolo11_node.hpp"
#include "image_utils.h"
#include "image_drawing.h"
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

RknnYolo11Node::RknnYolo11Node(const rclcpp::NodeOptions & options)
: Node("rknn_yolo11_node", options), model_initialized_(false)
{
    // 声明参数
    this->declare_parameter("model_path", "/path/to/your/model.rknn");
    this->declare_parameter("input_topic", "/camera/image_raw");
    this->declare_parameter("output_topic", "/detections");
    this->declare_parameter("debug_image_topic", "/debug_image");
    this->declare_parameter("confidence_threshold", 0.25);
    this->declare_parameter("nms_threshold", 0.45);
    this->declare_parameter("enable_debug_image", true);
    
    // 获取参数
    this->get_parameter("model_path", model_path_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("debug_image_topic", debug_image_topic_);
    this->get_parameter("confidence_threshold", confidence_threshold_);
    this->get_parameter("nms_threshold", nms_threshold_);
    this->get_parameter("enable_debug_image", enable_debug_image_);
    
    RCLCPP_INFO(this->get_logger(), "RKNN YOLO11 Node initializing...");
    RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    
    // 初始化RKNN模型
    initializeModel();
    
    RCLCPP_INFO(this->get_logger(), "RKNN YOLO11 Node constructed successfully");
}

void RknnYolo11Node::initialize()
{
    // 创建image_transport
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    
    // 创建订阅者和发布者
    image_sub_ = it_->subscribe(input_topic_, 1, 
        std::bind(&RknnYolo11Node::imageCallback, this, std::placeholders::_1));
    
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
        output_topic_, 10);
    
    if (enable_debug_image_) {
        debug_image_pub_ = it_->advertise(debug_image_topic_, 1);
    }
    
    RCLCPP_INFO(this->get_logger(), "RKNN YOLO11 Node initialized successfully");
}

RknnYolo11Node::~RknnYolo11Node()
{
    cleanupModel();
}

void RknnYolo11Node::initializeModel()
{
    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
    
    // 初始化后处理
    init_post_process();
    
    // 初始化YOLO11模型
    int ret = init_yolo11_model(model_path_.c_str(), &rknn_app_ctx_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO11 model! ret=%d", ret);
        throw std::runtime_error("Model initialization failed");
    }
    
    model_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "YOLO11 model initialized successfully");
}

void RknnYolo11Node::cleanupModel()
{
    if (model_initialized_) {
        deinit_post_process();
        int ret = release_yolo11_model(&rknn_app_ctx_);
        if (ret != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to release YOLO11 model! ret=%d", ret);
        }
        model_initialized_ = false;
    }
}

void RknnYolo11Node::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!model_initialized_) {
        RCLCPP_WARN(this->get_logger(), "Model not initialized, skipping image");
        return;
    }
    
    try {
        // 转换ROS图像消息为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        // 处理图像并获取检测结果
        vision_msgs::msg::Detection2DArray detections = processImage(image);
        
        // 设置消息头
        detections.header = msg->header;
        
        // 发布检测结果
        detection_pub_->publish(detections);
        
        // 如果启用调试图像，绘制检测框并发布
        if (enable_debug_image_) {
            cv::Mat debug_image = image.clone();
            
            for (const auto& detection : detections.detections) {
                if (!detection.results.empty()) {
                    auto bbox = detection.bbox;
                    cv::Point2f center(bbox.center.position.x, bbox.center.position.y);
                    cv::Size2f size(bbox.size_x, bbox.size_y);
                    
                    // 计算边界框坐标
                    int x1 = static_cast<int>(center.x - size.width / 2);
                    int y1 = static_cast<int>(center.y - size.height / 2);
                    int x2 = static_cast<int>(center.x + size.width / 2);
                    int y2 = static_cast<int>(center.y + size.height / 2);
                    
                    // 绘制边界框
                    cv::rectangle(debug_image, cv::Point(x1, y1), cv::Point(x2, y2), 
                                cv::Scalar(0, 255, 0), 2);
                    
                    // 绘制标签和置信度
                    if (!detection.results.empty()) {
                        std::string label = detection.results[0].hypothesis.class_id + 
                                          " " + std::to_string(detection.results[0].hypothesis.score);
                        cv::putText(debug_image, label, cv::Point(x1, y1 - 10),
                                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            
            // 发布调试图像
            sensor_msgs::msg::Image::SharedPtr debug_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
            debug_image_pub_.publish(debug_msg);
        }
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    }
}

vision_msgs::msg::Detection2DArray RknnYolo11Node::processImage(const cv::Mat& image)
{
    vision_msgs::msg::Detection2DArray detections_msg;
    
    // 转换OpenCV图像为RKNN image_buffer_t格式
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    
    // 设置图像参数
    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = image.cols * image.rows * 3;
    
    // 分配内存并转换颜色空间 (BGR -> RGB)
    src_image.virt_addr = (unsigned char*)malloc(src_image.size);
    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    memcpy(src_image.virt_addr, rgb_image.data, src_image.size);
    
    // 执行推理
    object_detect_result_list od_results;
    int ret = inference_yolo11_model(&rknn_app_ctx_, &src_image, &od_results);
    
    if (ret == 0) {
        // 转换检测结果为ROS消息格式
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result* det_result = &(od_results.results[i]);
            
            if (det_result->prop >= confidence_threshold_) {
                vision_msgs::msg::Detection2D detection;
                
                // 设置边界框
                detection.bbox.center.position.x = (det_result->box.left + det_result->box.right) / 2.0;
                detection.bbox.center.position.y = (det_result->box.top + det_result->box.bottom) / 2.0;
                detection.bbox.size_x = det_result->box.right - det_result->box.left;
                detection.bbox.size_y = det_result->box.bottom - det_result->box.top;
                
                // 设置检测结果
                vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                hypothesis.hypothesis.class_id = std::to_string(det_result->cls_id);
                hypothesis.hypothesis.score = det_result->prop;
                
                detection.results.push_back(hypothesis);
                detections_msg.detections.push_back(detection);
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "RKNN inference failed! ret=%d", ret);
    }
    
    // 释放内存
    if (src_image.virt_addr != NULL) {
        free(src_image.virt_addr);
    }
    
    return detections_msg;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RknnYolo11Node)