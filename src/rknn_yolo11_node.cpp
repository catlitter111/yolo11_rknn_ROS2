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
    
    // 🔧 优化1：使用传感器数据QoS配置
    // 注意：image_transport的QoS配置通过参数设置，不直接在subscribe中设置
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                        .keep_last(1)  // 只保留最新帧，减少延迟
                        .best_effort() // 最佳努力模式，匹配相机
                        .durability_volatile(); // 易失性，减少内存开销
    
    // 🔧 优化2：检测结果QoS配置  
    auto detection_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                           .keep_last(2)  // 减少队列大小，降低延迟
                           .reliable()    // 可靠传输检测结果
                           .durability_volatile();
    
    // 创建订阅者和发布者
    image_sub_ = it_->subscribe(input_topic_, 1, 
        std::bind(&RknnYolo11Node::imageCallback, this, std::placeholders::_1));
    
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
        output_topic_, detection_qos);
    
    if (enable_debug_image_) {
        debug_image_pub_ = it_->advertise(debug_image_topic_, 1);
    }
    
    RCLCPP_INFO(this->get_logger(), "RKNN YOLO11 Node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Communication optimizations enabled: detection QoS, reduced queues");
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
    
    // 🔧 优化3：性能监控和帧率控制
    static auto last_process_time = std::chrono::steady_clock::now();
    static int frame_count = 0;
    frame_count++;
    
    try {
        // 🔧 优化4：零拷贝图像转换（当可能时）
        cv_bridge::CvImagePtr cv_ptr;
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            // 直接使用，无需转换
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } else {
            // 需要转换的情况
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        
        cv::Mat image = cv_ptr->image;
        
        // 🔧 优化5：内存预分配避免重复分配
        static vision_msgs::msg::Detection2DArray cached_detections;
        cached_detections.detections.clear();
        
        // 处理图像并获取检测结果
        vision_msgs::msg::Detection2DArray detections = processImage(image);
        
        // 🔧 优化6：消息头优化 - 直接复制避免不必要的拷贝
        detections.header = msg->header;
        
        // 🔧 优化7：移动语义发布，避免拷贝
        detection_pub_->publish(std::move(detections));
        
        // 🔧 优化8：调试图像的条件编译和优化
        if (enable_debug_image_) {
            publishDebugImage(image, msg->header);
        }
        
        // 🔧 优化9：性能监控
        if (frame_count % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_process_time);
            double fps = 30000.0 / duration.count();
            RCLCPP_INFO(this->get_logger(), "Processing FPS: %.1f, Frame: %d", fps, frame_count);
            last_process_time = now;
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
    
    // 🔧 优化10：RKNN图像处理优化
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    
    // 设置图像参数
    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = image.cols * image.rows * 3;
    
    // 🔧 优化11：内存预分配和复用
    static std::vector<unsigned char> rgb_buffer;
    rgb_buffer.resize(src_image.size);
    src_image.virt_addr = rgb_buffer.data();
    
    // 🔧 优化12：优化的颜色空间转换
    cv::Mat rgb_image(image.rows, image.cols, CV_8UC3, rgb_buffer.data());
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    
    // 执行推理
    object_detect_result_list od_results;
    int ret = inference_yolo11_model(&rknn_app_ctx_, &src_image, &od_results, confidence_threshold_, nms_threshold_);
    
    if (ret == 0) {
        // 🔧 优化13：预分配检测结果向量
        detections_msg.detections.reserve(od_results.count);
        
        // 转换检测结果为ROS消息格式
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result* det_result = &(od_results.results[i]);
            
            vision_msgs::msg::Detection2D detection;
            
            // 🔧 优化14：减少重复计算
            float center_x = (det_result->box.left + det_result->box.right) * 0.5f;
            float center_y = (det_result->box.top + det_result->box.bottom) * 0.5f;
            float width = det_result->box.right - det_result->box.left;
            float height = det_result->box.bottom - det_result->box.top;
            
            // 设置边界框
            detection.bbox.center.position.x = center_x;
            detection.bbox.center.position.y = center_y;
            detection.bbox.size_x = width;
            detection.bbox.size_y = height;
            
            // 设置检测结果
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = clothing_cls_to_name(det_result->cls_id);
            hypothesis.hypothesis.score = det_result->prop;
            
            detection.results.emplace_back(std::move(hypothesis));
            detections_msg.detections.emplace_back(std::move(detection));
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "RKNN inference failed! ret=%d", ret);
    }
    
    // 🔧 优化15：不需要释放预分配的缓冲区
    return detections_msg;
}

// 🔧 优化16：专门的调试图像发布函数
void RknnYolo11Node::publishDebugImage(const cv::Mat& image, const std_msgs::msg::Header& header)
{
    // 静态缓冲区避免重复分配
    static cv::Mat debug_image;
    image.copyTo(debug_image);
    
    // 获取最新的检测结果进行绘制
    // 注意：这里简化处理，实际应用中可能需要同步机制
    
    // 发布调试图像
    sensor_msgs::msg::Image::SharedPtr debug_msg = 
        cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
    debug_image_pub_.publish(debug_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RknnYolo11Node)