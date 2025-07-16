#include "integrated_person_detection_node.hpp"
#include <algorithm>
#include <cmath>
#include <json/json.h>
#include <iomanip>
#include <sstream>

IntegratedPersonDetectionNode::IntegratedPersonDetectionNode(const rclcpp::NodeOptions & options)
: Node("integrated_person_detection_node", options), models_initialized_(false), pose_models_initialized_(false), current_mode_(DetectionMode::FULL_DETECTION), frame_count_(0), current_fps_(0.0)
{
    // 声明参数
    this->declare_parameter("model_path", "/userdata/rknn_yolo11_ros2/model/cloth.rknn");
    this->declare_parameter("pose_model_path", "/userdata/rknn_yolo11_ros2/model/yolov8_pose.rknn");
    this->declare_parameter("input_topic", "/camera/color/image_raw");
    this->declare_parameter("person_topic", "/person_detection/person_positions");
    this->declare_parameter("distance_query_topic", "/depth_reader/get_depth_at");
    this->declare_parameter("distance_result_topic", "/depth_reader/depth_value");
    this->declare_parameter("debug_image_topic", "/integrated_person/debug_image");
    this->declare_parameter("mode_topic", "/integrated_person/detection_mode");
    this->declare_parameter("confidence_threshold", 0.3);
    this->declare_parameter("nms_threshold", 0.5);
    this->declare_parameter("enable_debug_display", true);
    this->declare_parameter("keypoint_confidence_threshold", 0.3);
    
    // 获取参数
    this->get_parameter("model_path", model_path_);
    this->get_parameter("pose_model_path", pose_model_path_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("person_topic", person_topic_);
    this->get_parameter("distance_query_topic", distance_query_topic_);
    this->get_parameter("distance_result_topic", distance_result_topic_);
    this->get_parameter("debug_image_topic", debug_image_topic_);
    this->get_parameter("mode_topic", mode_topic_);
    this->get_parameter("confidence_threshold", confidence_threshold_);
    this->get_parameter("nms_threshold", nms_threshold_);
    this->get_parameter("enable_debug_display", enable_debug_display_);
    this->get_parameter("keypoint_confidence_threshold", keypoint_confidence_threshold_);
    
    // 初始化服装类别映射（基于YOLO11服装检测模型）
    initializeClothingCategories();
    
    // 初始化骨骼连接关系 (COCO格式)
    skeleton_connections_ = {
        {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13},
        {6, 12}, {7, 13}, {6, 7}, {6, 8}, {7, 9},
        {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3},
        {2, 4}, {3, 5}, {4, 6}, {5, 7}
    };
    
    // 初始化显示窗口
    window_name_ = "Integrated Person Detection";
    display_enabled_ = enable_debug_display_;
    
    RCLCPP_INFO(this->get_logger(), "集成人员检测节点初始化中...");
    RCLCPP_INFO(this->get_logger(), "模型路径: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "输入话题: %s", input_topic_.c_str());
    
    // 初始化模型
    if (!initializeModel()) {
        RCLCPP_ERROR(this->get_logger(), "服装检测模型初始化失败！");
        return;
    }
    
    // 初始化YOLOv8 Pose模型
    if (!initializePoseModel()) {
        RCLCPP_ERROR(this->get_logger(), "YOLOv8 Pose模型初始化失败！");
        return;
    }
    
    // 延迟初始化ROS组件
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            this->initialize();
            // 取消定时器
            init_timer_->cancel();
        }
    );
    
    last_fps_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "集成人员检测节点构建成功");
}

IntegratedPersonDetectionNode::~IntegratedPersonDetectionNode()
{
    if (display_enabled_) {
        cv::destroyAllWindows();
    }
    cleanupModel();
    cleanupPoseModel();
    RCLCPP_INFO(this->get_logger(), "集成人员检测节点销毁");
}

void IntegratedPersonDetectionNode::initialize()
{
    // 创建image_transport
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    
    // 优化的QoS配置
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                        .keep_last(1)
                        .best_effort()
                        .durability_volatile();
    
    auto reliable_qos = rclcpp::QoS(10).reliable().durability_volatile();
    
    // 订阅图像话题
    image_sub_ = it_->subscribe(input_topic_, 1,
        std::bind(&IntegratedPersonDetectionNode::imageCallback, this, std::placeholders::_1));
    
    // 订阅深度结果话题
    depth_result_sub_ = this->create_subscription<std_msgs::msg::String>(
        distance_result_topic_, reliable_qos,
        std::bind(&IntegratedPersonDetectionNode::depthResultCallback, this, std::placeholders::_1));
    
    // 订阅模式切换话题
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        mode_topic_, reliable_qos,
        std::bind(&IntegratedPersonDetectionNode::modeCallback, this, std::placeholders::_1));
    
    // 创建发布者
    person_pub_ = this->create_publisher<std_msgs::msg::String>(person_topic_, reliable_qos);
    depth_query_pub_ = this->create_publisher<std_msgs::msg::String>(distance_query_topic_, reliable_qos);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, sensor_qos);
    
    RCLCPP_INFO(this->get_logger(), "所有ROS2组件初始化完成");
}

void IntegratedPersonDetectionNode::initializeClothingCategories()
{
    // 基于13类服装检测模型的类别映射
    class_names_ = {
        {0, "short_sleeved_shirt"},     // 短袖衬衫
        {1, "long_sleeved_shirt"},      // 长袖衬衫
        {2, "short_sleeved_outwear"},   // 短袖外套
        {3, "long_sleeved_outwear"},    // 长袖外套
        {4, "vest"},                    // 背心
        {5, "sling"},                   // 吊带
        {6, "shorts"},                  // 短裤
        {7, "trousers"},                // 长裤
        {8, "skirt"},                   // 裙子
        {9, "short_sleeved_dress"},     // 短袖连衣裙
        {10, "long_sleeved_dress"},     // 长袖连衣裙
        {11, "vest_dress"},             // 背心裙
        {12, "sling_dress"}             // 吊带裙
    };
    
    // 上衣类别
    upper_clothing_classes_ = {0, 1, 2, 3, 4, 5}; // 0-5是上衣
    
    // 下装类别
    lower_clothing_classes_ = {6, 7, 8, 9, 10, 11, 12}; // 6-12是下装
}

bool IntegratedPersonDetectionNode::initializeModel()
{
    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
    
    // 初始化后处理
    if (init_post_process() != 0) {
        RCLCPP_ERROR(this->get_logger(), "初始化后处理失败");
        return false;
    }
    
    // 初始化YOLO11模型
    int ret = init_yolo11_model(model_path_.c_str(), &rknn_app_ctx_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "YOLO11模型初始化失败! ret=%d model_path=%s", 
                     ret, model_path_.c_str());
        return false;
    }
    
    models_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "YOLO11模型初始化成功");
    return true;
}

void IntegratedPersonDetectionNode::cleanupModel()
{
    if (models_initialized_) {
        release_yolo11_model(&rknn_app_ctx_);
        deinit_post_process();
        models_initialized_ = false;
    }
}

void IntegratedPersonDetectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!models_initialized_) {
        RCLCPP_WARN(this->get_logger(), "模型未初始化，跳过图像处理");
        return;
    }
    
    // 性能监控
    static auto last_process_time = std::chrono::steady_clock::now();
    frame_count_++;
    
    try {
        // 转换图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        // 1. 检测服装
        std::vector<ClothingDetection> detections = detectClothing(image);
        RCLCPP_DEBUG(this->get_logger(), "步骤1: 检测到 %zu 个服装项目", detections.size());
        
        // 2. 匹配服装项目
        std::vector<ClothingPair> pairs = matchClothingItems(detections);
        RCLCPP_DEBUG(this->get_logger(), "步骤2: 创建了 %zu 个服装配对", pairs.size());
        
        // 3. 确定人体位置
        std::vector<PersonInfo> persons = determinePersonPositions(pairs, image);
        RCLCPP_DEBUG(this->get_logger(), "步骤3: 确定了 %zu 个人员位置", persons.size());
        
        // 4. 检测关键点（仅在完整检测模式下）
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (current_mode_ == DetectionMode::FULL_DETECTION) {
                for (auto& person : persons) {
                    detectPersonKeypoints(image, person);
                }
                RCLCPP_DEBUG(this->get_logger(), "步骤4: 完成关键点检测");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "步骤4: 跳过关键点检测（部分检测模式）");
            }
        }
        
        // 5. 查询距离信息
        for (auto& person : persons) {
            queryDistance(person.center, person.person_id);
        }
        
        // 6. 发布结果
        publishPersonPositions(persons);
        cv::Mat vis_image = publishVisualization(image, persons);
        
        // 7. 显示调试图像
        if (display_enabled_) {
            displayDebugImage(vis_image);
        }
        
        // 性能监控
        if (frame_count_ % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_process_time);
            current_fps_ = 30000.0 / duration.count();
            RCLCPP_INFO(this->get_logger(), "处理FPS: %.1f, 帧数: %d, 检测到人数: %zu", 
                       current_fps_, frame_count_, persons.size());
            last_process_time = now;
        }
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    }
}

std::vector<ClothingDetection> IntegratedPersonDetectionNode::detectClothing(const cv::Mat& image)
{
    std::vector<ClothingDetection> detections;
    
    // 准备图像数据
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    
    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = image.cols * image.rows * 3;
    
    // 转换BGR到RGB
    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    src_image.virt_addr = rgb_image.data;
    
    // 执行推理
    object_detect_result_list od_results;
    int ret = inference_yolo11_model(&rknn_app_ctx_, &src_image, &od_results, 
                                    confidence_threshold_, nms_threshold_);
    
    if (ret == 0) {
        RCLCPP_DEBUG(this->get_logger(), "YOLO11检测到 %d 个物体", od_results.count);
        
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result* det_result = &(od_results.results[i]);
            
            ClothingDetection detection;
            detection.bbox = cv::Rect(det_result->box.left, det_result->box.top,
                                    det_result->box.right - det_result->box.left,
                                    det_result->box.bottom - det_result->box.top);
            detection.confidence = det_result->prop;
            detection.class_id = det_result->cls_id;
            
            // 确定类别 - 修复逻辑错误
            if (isUpperClothing(det_result->cls_id)) {
                detection.category = "upper";
            } else if (isLowerClothing(det_result->cls_id)) {
                detection.category = "lower";
            } else {
                continue; // 跳过未知类别
            }
            
            // 这些字段在ClothingDetection中不需要设置，它们属于ClothingPair
            detection.has_upper = false;
            detection.has_lower = false;
            
            // 检测服装颜色
            detectClothingColor(image, detection);
            
            detections.push_back(detection);
            
            RCLCPP_DEBUG(this->get_logger(), "检测到服装: 类别=%s, 置信度=%.2f, 位置=(%d,%d,%d,%d)", 
                        detection.category.c_str(), detection.confidence,
                        detection.bbox.x, detection.bbox.y, 
                        detection.bbox.width, detection.bbox.height);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "YOLO11推理失败, ret=%d", ret);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "总共检测到 %zu 个有效服装项目", detections.size());
    return detections;
}

std::vector<ClothingPair> IntegratedPersonDetectionNode::matchClothingItems(
    const std::vector<ClothingDetection>& detections)
{
    std::vector<ClothingPair> pairs;
    
    // 分离上衣和下装
    std::vector<ClothingDetection> upper_items;
    std::vector<ClothingDetection> lower_items;
    
    for (const auto& detection : detections) {
        if (detection.category == "upper") {
            upper_items.push_back(detection);
        } else if (detection.category == "lower") {
            lower_items.push_back(detection);
        }
    }
    
    // 匹配逻辑（基于Python版本的C++实现）
    std::vector<bool> lower_matched(lower_items.size(), false);
    
    // 为每件上衣寻找最近的下装
    for (const auto& upper : upper_items) {
        ClothingPair pair;
        pair.upper = upper;
        pair.has_upper = true;
        
        cv::Point2f upper_center = calculateCenter(upper.bbox);
        
        float min_distance = std::numeric_limits<float>::max();
        int best_lower_idx = -1;
        
        for (size_t j = 0; j < lower_items.size(); j++) {
            if (lower_matched[j]) continue;
            
            cv::Point2f lower_center = calculateCenter(lower_items[j].bbox);
            
            // 检查水平距离约束
            float x_distance = std::abs(upper_center.x - lower_center.x);
            float max_x_distance = upper.bbox.width * 0.5f; // 相对于上衣宽度的约束
            
            if (x_distance > max_x_distance) continue;
            
            float distance = calculateDistance(upper_center, lower_center);
            
            if (distance < min_distance) {
                min_distance = distance;
                best_lower_idx = j;
            }
        }
        
        if (best_lower_idx >= 0) {
            pair.lower = lower_items[best_lower_idx];
            pair.has_lower = true;
            lower_matched[best_lower_idx] = true;
        }
        
        pairs.push_back(pair);
    }
    
    // 添加未匹配的下装
    for (size_t i = 0; i < lower_items.size(); i++) {
        if (!lower_matched[i]) {
            ClothingPair pair;
            pair.lower = lower_items[i];
            pair.has_lower = true;
            pairs.push_back(pair);
        }
    }
    
    return pairs;
}

std::vector<PersonInfo> IntegratedPersonDetectionNode::determinePersonPositions(
    const std::vector<ClothingPair>& pairs, const cv::Mat& image)
{
    std::vector<PersonInfo> persons;
    
    RCLCPP_DEBUG(this->get_logger(), "开始确定人员位置，输入配对数: %zu", pairs.size());
    
    for (size_t i = 0; i < pairs.size(); i++) {
        const ClothingPair& pair = pairs[i];
        
        RCLCPP_DEBUG(this->get_logger(), "处理配对 %zu: has_upper=%s, has_lower=%s", 
                    i, pair.has_upper ? "true" : "false", pair.has_lower ? "true" : "false");
        
        // 确定人体边界框
        cv::Rect person_bbox;
        
        if (pair.has_upper && pair.has_lower) {
            // 两者都有，合并边界框
            person_bbox = pair.upper.bbox | pair.lower.bbox;
            RCLCPP_DEBUG(this->get_logger(), "合并上下装边界框: (%d,%d,%d,%d)", 
                        person_bbox.x, person_bbox.y, person_bbox.width, person_bbox.height);
            
            // 添加头部和脚部扩展
            int upper_height = pair.upper.bbox.height;
            int lower_height = pair.lower.bbox.height;
            int upper_width = pair.upper.bbox.width;
            
            int head_extension = static_cast<int>(upper_height * 0.5);
            int foot_extension = static_cast<int>(lower_height * 0.6);
            int side_extension = static_cast<int>(upper_width * 0.1);
            
            person_bbox.x = std::max(0, person_bbox.x - side_extension);
            person_bbox.y = std::max(0, person_bbox.y - head_extension);
            person_bbox.width = std::min(image.cols - person_bbox.x, 
                                       person_bbox.width + 2 * side_extension);
            person_bbox.height = std::min(image.rows - person_bbox.y, 
                                        person_bbox.height + head_extension + foot_extension);
            
        } else if (pair.has_upper) {
            // 只有上衣
            person_bbox = pair.upper.bbox;
            RCLCPP_DEBUG(this->get_logger(), "仅上衣边界框: (%d,%d,%d,%d)", 
                        person_bbox.x, person_bbox.y, person_bbox.width, person_bbox.height);
            
            int upper_height = pair.upper.bbox.height;
            
            person_bbox.x = std::max(0, person_bbox.x - 40);
            person_bbox.y = std::max(0, person_bbox.y - static_cast<int>(upper_height * 0.4));
            person_bbox.width = std::min(image.cols - person_bbox.x, person_bbox.width + 80);
            person_bbox.height = std::min(image.rows - person_bbox.y, 
                                        static_cast<int>(upper_height * 3.9));
            
        } else if (pair.has_lower) {
            // 只有下装
            person_bbox = pair.lower.bbox;
            RCLCPP_DEBUG(this->get_logger(), "仅下装边界框: (%d,%d,%d,%d)", 
                        person_bbox.x, person_bbox.y, person_bbox.width, person_bbox.height);
            
            int lower_height = pair.lower.bbox.height;
            
            person_bbox.x = std::max(0, person_bbox.x - 40);
            person_bbox.y = std::max(0, person_bbox.y - static_cast<int>(lower_height * 1.8));
            person_bbox.width = std::min(image.cols - person_bbox.x, person_bbox.width + 80);
            person_bbox.height = std::min(image.rows - person_bbox.y, 
                                        static_cast<int>(lower_height * 2.5));
        } else {
            RCLCPP_DEBUG(this->get_logger(), "跳过无效配对 %zu", i);
            continue; // 无效的配对
        }
        
        PersonInfo person;
        person.person_id = "Person_" + std::to_string(i);
        person.person_bbox = person_bbox;
        person.center = calculateCenter(person_bbox);
        person.clothing = pair;
        person.last_update = std::chrono::steady_clock::now();
        
        // 从缓存中获取距离信息（如果存在）
        if (person_cache_.find(person.person_id) != person_cache_.end()) {
            const auto& cached_person = person_cache_[person.person_id];
            person.distance = cached_person.distance;
            person.valid_distance = cached_person.valid_distance;
            RCLCPP_DEBUG(this->get_logger(), "从缓存中恢复人员 %s 的距离信息: %.2fm (valid: %s)", 
                        person.person_id.c_str(), person.distance, 
                        person.valid_distance ? "true" : "false");
        }
        
        RCLCPP_DEBUG(this->get_logger(), "创建人员 %s: 边界框=(%d,%d,%d,%d), 中心=(%.1f,%.1f)", 
                    person.person_id.c_str(), 
                    person_bbox.x, person_bbox.y, person_bbox.width, person_bbox.height,
                    person.center.x, person.center.y);
        
        persons.push_back(person);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "总共创建了 %zu 个人员", persons.size());
    return persons;
}

// 工具函数实现
bool IntegratedPersonDetectionNode::isUpperClothing(int class_id)
{
    return std::find(upper_clothing_classes_.begin(), upper_clothing_classes_.end(), class_id) 
           != upper_clothing_classes_.end();
}

bool IntegratedPersonDetectionNode::isLowerClothing(int class_id)
{
    return std::find(lower_clothing_classes_.begin(), lower_clothing_classes_.end(), class_id) 
           != lower_clothing_classes_.end();
}

float IntegratedPersonDetectionNode::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2)
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

cv::Point2f IntegratedPersonDetectionNode::calculateCenter(const cv::Rect& bbox)
{
    return cv::Point2f(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
}

void IntegratedPersonDetectionNode::queryDistance(const cv::Point2f& point, const std::string& person_id)
{
    try {
        // 创建距离查询
        DistanceQuery query;
        query.person_id = person_id;
        query.query_point = point;
        query.request_time = std::chrono::steady_clock::now();
        
        pending_queries_.push_back(query);
        
        // 发送查询请求
        Json::Value json_query;
        json_query["x"] = static_cast<int>(point.x);
        json_query["y"] = static_cast<int>(point.y);
        
        Json::StreamWriterBuilder builder;
        std::string query_str = Json::writeString(builder, json_query);
        
        std_msgs::msg::String query_msg;
        query_msg.data = query_str;
        depth_query_pub_->publish(query_msg);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "查询距离时出错: %s", e.what());
    }
}

void IntegratedPersonDetectionNode::depthResultCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    try {
        Json::Value result;
        Json::Reader reader;
        
        if (!reader.parse(msg->data, result)) {
            RCLCPP_ERROR(this->get_logger(), "解析深度结果JSON失败: %s", msg->data.c_str());
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "收到深度响应: %s", msg->data.c_str());
        
        int x = result["x"].asInt();
        int y = result["y"].asInt();
        bool valid = result.get("valid", true).asBool(); // 默认为true
        float depth_m = result.get("depth_m", -1.0f).asFloat();
        
        RCLCPP_INFO(this->get_logger(), "解析深度数据: x=%d, y=%d, valid=%s, depth=%.3fm", 
                   x, y, valid ? "true" : "false", depth_m);
        
        // 查找对应的查询请求
        bool found_match = false;
        for (auto it = pending_queries_.begin(); it != pending_queries_.end(); ++it) {
            if (std::abs(it->query_point.x - x) < 5.0f && std::abs(it->query_point.y - y) < 5.0f) {
                RCLCPP_INFO(this->get_logger(), "找到匹配的查询请求: %s, 坐标差异: (%.1f, %.1f)", 
                           it->person_id.c_str(), 
                           std::abs(it->query_point.x - x), 
                           std::abs(it->query_point.y - y));
                
                // 更新人员缓存
                if (person_cache_.find(it->person_id) != person_cache_.end()) {
                    person_cache_[it->person_id].distance = depth_m;
                    person_cache_[it->person_id].valid_distance = valid && (depth_m > 0);
                    RCLCPP_INFO(this->get_logger(), "更新人员 %s 距离缓存: %.3fm (valid: %s)", 
                               it->person_id.c_str(), depth_m, 
                               (valid && depth_m > 0) ? "true" : "false");
                } else {
                    // 如果缓存中没有，创建新的缓存项
                    PersonInfo cached_person;
                    cached_person.person_id = it->person_id;
                    cached_person.distance = depth_m;
                    cached_person.valid_distance = valid && (depth_m > 0);
                    cached_person.last_update = std::chrono::steady_clock::now();
                    person_cache_[it->person_id] = cached_person;
                    RCLCPP_INFO(this->get_logger(), "创建新的人员 %s 距离缓存: %.3fm (valid: %s)", 
                               it->person_id.c_str(), depth_m, 
                               (valid && depth_m > 0) ? "true" : "false");
                }
                
                // 移除已处理的查询
                pending_queries_.erase(it);
                found_match = true;
                break;
            }
        }
        
        if (!found_match) {
            RCLCPP_WARN(this->get_logger(), "未找到匹配的查询请求，坐标: (%d, %d), 待处理查询数: %zu", 
                       x, y, pending_queries_.size());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理深度结果时出错: %s", e.what());
    }
}

cv::Mat IntegratedPersonDetectionNode::publishVisualization(const cv::Mat& image, 
                                                        const std::vector<PersonInfo>& persons)
{
    try {
        cv::Mat vis_image = image.clone();
        
        // 绘制检测结果
        for (const auto& person : persons) {
            // 绘制人体边界框
            cv::rectangle(vis_image, person.person_bbox, cv::Scalar(0, 255, 0), 2);
            
            // 绘制人员ID
            cv::putText(vis_image, person.person_id, 
                       cv::Point(person.person_bbox.x, person.person_bbox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // 绘制上衣边界框和颜色信息
            if (person.clothing.has_upper) {
                cv::rectangle(vis_image, person.clothing.upper.bbox, cv::Scalar(255, 0, 0), 2);
                
                // 显示颜色信息（在边界框内）
                std::string upper_color = "upper:" + person.clothing.upper.color;
                cv::Point color_pos(person.clothing.upper.bbox.x + 5, 
                                   person.clothing.upper.bbox.y + 20);
                
                // 绘制颜色背景
                int baseline = 0;
                cv::Size color_text_size = cv::getTextSize(upper_color, cv::FONT_HERSHEY_SIMPLEX, 
                                                         0.6, 2, &baseline);
                cv::rectangle(vis_image, 
                            cv::Point(color_pos.x - 2, color_pos.y - color_text_size.height - 2),
                            cv::Point(color_pos.x + color_text_size.width + 2, color_pos.y + baseline + 2),
                            cv::Scalar(0, 0, 0), -1); // 黑色背景
                
                // 绘制颜色文字
                cv::putText(vis_image, upper_color, color_pos,
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            }
            
            // 绘制下装边界框和颜色信息
            if (person.clothing.has_lower) {
                cv::rectangle(vis_image, person.clothing.lower.bbox, cv::Scalar(0, 0, 255), 2);
                
                // 显示颜色信息（在边界框内）
                std::string lower_color = "lower:" + person.clothing.lower.color;
                cv::Point color_pos(person.clothing.lower.bbox.x + 5, 
                                   person.clothing.lower.bbox.y + 20);
                
                // 绘制颜色背景
                int baseline = 0;
                cv::Size color_text_size = cv::getTextSize(lower_color, cv::FONT_HERSHEY_SIMPLEX, 
                                                         0.6, 2, &baseline);
                cv::rectangle(vis_image, 
                            cv::Point(color_pos.x - 2, color_pos.y - color_text_size.height - 2),
                            cv::Point(color_pos.x + color_text_size.width + 2, color_pos.y + baseline + 2),
                            cv::Scalar(0, 0, 0), -1); // 黑色背景
                
                // 绘制颜色文字
                cv::putText(vis_image, lower_color, color_pos,
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            }
            
            // 显示距离信息
            if (person.valid_distance) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << person.distance << "m";
                
                // 计算文字位置 - 在人体中心点附近
                cv::Point text_pos(static_cast<int>(person.center.x - 40), 
                                 static_cast<int>(person.center.y + 30));
                
                // 绘制黑色背景矩形让文字更清晰
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, 
                                                   1.2, 3, &baseline);
                // cv::rectangle(vis_image, 
                //             cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                //             cv::Point(text_pos.x + text_size.width + 5, text_pos.y + baseline + 5),
                //             cv::Scalar(0, 0, 0), -1); // 黑色背景
                
                // // 绘制距离文字 - 更大字体，白色，更厚
                // cv::putText(vis_image, ss.str(), text_pos,
                //            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 3);
            }
            
            // 标记中心点
            cv::circle(vis_image, cv::Point(static_cast<int>(person.center.x), 
                                           static_cast<int>(person.center.y)), 5, cv::Scalar(255, 0, 0), -1);
            
            // 绘制关键点和骨架
            if (person.has_keypoints) {
                drawKeypoints(vis_image, person);
                drawSkeleton(vis_image, person);
            }
        }
        
        // 添加统计信息
        std::string mode_str;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode_str = (current_mode_ == DetectionMode::FULL_DETECTION) ? "FULL" : "PARTIAL";
        }
        
        std::string stats = "Persons: " + std::to_string(persons.size()) + 
                           ", FPS: " + std::to_string(static_cast<int>(current_fps_)) +
                           ", Mode: " + mode_str;
        cv::putText(vis_image, stats, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 发布调试图像
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", vis_image).toImageMsg();
        debug_image_pub_->publish(*img_msg);
        
        return vis_image; // 返回可视化图像
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "发布可视化图像时出错: %s", e.what());
        return cv::Mat(); // 返回空图像
    }
}

void IntegratedPersonDetectionNode::publishPersonPositions(const std::vector<PersonInfo>& persons)
{
    try {
        Json::Value positions_data;
        positions_data["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        positions_data["person_count"] = static_cast<int>(persons.size());
        
        // 添加检测模式信息
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            positions_data["detection_mode"] = (current_mode_ == DetectionMode::FULL_DETECTION) ? "FULL" : "PARTIAL";
        }
        
        Json::Value persons_array(Json::arrayValue);
        
        for (const auto& person : persons) {
            Json::Value person_data;
            person_data["id"] = person.person_id;
            
            Json::Value bbox_array(Json::arrayValue);
            bbox_array.append(person.person_bbox.x);
            bbox_array.append(person.person_bbox.y);
            bbox_array.append(person.person_bbox.x + person.person_bbox.width);
            bbox_array.append(person.person_bbox.y + person.person_bbox.height);
            person_data["bbox"] = bbox_array;
            
            Json::Value center_array(Json::arrayValue);
            center_array.append(static_cast<int>(person.center.x));
            center_array.append(static_cast<int>(person.center.y));
            person_data["center"] = center_array;
            
            person_data["distance"] = person.valid_distance ? person.distance : Json::Value::null;
            person_data["valid_distance"] = person.valid_distance;
            
            // 添加服装颜色信息
            Json::Value clothing_data;
            if (person.clothing.has_upper) {
                Json::Value upper_data;
                upper_data["color"] = person.clothing.upper.color;
                upper_data["confidence"] = person.clothing.upper.confidence;
                clothing_data["upper"] = upper_data;
            }
            if (person.clothing.has_lower) {
                Json::Value lower_data;
                lower_data["color"] = person.clothing.lower.color;
                lower_data["confidence"] = person.clothing.lower.confidence;
                clothing_data["lower"] = lower_data;
            }
            person_data["clothing"] = clothing_data;
            
            // 添加关键点信息
            if (person.has_keypoints) {
                Json::Value keypoints_array(Json::arrayValue);
                for (int i = 0; i < 17; i++) {
                    Json::Value keypoint_data(Json::arrayValue);
                    keypoint_data.append(person.keypoints[i][0]); // x
                    keypoint_data.append(person.keypoints[i][1]); // y
                    keypoint_data.append(person.keypoints[i][2]); // confidence
                    keypoints_array.append(keypoint_data);
                }
                person_data["keypoints"] = keypoints_array;
                person_data["has_keypoints"] = true;
            } else {
                person_data["has_keypoints"] = false;
            }
            
            // 添加身体比例信息
            if (person.has_body_ratios) {
                Json::Value ratios_array(Json::arrayValue);
                for (int i = 0; i < 16; i++) {
                    ratios_array.append(person.body_ratios[i]);
                }
                person_data["body_ratios"] = ratios_array;
                person_data["has_body_ratios"] = true;
                
                // 添加身体比例标签说明
                Json::Value ratio_labels(Json::arrayValue);
                ratio_labels.append("上肢与下肢比例");
                ratio_labels.append("躯干与身高比例");
                ratio_labels.append("肩宽与身高比例");
                ratio_labels.append("臀宽与肩宽比例");
                ratio_labels.append("头部与躯干比例");
                ratio_labels.append("手臂与身高比例");
                ratio_labels.append("腿长与身高比例");
                ratio_labels.append("上臂与下臂比例");
                ratio_labels.append("大腿与小腿比例");
                ratio_labels.append("躯干与腿长比例");
                ratio_labels.append("手臂与腿长比例");
                ratio_labels.append("肩宽与髋宽比例");
                ratio_labels.append("头围与身高比例");
                ratio_labels.append("脚长与身高比例");
                ratio_labels.append("脚踝宽与身高比例");
                ratio_labels.append("腰围与身高比例");
                person_data["ratio_labels"] = ratio_labels;
            } else {
                person_data["has_body_ratios"] = false;
            }
            
            persons_array.append(person_data);
        }
        
        positions_data["persons"] = persons_array;
        
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";  // 紧凑格式
        std::string json_str = Json::writeString(builder, positions_data);
        
        std_msgs::msg::String pos_msg;
        pos_msg.data = json_str;
        person_pub_->publish(pos_msg);
        
        // 更新人员缓存
        for (const auto& person : persons) {
            person_cache_[person.person_id] = person;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "发布人员位置时出错: %s", e.what());
    }
}

void IntegratedPersonDetectionNode::displayDebugImage(const cv::Mat& image)
{
    if (!display_enabled_) return;
    
    try {
        cv::imshow(window_name_, image);
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' 或 ESC
            RCLCPP_INFO(this->get_logger(), "用户按下退出键");
            rclcpp::shutdown();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "显示调试图像时出错: %s", e.what());
    }
}

// 颜色检测函数实现
void IntegratedPersonDetectionNode::detectClothingColor(const cv::Mat& image, ClothingDetection& detection)
{
    try {
        // 确保边界框在图像范围内
        cv::Rect safe_bbox = detection.bbox & cv::Rect(0, 0, image.cols, image.rows);
        if (safe_bbox.width <= 0 || safe_bbox.height <= 0) {
            detection.color = "unknown";
            detection.color_rgb = cv::Scalar(128, 128, 128);
            return;
        }
        
        // 提取服装区域
        cv::Mat roi = image(safe_bbox);
        
        // 获取主要颜色
        cv::Scalar main_color = getMainColor(roi);
        detection.color_rgb = main_color;
        
        // 转换为HSV进行颜色分类
        cv::Mat bgr_sample(1, 1, CV_8UC3);
        bgr_sample.at<cv::Vec3b>(0, 0) = cv::Vec3b(
            static_cast<uchar>(main_color[0]), // B
            static_cast<uchar>(main_color[1]), // G  
            static_cast<uchar>(main_color[2])  // R
        );
        cv::Mat hsv_converted;
        cv::cvtColor(bgr_sample, hsv_converted, cv::COLOR_BGR2HSV);
        cv::Vec3b hsv_pixel = hsv_converted.at<cv::Vec3b>(0, 0);
        cv::Scalar hsv_color(hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]);
        
        // 获取颜色名称
        detection.color = getColorName(hsv_color);
        
        RCLCPP_INFO(this->get_logger(), "detect %s color: %s, BGR(%.0f,%.0f,%.0f), HSV(%.0f,%.0f,%.0f)", 
                    detection.category.c_str(), detection.color.c_str(),
                    main_color[0], main_color[1], main_color[2], // BGR原始顺序
                    hsv_color[0], hsv_color[1], hsv_color[2]); // HSV值
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "颜色检测出错: %s", e.what());
        detection.color = "unknown";
        detection.color_rgb = cv::Scalar(128, 128, 128);
    }
}

cv::Scalar IntegratedPersonDetectionNode::getMainColor(const cv::Mat& roi)
{
    // 缩小ROI以提高性能
    cv::Mat small_roi;
    cv::resize(roi, small_roi, cv::Size(32, 32));
    
    // 直接计算平均颜色，不使用复杂的掩码
    // 因为服装检测已经给出了相对准确的边界框
    cv::Scalar mean_color = cv::mean(small_roi);
    
    RCLCPP_DEBUG(rclcpp::get_logger("color_debug"), 
                "ROI size: %dx%d, Mean BGR: (%.1f, %.1f, %.1f)", 
                small_roi.cols, small_roi.rows, 
                mean_color[0], mean_color[1], mean_color[2]);
    
    return mean_color;
}

std::string IntegratedPersonDetectionNode::getColorName(const cv::Scalar& hsv_color)
{
    int h = static_cast<int>(hsv_color[0]);
    int s = static_cast<int>(hsv_color[1]);
    int v = static_cast<int>(hsv_color[2]);
    
    RCLCPP_INFO(rclcpp::get_logger("color_debug"), 
               "HSV analysis: H=%d, S=%d, V=%d", h, s, v);
    
    // 优化的颜色判断逻辑
    // 首先判断亮度极值
    if (v < 30) {
        RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Very dark -> black");
        return "black";
    }
    
    if (v > 240 && s < 20) {
        RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Very bright + low saturation -> white");
        return "white";
    }
    
    // 低饱和度的情况（黑白灰）
    if (s < 25) {
        if (v < 80) {
            RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Low saturation + dark -> black");
            return "black";
        } else if (v > 180) {
            RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Low saturation + bright -> white");
            return "white";
        } else {
            RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Low saturation + medium -> gray");
            return "gray";
        }
    }
    
    // 中等亮度但低饱和度
    if (v < 80) {
        RCLCPP_INFO(rclcpp::get_logger("color_debug"), "Low brightness -> dark");
        return "dark";
    }
    
    // 根据色相判断彩色
    std::string color_result;
    if (h < 8 || h >= 172) color_result = "red";
    else if (h < 25) color_result = "orange";  
    else if (h < 35) color_result = "yellow";
    else if (h < 78) color_result = "green";
    else if (h < 125) color_result = "blue";
    else if (h < 155) color_result = "purple";
    else color_result = "pink";
    
    RCLCPP_INFO(rclcpp::get_logger("color_debug"), 
               "Color based on hue %d -> %s", h, color_result.c_str());
    
    return color_result;
}

// YOLOv8 Pose模型初始化
bool IntegratedPersonDetectionNode::initializePoseModel()
{
    memset(&pose_rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
    
    // 初始化YOLOv8 Pose后处理
    if (init_pose_post_process() != 0) {
        RCLCPP_ERROR(this->get_logger(), "初始化姿态后处理失败");
        return false;
    }
    
    // 初始化YOLOv8 Pose模型
    int ret = init_yolov8_pose_model(pose_model_path_.c_str(), &pose_rknn_app_ctx_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "YOLOv8 Pose模型初始化失败! ret=%d model_path=%s", 
                     ret, pose_model_path_.c_str());
        return false;
    }
    
    pose_models_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "YOLOv8 Pose模型初始化成功");
    return true;
}

void IntegratedPersonDetectionNode::cleanupPoseModel()
{
    if (pose_models_initialized_) {
        release_yolov8_pose_model(&pose_rknn_app_ctx_);
        deinit_pose_post_process();
        pose_models_initialized_ = false;
    }
}

cv::Mat IntegratedPersonDetectionNode::extractPersonROI(const cv::Mat& image, const cv::Rect& person_bbox)
{
    // 确保边界框在图像范围内
    cv::Rect safe_bbox = person_bbox & cv::Rect(0, 0, image.cols, image.rows);
    if (safe_bbox.width <= 0 || safe_bbox.height <= 0) {
        RCLCPP_WARN(this->get_logger(), "无效的人体边界框");
        return cv::Mat();
    }
    
    // 提取人体区域
    cv::Mat person_roi = image(safe_bbox);
    
    // 可选：对ROI进行预处理（如调整大小等）
    // 这里保持原始大小，让模型内部处理
    
    return person_roi.clone();
}

void IntegratedPersonDetectionNode::detectPersonKeypoints(const cv::Mat& image, PersonInfo& person)
{
    if (!pose_models_initialized_) {
        RCLCPP_WARN(this->get_logger(), "YOLOv8 Pose模型未初始化");
        return;
    }
    
    try {
        // 提取人体区域
        cv::Mat person_roi = extractPersonROI(image, person.person_bbox);
        if (person_roi.empty()) {
            RCLCPP_WARN(this->get_logger(), "无法提取人体区域 for %s", person.person_id.c_str());
            return;
        }
        
        // 准备图像数据
        image_buffer_t src_image;
        memset(&src_image, 0, sizeof(image_buffer_t));
        
        src_image.width = person_roi.cols;
        src_image.height = person_roi.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.size = person_roi.cols * person_roi.rows * 3;
        
        // 转换BGR到RGB
        cv::Mat rgb_image;
        cv::cvtColor(person_roi, rgb_image, cv::COLOR_BGR2RGB);
        src_image.virt_addr = rgb_image.data;
        
        // 执行推理
        pose_object_detect_result_list od_results;
        int ret = inference_yolov8_pose_model(&pose_rknn_app_ctx_, &src_image, &od_results);
        
        if (ret == 0 && od_results.count > 0) {
            // 取第一个检测结果（应该是最高置信度的人体）
            const pose_object_detect_result& result = od_results.results[0];
            
            // 将关键点坐标转换回原图坐标系
            cv::Rect safe_bbox = person.person_bbox & cv::Rect(0, 0, image.cols, image.rows);
            float scale_x = static_cast<float>(person_roi.cols) / safe_bbox.width;
            float scale_y = static_cast<float>(person_roi.rows) / safe_bbox.height;
            
            for (int i = 0; i < 17; i++) {
                // 转换坐标：ROI坐标 -> 原图坐标
                person.keypoints[i][0] = result.keypoints[i][0] / scale_x + safe_bbox.x;
                person.keypoints[i][1] = result.keypoints[i][1] / scale_y + safe_bbox.y;
                person.keypoints[i][2] = result.keypoints[i][2]; // 置信度保持不变
            }
            
            person.has_keypoints = true;
            
            // 计算身体比例
            if (calculateBodyRatios(person)) {
                RCLCPP_DEBUG(this->get_logger(), "成功计算人员 %s 的身体比例", person.person_id.c_str());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "人员 %s 身体比例计算失败", person.person_id.c_str());
            }
            
            RCLCPP_DEBUG(this->get_logger(), "成功检测到人员 %s 的关键点", person.person_id.c_str());
            
        } else {
            RCLCPP_DEBUG(this->get_logger(), "人员 %s 关键点检测失败 ret=%d, count=%d", 
                        person.person_id.c_str(), ret, od_results.count);
            person.has_keypoints = false;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "关键点检测异常: %s", e.what());
        person.has_keypoints = false;
    }
}

void IntegratedPersonDetectionNode::drawKeypoints(cv::Mat& image, const PersonInfo& person)
{
    if (!person.has_keypoints) return;
    
    // 绘制关键点
    for (int k = 0; k < 17; k++) {
        float x = person.keypoints[k][0];
        float y = person.keypoints[k][1];
        float conf = person.keypoints[k][2];
        
        // 检查关键点是否在图像范围内且置信度足够
        if (conf > keypoint_confidence_threshold_ && 
            x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            
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
                       cv::Point(static_cast<int>(x)+3, static_cast<int>(y)-3),
                       cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
        }
    }
}

void IntegratedPersonDetectionNode::drawSkeleton(cv::Mat& image, const PersonInfo& person)
{
    if (!person.has_keypoints) return;
    
    // 绘制骨架连接
    for (const auto& connection : skeleton_connections_) {
        int idx1 = connection.first - 1;  // 转换为0-based索引
        int idx2 = connection.second - 1;
        
        if (idx1 >= 0 && idx1 < 17 && idx2 >= 0 && idx2 < 17) {
            float x1 = person.keypoints[idx1][0];
            float y1 = person.keypoints[idx1][1];
            float conf1 = person.keypoints[idx1][2];
            
            float x2 = person.keypoints[idx2][0];
            float y2 = person.keypoints[idx2][1];
            float conf2 = person.keypoints[idx2][2];
            
            // 检查坐标是否在图像范围内
            bool valid1 = conf1 > keypoint_confidence_threshold_ && 
                         x1 >= 0 && x1 < image.cols && y1 >= 0 && y1 < image.rows;
            bool valid2 = conf2 > keypoint_confidence_threshold_ && 
                         x2 >= 0 && x2 < image.cols && y2 >= 0 && y2 < image.rows;
            
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

// 身体比例计算实现
bool IntegratedPersonDetectionNode::calculateBodyRatios(PersonInfo& person)
{
    if (!person.has_keypoints) {
        RCLCPP_DEBUG(this->get_logger(), "没有关键点数据，无法计算身体比例");
        return false;
    }
    
    try {
        // YOLOv8 Pose关键点索引定义:
        // 0: 鼻子, 1: 左眼, 2: 右眼, 3: 左耳, 4: 右耳, 5: 左肩
        // 6: 右肩, 7: 左肘, 8: 右肘, 9: 左腕, 10: 右腕, 11: 左髋
        // 12: 右髋, 13: 左膝, 14: 右膝, 15: 左踝, 16: 右踝
        
        // 计算16个身体比例
        std::vector<float> ratios(16, 0.0f);
        
        // 1. 上肢与下肢比例
        float upper_limb = (calculateKeypointDistance(person.keypoints, 5, 7) + 
                           calculateKeypointDistance(person.keypoints, 7, 9) + 
                           calculateKeypointDistance(person.keypoints, 6, 8) + 
                           calculateKeypointDistance(person.keypoints, 8, 10)) / 4.0f;
                           
        float lower_limb = (calculateKeypointDistance(person.keypoints, 11, 13) + 
                           calculateKeypointDistance(person.keypoints, 13, 15) + 
                           calculateKeypointDistance(person.keypoints, 12, 14) + 
                           calculateKeypointDistance(person.keypoints, 14, 16)) / 4.0f;
                           
        if (upper_limb > 0 && lower_limb > 0) {
            ratios[0] = upper_limb / lower_limb;
        }
        
        // 2. 躯干与身高比例
        float torso_height = (calculateKeypointDistance(person.keypoints, 5, 11) + 
                             calculateKeypointDistance(person.keypoints, 6, 12)) / 2.0f;
                             
        float body_height = (calculateKeypointDistance(person.keypoints, 0, 15) + 
                            calculateKeypointDistance(person.keypoints, 0, 16)) / 2.0f;
                            
        if (torso_height > 0 && body_height > 0) {
            ratios[1] = torso_height / body_height;
        }
        
        // 3. 肩宽与身高比例
        float shoulder_width = calculateKeypointDistance(person.keypoints, 5, 6);
        if (shoulder_width > 0 && body_height > 0) {
            ratios[2] = shoulder_width / body_height;
        }
        
        // 4. 臀宽与肩宽比例
        float hip_width = calculateKeypointDistance(person.keypoints, 11, 12);
        if (hip_width > 0 && shoulder_width > 0) {
            ratios[3] = hip_width / shoulder_width;
        }
        
        // 5. 头部与躯干比例
        float head_height = (calculateKeypointDistance(person.keypoints, 0, 5) + 
                            calculateKeypointDistance(person.keypoints, 0, 6)) / 2.0f;
        if (head_height > 0 && torso_height > 0) {
            ratios[4] = head_height / torso_height;
        }
        
        // 6. 手臂与身高比例
        float arm_length = (calculateKeypointDistance(person.keypoints, 5, 9) + 
                           calculateKeypointDistance(person.keypoints, 6, 10)) / 2.0f;
        if (arm_length > 0 && body_height > 0) {
            ratios[5] = arm_length / body_height;
        }
        
        // 7. 腿长与身高比例
        float leg_length = (calculateKeypointDistance(person.keypoints, 11, 15) + 
                           calculateKeypointDistance(person.keypoints, 12, 16)) / 2.0f;
        if (leg_length > 0 && body_height > 0) {
            ratios[6] = leg_length / body_height;
        }
        
        // 8. 上臂与下臂比例
        float upper_arm = (calculateKeypointDistance(person.keypoints, 5, 7) + 
                          calculateKeypointDistance(person.keypoints, 6, 8)) / 2.0f;
        float lower_arm = (calculateKeypointDistance(person.keypoints, 7, 9) + 
                          calculateKeypointDistance(person.keypoints, 8, 10)) / 2.0f;
        if (upper_arm > 0 && lower_arm > 0) {
            ratios[7] = upper_arm / lower_arm;
        }
        
        // 9. 大腿与小腿比例
        float thigh = (calculateKeypointDistance(person.keypoints, 11, 13) + 
                      calculateKeypointDistance(person.keypoints, 12, 14)) / 2.0f;
        float calf = (calculateKeypointDistance(person.keypoints, 13, 15) + 
                     calculateKeypointDistance(person.keypoints, 14, 16)) / 2.0f;
        if (thigh > 0 && calf > 0) {
            ratios[8] = thigh / calf;
        }
        
        // 10. 躯干与腿长比例
        if (torso_height > 0 && leg_length > 0) {
            ratios[9] = torso_height / leg_length;
        }
        
        // 11. 手臂与腿长比例
        if (arm_length > 0 && leg_length > 0) {
            ratios[10] = arm_length / leg_length;
        }
        
        // 12. 肩宽与髋宽比例
        if (shoulder_width > 0 && hip_width > 0) {
            ratios[11] = shoulder_width / hip_width;
        }
        
        // 13. 头围与身高比例（估算头围）
        float head_width = calculateKeypointDistance(person.keypoints, 3, 4) * 1.2f; // 估算头围
        if (head_width > 0 && body_height > 0) {
            ratios[12] = head_width / body_height;
        }
        
        // 14. 脚长与身高比例（估算脚长）
        float foot_length = calculateKeypointDistance(person.keypoints, 15, 16) * 0.7f; // 估算脚长
        if (foot_length > 0 && body_height > 0) {
            ratios[13] = foot_length / body_height;
        }
        
        // 15. 脚踝宽与身高比例
        float ankle_width = calculateKeypointDistance(person.keypoints, 15, 16);
        if (ankle_width > 0 && body_height > 0) {
            ratios[14] = ankle_width / body_height;
        }
        
        // 16. 腰围与身高比例（估算腰围）
        float waist = hip_width * 0.85f; // 估算腰围
        if (waist > 0 && body_height > 0) {
            ratios[15] = waist / body_height;
        }
        
        // 复制比例数据到PersonInfo
        for (int i = 0; i < 16; i++) {
            person.body_ratios[i] = ratios[i];
        }
        
        // 检查是否有有效的比例数据
        int valid_ratios = 0;
        for (int i = 0; i < 16; i++) {
            if (person.body_ratios[i] > 0.0f) {
                valid_ratios++;
            }
        }
        
        person.has_body_ratios = (valid_ratios > 0);
        
        if (person.has_body_ratios) {
            RCLCPP_DEBUG(this->get_logger(), "成功计算 %d/16 个有效身体比例", valid_ratios);
        }
        
        return person.has_body_ratios;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "计算身体比例时出错: %s", e.what());
        person.has_body_ratios = false;
        return false;
    }
}

bool IntegratedPersonDetectionNode::isValidKeypoint(const float keypoints[17][3], int idx)
{
    if (idx < 0 || idx >= 17) {
        return false;
    }
    
    return (keypoints[idx][2] > 0.5f &&  // 置信度阈值
            keypoints[idx][0] > 0.0f &&  // x坐标有效
            keypoints[idx][1] > 0.0f);   // y坐标有效
}

float IntegratedPersonDetectionNode::calculateKeypointDistance(const float keypoints[17][3], int idx1, int idx2)
{
    if (!isValidKeypoint(keypoints, idx1) || !isValidKeypoint(keypoints, idx2)) {
        return 0.0f;
    }
    
    float dx = keypoints[idx1][0] - keypoints[idx2][0];
    float dy = keypoints[idx1][1] - keypoints[idx2][1];
    
    return std::sqrt(dx * dx + dy * dy);
}

void IntegratedPersonDetectionNode::modeCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    try {
        std::string mode_str = msg->data;
        DetectionMode new_mode;
        
        if (mode_str == "FULL" || mode_str == "full" || mode_str == "FULL_DETECTION") {
            new_mode = DetectionMode::FULL_DETECTION;
        } else if (mode_str == "PARTIAL" || mode_str == "partial" || mode_str == "PARTIAL_DETECTION") {
            new_mode = DetectionMode::PARTIAL_DETECTION;
        } else {
            RCLCPP_WARN(this->get_logger(), "未知的检测模式: %s, 支持的模式: FULL, PARTIAL", mode_str.c_str());
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (current_mode_ != new_mode) {
                current_mode_ = new_mode;
                std::string mode_name = (new_mode == DetectionMode::FULL_DETECTION) ? "完整检测" : "部分检测";
                RCLCPP_INFO(this->get_logger(), "检测模式已切换到: %s", mode_name.c_str());
                
                if (new_mode == DetectionMode::PARTIAL_DETECTION) {
                    RCLCPP_INFO(this->get_logger(), "部分检测模式: 仅进行服装检测，跳过关键点检测和身体比例计算");
                } else {
                    RCLCPP_INFO(this->get_logger(), "完整检测模式: 进行服装检测、关键点检测和身体比例计算");
                }
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理模式切换消息时出错: %s", e.what());
    }
}