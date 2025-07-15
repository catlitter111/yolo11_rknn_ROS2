#include "integrated_person_detection_node.hpp"
#include <algorithm>
#include <cmath>
#include <json/json.h>
#include <iomanip>
#include <sstream>

IntegratedPersonDetectionNode::IntegratedPersonDetectionNode(const rclcpp::NodeOptions & options)
: Node("integrated_person_detection_node", options), models_initialized_(false), frame_count_(0)
{
    // 声明参数
    this->declare_parameter("model_path", "./model/cloth.rknn");
    this->declare_parameter("input_topic", "/camera/color/image_raw");
    this->declare_parameter("person_topic", "/person_detection/person_positions");
    this->declare_parameter("distance_query_topic", "/depth_reader/get_depth_at");
    this->declare_parameter("distance_result_topic", "/depth_reader/depth_value");
    this->declare_parameter("debug_image_topic", "/integrated_person/debug_image");
    this->declare_parameter("confidence_threshold", 0.3);
    this->declare_parameter("nms_threshold", 0.5);
    this->declare_parameter("enable_debug_display", true);
    
    // 获取参数
    this->get_parameter("model_path", model_path_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("person_topic", person_topic_);
    this->get_parameter("distance_query_topic", distance_query_topic_);
    this->get_parameter("distance_result_topic", distance_result_topic_);
    this->get_parameter("debug_image_topic", debug_image_topic_);
    this->get_parameter("confidence_threshold", confidence_threshold_);
    this->get_parameter("nms_threshold", nms_threshold_);
    this->get_parameter("enable_debug_display", enable_debug_display_);
    
    // 初始化服装类别映射（基于YOLO11服装检测模型）
    initializeClothingCategories();
    
    // 初始化显示窗口
    window_name_ = "Integrated Person Detection";
    display_enabled_ = enable_debug_display_;
    
    RCLCPP_INFO(this->get_logger(), "集成人员检测节点初始化中...");
    RCLCPP_INFO(this->get_logger(), "模型路径: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "输入话题: %s", input_topic_.c_str());
    
    // 初始化模型
    if (!initializeModel()) {
        RCLCPP_ERROR(this->get_logger(), "模型初始化失败！");
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
        
        // 4. 查询距离信息
        for (auto& person : persons) {
            queryDistance(person.center, person.person_id);
        }
        
        // 5. 发布结果
        publishPersonPositions(persons);
        cv::Mat vis_image = publishVisualization(image, persons);
        
        // 6. 显示调试图像
        if (display_enabled_) {
            displayDebugImage(vis_image);
        }
        
        // 性能监控
        if (frame_count_ % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_process_time);
            double fps = 30000.0 / duration.count();
            RCLCPP_INFO(this->get_logger(), "处理FPS: %.1f, 帧数: %d, 检测到人数: %zu", 
                       fps, frame_count_, persons.size());
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
            int side_extension = static_cast<int>(upper_width * 0.3);
            
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
            RCLCPP_ERROR(this->get_logger(), "解析深度结果JSON失败");
            return;
        }
        
        int x = result["x"].asInt();
        int y = result["y"].asInt();
        bool valid = result.get("valid", false).asBool();
        float depth_m = result.get("depth_m", -1.0f).asFloat();
        
        // 查找对应的查询请求
        for (auto it = pending_queries_.begin(); it != pending_queries_.end(); ++it) {
            if (std::abs(it->query_point.x - x) < 1.0f && std::abs(it->query_point.y - y) < 1.0f) {
                // 更新人员缓存
                if (person_cache_.find(it->person_id) != person_cache_.end()) {
                    person_cache_[it->person_id].distance = depth_m;
                    person_cache_[it->person_id].valid_distance = valid;
                }
                
                // 移除已处理的查询
                pending_queries_.erase(it);
                break;
            }
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
            
            // 绘制上衣边界框
            if (person.clothing.has_upper) {
                cv::rectangle(vis_image, person.clothing.upper.bbox, cv::Scalar(255, 0, 0), 2);
                
                std::string upper_text = "Upper: " + 
                    std::to_string(static_cast<int>(person.clothing.upper.confidence * 100)) + "%";
                cv::putText(vis_image, upper_text,
                           cv::Point(person.clothing.upper.bbox.x, person.clothing.upper.bbox.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            }
            
            // 绘制下装边界框
            if (person.clothing.has_lower) {
                cv::rectangle(vis_image, person.clothing.lower.bbox, cv::Scalar(0, 0, 255), 2);
                
                std::string lower_text = "Lower: " + 
                    std::to_string(static_cast<int>(person.clothing.lower.confidence * 100)) + "%";
                cv::putText(vis_image, lower_text,
                           cv::Point(person.clothing.lower.bbox.x, person.clothing.lower.bbox.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }
            
            // 显示距离信息
            if (person.valid_distance) {
                std::stringstream ss;
                ss << "Dist: " << std::fixed << std::setprecision(2) << person.distance << "m";
                cv::putText(vis_image, ss.str(),
                           cv::Point(static_cast<int>(person.center.x - 30), 
                                   static_cast<int>(person.center.y + 20)),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }
            
            // 标记中心点
            cv::circle(vis_image, cv::Point(static_cast<int>(person.center.x), 
                                           static_cast<int>(person.center.y)), 5, cv::Scalar(255, 0, 0), -1);
        }
        
        // 添加统计信息
        std::string stats = "Persons: " + std::to_string(persons.size()) + 
                           ", Frame: " + std::to_string(frame_count_);
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