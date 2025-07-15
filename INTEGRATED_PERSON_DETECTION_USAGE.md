# 集成人员检测节点使用说明

## 概述

本节点集成了YOLO11服装检测、服装配对和人体位置确定功能，提供完整的人员检测解决方案。

## 功能特性

1. **YOLO11服装检测**: 检测13类服装（上衣6类，下装7类）
2. **智能服装配对**: 自动匹配上衣和下装，确定完整的服装搭配
3. **人体位置估算**: 基于服装检测结果推算整个人体的边界框
4. **距离查询集成**: 自动查询人体中心点的距离信息
5. **实时可视化**: OpenCV调试窗口显示检测结果
6. **高性能设计**: 优化的C++实现，支持实时处理

## 安装和编译

### 依赖项
- ROS2 Humble
- OpenCV 4.x
- RKNN Runtime
- jsoncpp
- astra_camera (用于深度相机)
- astra_depth_reader (用于距离查询)

### 编译
```bash
cd /userdata/rknn_yolo11_ros2
colcon build --packages-select rknn_yolo11_ros2
source install/setup.bash
```

## 使用方法

### 1. 启动完整系统
```bash
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py
```

### 2. 自定义参数启动
```bash
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    yolo11_model_path:=/path/to/your/cloth.rknn \
    confidence_threshold:=0.3 \
    nms_threshold:=0.5 \
    enable_debug_display:=true
```

### 3. 单独运行节点
```bash
ros2 run rknn_yolo11_ros2 integrated_person_detection_node \
    --ros-args \
    -p model_path:=/userdata/rknn_yolo11_ros2/model/cloth.rknn \
    -p input_topic:=/camera/color/image_raw \
    -p confidence_threshold:=0.3
```

## 配置参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `model_path` | string | "./model/cloth.rknn" | YOLO11模型文件路径 |
| `input_topic` | string | "/camera/color/image_raw" | 输入图像话题 |
| `person_topic` | string | "/person_detection/person_positions" | 人员位置输出话题 |
| `distance_query_topic` | string | "/depth_reader/get_depth_at" | 距离查询话题 |
| `distance_result_topic` | string | "/depth_reader/depth_value" | 距离结果话题 |
| `debug_image_topic` | string | "/integrated_person/debug_image" | 调试图像输出话题 |
| `confidence_threshold` | double | 0.3 | 检测置信度阈值 |
| `nms_threshold` | double | 0.5 | NMS阈值 |
| `enable_debug_display` | bool | true | 启用OpenCV调试显示 |

## 话题接口

### 订阅话题
- `/camera/color/image_raw` (sensor_msgs/Image): 输入图像
- `/depth_reader/depth_value` (std_msgs/String): 距离查询结果

### 发布话题
- `/person_detection/person_positions` (std_msgs/String): 人员位置信息
- `/depth_reader/get_depth_at` (std_msgs/String): 距离查询请求
- `/integrated_person/debug_image` (sensor_msgs/Image): 调试可视化图像

## 输出数据格式

### 人员位置信息 (JSON格式)
```json
{
    "timestamp": 1705123456789,
    "person_count": 2,
    "persons": [
        {
            "id": "Person_0",
            "bbox": [100, 50, 300, 400],
            "center": [200, 225],
            "distance": 1.85,
            "valid_distance": true
        },
        {
            "id": "Person_1", 
            "bbox": [350, 60, 550, 420],
            "center": [450, 240],
            "distance": null,
            "valid_distance": false
        }
    ]
}
```

## 服装类别

### 上衣类别 (ID 0-5)
- 0: 短袖衬衫 (short_sleeved_shirt)
- 1: 长袖衬衫 (long_sleeved_shirt)
- 2: 短袖外套 (short_sleeved_outwear)
- 3: 长袖外套 (long_sleeved_outwear)
- 4: 背心 (vest)
- 5: 吊带 (sling)

### 下装类别 (ID 6-12)
- 6: 短裤 (shorts)
- 7: 长裤 (trousers)
- 8: 裙子 (skirt)
- 9: 短袖连衣裙 (short_sleeved_dress)
- 10: 长袖连衣裙 (long_sleeved_dress)
- 11: 背心裙 (vest_dress)
- 12: 吊带裙 (sling_dress)

## 调试和测试

### 1. 运行测试脚本
```bash
python3 test_integrated_node.py
```

### 2. 查看话题信息
```bash
# 查看人员位置信息
ros2 topic echo /person_detection/person_positions

# 查看话题列表
ros2 topic list | grep integrated
```

### 3. 检查节点状态
```bash
ros2 node info /integrated_person_detection_node
```

## 性能优化

### 1. 调整检测阈值
- 降低 `confidence_threshold` 可以检测更多目标，但可能增加误检
- 提高 `nms_threshold` 可以减少重复检测

### 2. 图像分辨率
- 输入图像分辨率越高，检测精度越好，但处理速度越慢
- 建议使用640x480或1280x720分辨率

### 3. 系统资源
- 确保系统有足够的内存和计算资源
- 监控CPU和GPU使用率

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查模型文件路径是否正确
   - 确认RKNN Runtime已正确安装

2. **图像话题无数据**
   - 检查相机节点是否正常运行
   - 确认话题名称匹配

3. **距离信息无效**
   - 检查深度相机是否正常工作
   - 确认astra_depth_reader节点运行正常

4. **检测精度不佳**
   - 调整confidence_threshold参数
   - 确保光照条件良好
   - 检查模型是否适合当前场景

### 日志级别
```bash
# 启用详细日志
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    --ros-args --log-level DEBUG
```

## 扩展功能

本节点设计时考虑了扩展性，可以轻松添加以下功能：

1. **多人跟踪**: 结合卡尔曼滤波器实现人员ID保持
2. **行为分析**: 基于人体位置和姿态分析行为模式
3. **属性识别**: 扩展识别年龄、性别等属性
4. **路径规划**: 为机器人导航提供人员位置信息

## 许可证

本项目遵循 Apache 2.0 许可证。 