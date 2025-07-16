# 整合人员检测和跟踪系统使用指南

## 系统概述

该系统整合了 `integrated_person_detection_node` 和 `bytetracker_node`，提供完整的人员检测和跟踪功能：

- **服装检测**：基于YOLO11的上衣和下装检测
- **姿态检测**：基于YOLOv8的17个关键点检测
- **身体比例计算**：计算16个身体比例特征
- **多目标跟踪**：使用ByteTracker算法进行跟踪
- **单目标跟踪**：基于特征匹配的目标锁定

## 数据流

```
相机图像 → integrated_person_detection_node → /person_detection/person_positions (JSON)
                                                      ↓
                                           bytetracker_node → 跟踪结果
```

### 数据格式

**检测数据格式** (`/person_detection/person_positions`):
```json
{
  "timestamp": 1234567890123,
  "person_count": 2,
  "detection_mode": "FULL",
  "persons": [
    {
      "id": "Person_0",
      "bbox": [x1, y1, x2, y2],
      "center": [cx, cy],
      "distance": 2.5,
      "valid_distance": true,
      "clothing": {
        "upper": {"color_rgb": [255, 0, 0], "confidence": 0.8},
        "lower": {"color_rgb": [0, 0, 255], "confidence": 0.7}
      },
      "keypoints": [[x, y, conf], ...],  // 17个关键点
      "body_ratios": [0.45, 0.32, ...], // 16个比例值
      "has_keypoints": true,
      "has_body_ratios": true
    }
  ]
}
```

**跟踪数据格式** (bytetracker处理后):
```
(full_bbox, upper_color, lower_color, confidence, body_ratios)
```
- `full_bbox`: [x1, y1, x2, y2] 边界框
- `upper_color`: (B, G, R) BGR颜色元组（从RGB数组转换）
- `lower_color`: (B, G, R) BGR颜色元组（从RGB数组转换）
- `confidence`: 上衣置信度 + 下装置信度
- `body_ratios`: 16个身体比例值的列表

## 启动方法

### 1. 基本启动

```bash
# 启动完整系统（默认多目标跟踪）
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py

# 指定参数启动
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
  tracking_mode:=multi \
  confidence_threshold:=0.3 \
  color_weight:=0.5
```

### 2. 单目标跟踪启动

```bash
# 需要准备目标特征文件
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
  tracking_mode:=single \
  target_features_file:=/path/to/target_features.xlsx
```

### 3. 分步启动（调试用）

```bash
# 终端1: 启动相机
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py use_depth_service:=false

# 终端2: 启动深度服务（如需要）
ros2 run astra_depth_reader depth_service

# 终端3: 启动ByteTracker
ros2 run following_robot bytetracker_node.py
```

## 启动参数

### 基础参数
- `yolo11_model_path`: YOLO11服装检测模型路径
- `pose_model_path`: YOLOv8姿态检测模型路径
- `camera_name`: 相机命名空间 (默认: "camera")
- `input_topic`: 输入图像话题 (默认: "/camera/color/image_raw")
- `confidence_threshold`: 检测置信度阈值 (默认: 0.3)
- `nms_threshold`: NMS阈值 (默认: 0.6)
- `enable_debug_display`: 是否显示调试窗口 (默认: true)

### 跟踪参数
- `tracking_mode`: 跟踪模式 ("multi" 或 "single", 默认: "multi")
- `target_features_file`: 目标特征文件路径（单目标模式用）
- `track_thresh`: 跟踪置信度阈值 (默认: 0.5)
- `color_weight`: 颜色特征权重 (默认: 0.5)

### 深度参数
- `use_depth_service`: 是否启用深度服务 (默认: true)

## 测试和验证

### 1. 运行测试脚本

```bash
# 监控数据流
python3 test_integrated_bytetracker.py
```

### 2. 检查话题

```bash
# 查看所有话题
ros2 topic list

# 监控检测数据
ros2 topic echo /person_detection/person_positions

# 监控跟踪结果
ros2 topic echo /bytetracker/tracked_persons
ros2 topic echo /bytetracker/tracking_result
```

### 3. 检查节点状态

```bash
# 查看节点信息
ros2 node list
ros2 node info /integrated_person_detection_node
ros2 node info /bytetracker_node
```

## 模式切换

### 运行时切换检测模式

```bash
# 切换到完整检测（服装+姿态）
ros2 topic pub /integrated_person/detection_mode std_msgs/String "data: 'FULL'"

# 切换到部分检测（仅服装）
ros2 topic pub /integrated_person/detection_mode std_msgs/String "data: 'PARTIAL'"
```

### 运行时切换跟踪模式

```bash
# 切换到多目标跟踪
ros2 topic pub /bytetracker/set_mode std_msgs/String "data: 'multi'"

# 切换到单目标跟踪
ros2 topic pub /bytetracker/set_mode std_msgs/String "data: 'single'"
```

## 故障排除

### 1. 检测节点问题

**症状**: 无检测数据输出
```bash
# 检查模型文件
ls -la /userdata/rknn_yolo11_ros2/model/cloth.rknn
ls -la /userdata/rknn_yolo11_ros2/model/yolov8_pose.rknn

# 检查相机数据
ros2 topic hz /camera/color/image_raw
```

### 2. 跟踪节点问题

**症状**: 有检测数据但无跟踪结果
```bash
# 检查消息依赖
ros2 interface show custom_msgs/msg/TrackedPerson

# 重新编译消息
cd ~/workspace
colcon build --packages-select custom_msgs
source install/setup.bash
```

### 3. 数据格式错误

**症状**: JSON解析错误
```bash
# 查看原始JSON数据
ros2 topic echo /person_detection/person_positions --once

# 检查数据格式
python3 -c "
import json
data = '{...}'  # 粘贴实际数据
parsed = json.loads(data)
print(json.dumps(parsed, indent=2))
"
```

### 4. 性能问题

**症状**: 处理延迟或丢帧
```bash
# 降低检测阈值
ros2 param set /integrated_person_detection_node confidence_threshold 0.5

# 切换到部分检测模式
ros2 topic pub /integrated_person/detection_mode std_msgs/String "data: 'PARTIAL'"

# 检查系统资源
htop
```

## 输出话题

### 检测节点输出
- `/person_detection/person_positions`: 人员位置和特征数据 (std_msgs/String)
- `/integrated_person/debug_image`: 调试图像 (sensor_msgs/Image)

### 跟踪节点输出
- `/bytetracker/tracked_persons`: 跟踪的人员数组 (custom_msgs/TrackedPersonArray)
- `/bytetracker/tracking_result`: 跟踪状态信息 (custom_msgs/TrackingResult)
- `/bytetracker/detailed_tracking_data`: 详细跟踪数据 (std_msgs/String)
- `/bytetracker/status`: 节点状态信息 (std_msgs/String)

## 性能优化建议

1. **并行推理**: 完整检测模式下，服装检测和姿态检测并行运行
2. **动态模式切换**: 根据需要在FULL和PARTIAL模式间切换
3. **阈值调整**: 根据场景调整置信度阈值
4. **颜色权重**: 调整颜色特征在跟踪中的权重

## 依赖项

### ROS2包
- `rknn_yolo11_ros2`
- `following_robot`
- `custom_msgs`
- `astra_camera`
- `astra_depth_reader`

### Python包
- `numpy`
- `opencv-python`
- `scipy`
- `openpyxl`
- `json`

### 系统库
- RKNN Runtime
- OpenCV
- 相机驱动程序

## 更新日志

- **v1.0**: 基础整合版本
- **v1.1**: 添加并行推理支持
- **v1.2**: 修复数据格式转换问题
- **v1.3**: 添加测试脚本和详细文档 