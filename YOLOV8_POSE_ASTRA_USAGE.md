# YOLOv8 姿态检测系统使用指南 (Astra相机版)

## 概述

本系统集成了 Astra 深度相机和 YOLOv8 姿态检测，实现实时人体姿态检测功能。系统支持 17 个关键点检测和骨架可视化。

## 系统架构

```
Astra相机 -> /camera/color/image_raw -> YOLOv8检测节点 -> /yolov8_pose/image -> 显示节点
                                                   |
                                                   v
                                            /yolov8_pose/detections
```

## 功能特性

- ✅ **实时姿态检测**: 基于 YOLOv8 的人体姿态检测
- ✅ **Astra相机支持**: 集成 Astra 深度相机输入
- ✅ **17关键点检测**: 支持 COCO 格式的 17 个关键点
- ✅ **骨架可视化**: 实时显示人体骨架连接线
- ✅ **多话题输出**: 检测结果和可视化图像分别发布
- ✅ **参数配置**: 支持动态参数调整
- ✅ **性能监控**: 实时 FPS 显示

## 快速启动

### 1. 编译项目

```bash
cd /userdata/rknn_yolo11_ros2
colcon build --packages-select rknn_yolo11_ros2
source install/setup.bash
```

### 2. 启动系统

```bash
# 启动完整系统（包含 Astra 相机）
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py

# 或者使用自定义参数
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py \
    camera_name:=camera \
    input_topic:=/camera/color/image_raw \
    show_fps:=true \
    enable_display:=true
```

### 3. 不使用相机（外部图像源）

```bash
# 禁用 Astra 相机节点，使用外部图像源
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py use_camera:=false
```

## 话题说明

### 输入话题
- `/camera/color/image_raw` - 来自 Astra 相机的彩色图像

### 输出话题
- `/yolov8_pose/image` - 带有姿态检测可视化的图像
- `/yolov8_pose/detections` - 检测结果 (vision_msgs/Detection2DArray)

### 相机话题
- `/camera/color/image_raw` - 彩色图像
- `/camera/depth/image_raw` - 深度图像
- `/camera/color/camera_info` - 相机内参

## 启动参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `model_path` | string | `model/yolov8_pose.rknn` | YOLOv8模型文件路径 |
| `camera_name` | string | `camera` | 相机名称前缀 |
| `input_topic` | string | `/camera/color/image_raw` | 输入图像话题 |
| `output_topic` | string | `/yolov8_pose/image` | 输出图像话题 |
| `detection_topic` | string | `/yolov8_pose/detections` | 检测结果话题 |
| `show_fps` | bool | `true` | 显示FPS |
| `enable_visualization` | bool | `true` | 启用可视化 |
| `enable_display` | bool | `true` | 启用OpenCV显示窗口 |
| `use_camera` | bool | `true` | 启用Astra相机节点 |
| `use_rviz` | bool | `false` | 启动RViz |

## 配置文件

### 主配置文件: `config/rknn_yolov8_pose_params.yaml`

```yaml
rknn_yolov8_pose_node:
  ros__parameters:
    # 模型路径
    model_path: "./model/yolov8_pose.rknn"
    
    # 话题配置
    input_topic: "/camera/color/image_raw"
    output_topic: "/yolov8_pose/image"
    detection_topic: "/yolov8_pose/detections"
    
    # 显示配置
    show_fps: true
    enable_visualization: true
    
    # 检测参数
    confidence_threshold: 0.5
    nms_threshold: 0.4
    keypoint_confidence_threshold: 0.5

pose_display_node:
  ros__parameters:
    input_topic: "/yolov8_pose/image"
    window_name: "YOLOv8 Pose Detection"
    enable_display: true
```

## 节点说明

### 1. `astra_camera_node`
- **功能**: Astra 深度相机驱动
- **输出**: 彩色图像、深度图像、相机内参
- **配置**: 640x480@30fps，MJPEG格式

### 2. `rknn_yolov8_pose_node`
- **功能**: YOLOv8 姿态检测主节点
- **输入**: 彩色图像
- **输出**: 检测结果、可视化图像
- **性能**: 支持RKNN加速推理

### 3. `pose_display_node`
- **功能**: OpenCV图像显示
- **输入**: 可视化图像
- **功能**: 实时显示检测结果

## 性能优化

### 1. 相机配置优化
- 使用 MJPEG 压缩格式减少带宽
- 640x480 分辨率平衡性能和质量
- 30fps 帧率确保实时性

### 2. 检测参数调优
```yaml
confidence_threshold: 0.5      # 提高减少误检
nms_threshold: 0.4            # 调整重叠框过滤
keypoint_confidence_threshold: 0.5  # 关键点置信度
```

## 故障排除

### 1. 相机连接问题
```bash
# 检查USB设备
lsusb | grep 2bc5

# 检查相机权限
sudo chmod 666 /dev/video*
```

### 2. 模型文件问题
```bash
# 检查模型文件
ls -la install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/model/yolov8_pose.rknn
```

### 3. 话题检查
```bash
# 查看可用话题
ros2 topic list

# 监听图像话题
ros2 topic echo /camera/color/image_raw --no-arr

# 监听检测结果
ros2 topic echo /yolov8_pose/detections
```

### 4. 性能监控
```bash
# 查看节点信息
ros2 node info /rknn_yolov8_pose_node

# 查看话题频率
ros2 topic hz /camera/color/image_raw
ros2 topic hz /yolov8_pose/image
```

## 键盘控制

在显示窗口中：
- `q` 或 `ESC`: 退出程序
- `s`: 保存当前帧（如果实现）

## 开发指南

### 添加自定义可视化
1. 修改 `src/rknn_yolov8_pose_node.cpp` 中的 `draw_keypoints()` 函数
2. 调整 `skeleton_connections_` 配置

### 集成新的检测模型
1. 替换 `model/yolov8_pose.rknn` 文件
2. 更新 `model/yolov8_pose_labels_list.txt` 标签文件
3. 修改后处理参数

## 日志和调试

### 启用详细日志
```bash
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py --ros-args --log-level debug
```

### 查看系统状态
```bash
# 检查节点状态
ros2 node list
ros2 lifecycle get /rknn_yolov8_pose_node

# 查看参数
ros2 param list /rknn_yolov8_pose_node
ros2 param get /rknn_yolov8_pose_node model_path
```

## 版本信息

- **ROS2版本**: Humble
- **OpenCV版本**: 4.x
- **RKNN版本**: 2.x
- **Astra驱动**: ros2-astra-camera

## 支持和反馈

如果遇到问题，请检查：
1. 相机连接和权限
2. 模型文件完整性
3. ROS2环境配置
4. 系统资源占用

---

*最后更新: 2024年7月* 