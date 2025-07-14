# RKNN YOLO11 ROS2 Package

这是一个将瑞芯微RKNN YOLO11目标检测算法移植到ROS2 Humble的功能包。

## 功能特性

- 基于RKNN Runtime的高性能目标检测
- 支持实时图像流处理
- **集成双目立体视觉距离测量**
- **智能显示：检测框+距离信息叠加**
- 兼容ROS2 Humble版本
- 支持调试图像输出
- 可配置的检测参数
- 组件化设计，支持多种部署方式

## 系统要求

### 硬件要求
- 瑞芯微芯片平台（RK3588, RK3566, RV1106, RV1103等）
- 摄像头（可选）

### 软件要求
- Ubuntu 22.04
- ROS2 Humble
- OpenCV 4.x
- RKNN Runtime (librknnrt)

## 安装

### 1. 安装依赖

```bash
# 安装ROS2依赖
sudo apt update
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-image-transport

# 安装摄像头支持（可选）
sudo apt install ros-humble-v4l2-camera

# 确保已安装RKNN Runtime
# 通常在瑞芯微官方镜像中已预装
```

### 2. 编译功能包

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制此功能包到src目录
cp -r /path/to/rknn_yolo11_ros2 .

# 编译
cd ~/ros2_ws
colcon build --packages-select rknn_yolo11_ros2

# 加载环境
source install/setup.bash
```

## 使用方法

### 1. 配置模型路径

编辑配置文件 `config/rknn_yolo11_params.yaml`，设置正确的模型路径：

```yaml
rknn_yolo11_node:
  ros__parameters:
    model_path: "/userdata/rknn_yolo11_ros2/model/cloth.rknn"
```

### 2. 启动节点

#### 智能检测+距离测量系统（推荐）
```bash
source install/setup.bash
ros2 launch rknn_yolo11_ros2 yolo11_with_distance.launch.py camera_id:=1
```

#### 完整系统启动（基础版）- 双目相机+YOLO11检测
```bash
source install/setup.bash
ros2 launch rknn_yolo11_ros2 yolo11_with_stereo.launch.py
```

#### 仅YOLO11节点（需要其他图像源）
```bash
ros2 launch rknn_yolo11_ros2 rknn_yolo11_simple.launch.py model_path:=/path/to/your/model.rknn
```

#### 分步启动
```bash
# 1. 先启动双目相机
cd /userdata/stereo_demo
source install/setup.bash
ros2 launch stereo_camera_cpp stereo_camera.launch.py camera_id:=1

# 2. 再启动YOLO11检测（新终端）
cd /userdata/rknn_yolo11_ros2
source install/setup.bash
ros2 run rknn_yolo11_ros2 rknn_yolo11_ros2_node --ros-args --params-file config/rknn_yolo11_params.yaml
```

#### 使用参数文件启动
```bash
ros2 run rknn_yolo11_ros2 rknn_yolo11_ros2_node --ros-args --params-file config/rknn_yolo11_params.yaml
```

### 3. 查看检测结果

**智能显示窗口功能：**
- 实时显示原始图像
- 叠加YOLO检测框
- 显示每个目标的距离信息
- 统计信息（帧数、检测数、服务状态等）
- 按'q'或ESC退出

**命令行查看：**
```bash
# 查看检测结果
ros2 topic echo /detections

# 查看距离服务状态
ros2 service list | grep distance

# 手动测试距离服务
ros2 service call /stereo/get_distance stereo_camera_cpp/srv/GetDistance "{center_x: 320, center_y: 240, radius: 5}"

# 查看调试图像（如果启用）
ros2 run rqt_image_view rqt_image_view /yolo_debug_image

# 查看双目相机左目图像
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw

# 查看所有话题
ros2 topic list
```

### 4. 快速功能验证

```bash
# 验证完整系统
cd /userdata/rknn_yolo11_ros2
source install/setup.bash

# 启动智能检测+距离测量系统
ros2 launch rknn_yolo11_ros2 yolo11_with_distance.launch.py

# 在另一个终端检查话题
ros2 topic list
ros2 topic hz /detections
ros2 service list | grep distance
```

## 话题接口

### 订阅的话题
- `/stereo/left/image_raw` (sensor_msgs/Image): 双目相机左目原始图像
- `/detections` (vision_msgs/Detection2DArray): YOLO检测结果（显示节点订阅）

### 发布的话题
- `/detections` (vision_msgs/Detection2DArray): 目标检测结果
- `/yolo_debug_image` (sensor_msgs/Image): 带检测框的调试图像（可选）

### 服务接口
- `/stereo/get_distance` (stereo_camera_cpp/srv/GetDistance): 距离测量服务

### 双目相机相关话题
- `/stereo/left/image_raw` (sensor_msgs/Image): 左目原始图像
- `/stereo/right/image_raw` (sensor_msgs/Image): 右目原始图像
- `/stereo/disparity` (sensor_msgs/Image): 视差图（可选）

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| model_path | string | "" | RKNN模型文件路径 |
| input_topic | string | "/stereo_camera/left/image_rectified" | 输入图像话题 |
| output_topic | string | "/detections" | 检测结果话题 |
| debug_image_topic | string | "/yolo_debug_image" | 调试图像话题 |
| confidence_threshold | double | 0.25 | 置信度阈值 |
| nms_threshold | double | 0.45 | NMS阈值 |
| enable_debug_image | bool | true | 是否启用调试图像 |

## 性能优化建议

1. **选择合适的模型**：使用针对目标平台优化的RKNN模型
2. **调整图像分辨率**：降低输入图像分辨率可提高帧率
3. **启用零拷贝**：在支持的平台上启用零拷贝优化
4. **进程内通信**：对于同一进程内的节点，启用intra_process_comms

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查模型路径是否正确
   - 确认RKNN Runtime已正确安装
   - 验证模型与当前平台兼容

2. **摄像头无法打开**
   - 检查摄像头设备路径 `/dev/video0`
   - 确认摄像头权限：`sudo chmod 666 /dev/video0`

3. **编译错误**
   - 检查是否安装了所有ROS2依赖
   - 确认OpenCV版本兼容性

### 调试模式

启用详细日志：
```bash
ros2 run rknn_yolo11_ros2 rknn_yolo11_ros2_node --ros-args --log-level debug
```

## 开发说明

### 目录结构
```
rknn_yolo11_ros2/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 功能包描述
├── README.md              # 说明文档
├── config/                # 配置文件
├── include/               # 头文件
├── launch/                # 启动文件
├── model/                 # 模型文件
└── src/                   # 源代码
    ├── main.cpp           # 主程序
    ├── rknn_yolo11_node.cpp  # ROS2节点实现
    ├── postprocess.cc     # 后处理
    ├── yolo11.cc         # RKNN推理
    └── utils/            # 工具函数
```

### 扩展开发

要添加新功能或适配其他模型：

1. 修改 `postprocess.cc` 中的后处理逻辑
2. 调整 `rknn_yolo11_node.cpp` 中的消息转换
3. 更新相应的配置参数

## 许可证

Apache License 2.0

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

如有问题，请通过GitHub Issues联系。