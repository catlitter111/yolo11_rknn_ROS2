# YOLOv8 Pose 故障排除指南

## 🚨 常见问题及解决方案

### 1. 节点崩溃 (Exit Code -11)

**现象**: 
```
[ERROR] [rknn_yolov8_pose_node-2]: process has died [pid 11781, exit code -11]
```

**原因**: 段错误（Segmentation Fault），通常由内存访问违规引起

**解决方案**:

#### A. 检查模型文件
```bash
# 检查模型文件是否存在且完整
ls -la install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/model/yolov8_pose.rknn

# 检查文件大小（应该在5MB左右）
du -h install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/model/yolov8_pose.rknn
```

#### B. 使用调试模式启动
```bash
# 使用 GDB 调试
source install/setup.bash
gdb --args ./install/rknn_yolo11_ros2/lib/rknn_yolo11_ros2/rknn_yolov8_pose_node --ros-args --params-file ./install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/config/rknn_yolov8_pose_params.yaml

# 在 GDB 中运行
(gdb) run
# 当崩溃时查看堆栈
(gdb) bt
```

#### C. 使用Valgrind检查内存
```bash
# 安装 valgrind（如果没有）
sudo apt install valgrind

# 运行内存检查
source install/setup.bash
valgrind --tool=memcheck --leak-check=full --track-origins=yes \
./install/rknn_yolo11_ros2/lib/rknn_yolo11_ros2/rknn_yolov8_pose_node \
--ros-args --params-file ./install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/config/rknn_yolov8_pose_params.yaml
```

### 2. 关键点不显示

**现象**: 检测到人员但没有关键点和骨架

**解决方案**:

#### A. 启用调试模式
```bash
# 编辑配置文件
nano config/rknn_yolov8_pose_params.yaml

# 设置调试参数
debug_keypoints: true
keypoint_confidence_threshold: 0.1  # 降低阈值
```

#### B. 检查关键点数据
使用调试脚本监控：
```bash
source install/setup.bash
python3 debug_pose.py
```

#### C. 查看详细日志
```bash
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py --ros-args --log-level debug
```

### 3. 图像卡死/显示延迟

**现象**: OpenCV 窗口卡死或更新缓慢

**解决方案**:

#### A. 检查话题频率
```bash
# 检查各话题的发布频率
ros2 topic hz /camera/color/image_raw
ros2 topic hz /yolov8_pose/image
ros2 topic hz /yolov8_pose/detections
```

#### B. 降低图像分辨率
```bash
# 在 launch 文件中修改相机参数
# yolov8_pose.launch.py 中的 astra_camera_node 参数
'color_width': 320,
'color_height': 240,
```

#### C. 禁用不必要的输出
```bash
# 暂时禁用显示节点测试
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py enable_display:=false
```

### 4. 相机连接问题

**现象**: 
```
[ERROR] Could not open camera
```

**解决方案**:

#### A. 检查USB连接
```bash
# 查看USB设备
lsusb | grep 2bc5

# 检查设备权限
ls -la /dev/video*
sudo chmod 666 /dev/video*
```

#### B. 重启相机服务
```bash
# 杀死现有相机进程
pkill -f astra_camera

# 重新插拔USB相机
# 然后重新启动
```

## 🔧 调试工具

### 1. 话题监控
```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /camera/color/image_raw
ros2 topic info /yolov8_pose/detections

# 查看话题数据
ros2 topic echo /yolov8_pose/detections --no-arr
```

### 2. 节点状态检查
```bash
# 查看运行的节点
ros2 node list

# 查看节点信息
ros2 node info /rknn_yolov8_pose_node

# 查看参数
ros2 param list /rknn_yolov8_pose_node
ros2 param get /rknn_yolov8_pose_node model_path
```

### 3. 系统资源监控
```bash
# 查看CPU和内存使用
htop

# 查看GPU使用（如果有）
nvidia-smi  # 或对于RK芯片: cat /sys/class/devfreq/fdab0000.gpu/load

# 查看系统负载
uptime
```

## 🛠️ 高级调试

### 1. 单步调试启动

#### 步骤1: 仅启动相机
```bash
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py use_camera:=true \
    rknn_yolov8_pose_node:=false pose_display_node:=false
```

#### 步骤2: 手动启动检测节点
```bash
# 新终端
source install/setup.bash
ros2 run rknn_yolo11_ros2 rknn_yolov8_pose_node --ros-args \
    --params-file install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/config/rknn_yolov8_pose_params.yaml
```

#### 步骤3: 启动显示节点
```bash
# 新终端
source install/setup.bash
ros2 run rknn_yolo11_ros2 pose_display_node --ros-args \
    --params-file install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/config/rknn_yolov8_pose_params.yaml
```

### 2. 模型验证

#### 使用静态图像测试
```bash
# 创建测试脚本
cat > test_static_image.py << EOF
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class StaticImagePublisher(Node):
    def __init__(self):
        super().__init__('static_image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        
        # 加载测试图像
        self.image = cv2.imread('/userdata/rknn_yolo11_ros2/model/bus.jpg')
        if self.image is None:
            self.get_logger().error("无法加载测试图像")
            return
            
        self.timer = self.create_timer(0.1, self.publish_image)  # 10Hz
        
    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = StaticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x test_static_image.py
```

### 3. 配置调优

#### 降低处理负载
```yaml
# config/rknn_yolov8_pose_params.yaml
rknn_yolov8_pose_node:
  ros__parameters:
    # 降低检测阈值，可能减少处理量
    confidence_threshold: 0.7
    nms_threshold: 0.5
    
    # 降低关键点阈值
    keypoint_confidence_threshold: 0.1
    
    # 禁用一些输出以减少负载
    show_fps: false
    debug_keypoints: false
```

## 📊 性能优化建议

### 1. 系统级优化
```bash
# 设置CPU性能模式
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 增加共享内存（如果需要）
sudo mount -o remount,size=512M /dev/shm
```

### 2. ROS2优化
```bash
# 使用单线程执行器
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI='<Domain id="0"><Tracing><Verbosity>severe</Verbosity></Tracing></Domain>'
```

### 3. 编译优化
```bash
# 使用优化编译标志
colcon build --packages-select rknn_yolo11_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -march=native"
```

## 📝 日志分析

### 常见错误消息及含义

| 错误消息 | 含义 | 解决方案 |
|---------|------|----------|
| `exit code -11` | 段错误 | 检查内存访问，使用调试器 |
| `Failed to init yolov8 pose model` | 模型加载失败 | 检查模型文件路径和权限 |
| `invalid keypoints_index` | 关键点索引越界 | 检查模型输出格式 |
| `cv_bridge exception` | 图像转换失败 | 检查图像格式和编码 |
| `Could not open camera` | 相机打开失败 | 检查USB连接和权限 |

### 收集完整日志
```bash
# 启动时保存完整日志
ros2 launch rknn_yolo11_ros2 yolov8_pose.launch.py 2>&1 | tee pose_debug.log

# 分析特定错误
grep -i "error\|fail\|crash\|segmentation" pose_debug.log
```

## 🆘 获取帮助

如果以上方法都无法解决问题，请收集以下信息：

1. **系统信息**:
```bash
uname -a
cat /proc/cpuinfo | head -20
free -h
df -h
```

2. **ROS2环境**:
```bash
printenv | grep ROS
ros2 doctor
```

3. **完整错误日志**:
- 启动日志
- 崩溃时的backtrace（如果使用GDB）
- 相关话题的数据示例

4. **模型文件信息**:
```bash
file install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/model/yolov8_pose.rknn
md5sum install/rknn_yolo11_ros2/share/rknn_yolo11_ros2/model/yolov8_pose.rknn
```

---

*最后更新: 2024年7月* 