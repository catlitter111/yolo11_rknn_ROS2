# 集成人员检测节点 - 快速启动指南

## 🚀 一键启动

```bash
# 1. 编译项目
cd /userdata/rknn_yolo11_ros2
colcon build --packages-select rknn_yolo11_ros2

# 2. 加载环境
source install/setup.bash

# 3. 启动系统
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py
```

## ✅ 验证运行

### 检查话题
```bash
# 查看人员检测结果
ros2 topic echo /person_detection/person_positions

# 查看调试图像
ros2 topic hz /integrated_person/debug_image
```

### 运行测试
```bash
python3 test_integrated_node.py
```

## 🎯 核心功能

- ✅ **YOLO11服装检测**: 13类服装实时检测
- ✅ **智能配对**: 自动匹配上衣下装
- ✅ **人体定位**: 精确确定人体位置
- ✅ **距离测量**: 集成深度信息查询
- ✅ **实时显示**: OpenCV窗口可视化

## 🔧 常用参数调整

```bash
# 降低检测阈值，检测更多目标
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    confidence_threshold:=0.2

# 关闭调试显示，提高性能
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    enable_debug_display:=false

# 使用自定义模型
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    yolo11_model_path:=/path/to/your/model.rknn
```

## 📊 输出示例

```json
{
    "timestamp": 1705123456789,
    "person_count": 1,
    "persons": [
        {
            "id": "Person_0",
            "bbox": [120, 80, 280, 450],
            "center": [200, 265],
            "distance": 1.75,
            "valid_distance": true
        }
    ]
}
```

## 🐛 快速故障排除

| 问题 | 解决方案 |
|------|----------|
| 模型加载失败 | 检查 `/userdata/rknn_yolo11_ros2/model/cloth.rknn` 是否存在 |
| 无图像输入 | 确认相机节点运行：`ros2 node list \| grep camera` |
| 距离信息无效 | 检查深度服务：`ros2 node list \| grep depth` |
| 检测效果差 | 调整光照，降低置信度阈值 |

## 📱 界面说明

OpenCV调试窗口显示：
- 🟢 **绿框**: 人体边界框
- 🔵 **蓝框**: 上衣检测框  
- 🔴 **红框**: 下装检测框
- 🔵 **蓝点**: 人体中心点
- 🟡 **黄字**: 距离信息

按 `q` 或 `ESC` 退出程序。

## 📋 系统要求

- Ubuntu 20.04 / ROS2 Humble
- RKNN Runtime 1.4+
- OpenCV 4.2+
- 相机: Astra Pro / Astra Pro Plus
- 内存: 4GB+ 推荐
- 存储: 2GB+ 可用空间

## 🔗 相关命令

```bash
# 查看节点信息
ros2 node info /integrated_person_detection_node

# 监控系统性能
htop

# 查看日志
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py \
    --ros-args --log-level DEBUG
```

---

🎉 **恭喜！** 您的集成人员检测系统已经成功运行！

如需详细配置和扩展功能，请参考 [完整使用说明](INTEGRATED_PERSON_DETECTION_USAGE.md)。 