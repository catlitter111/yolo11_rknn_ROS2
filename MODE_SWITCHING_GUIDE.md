# 检测模式切换功能使用指南

## 概述

集成人员检测节点现在支持两种检测模式的动态切换：

- **完整检测模式 (FULL_DETECTION)**: 进行服装检测、关键点检测和身体比例计算
- **部分检测模式 (PARTIAL_DETECTION)**: 仅进行服装检测，跳过关键点检测和身体比例计算

## 功能特点

### 完整检测模式 (FULL)
- ✅ 服装检测 (上衣/下装识别和颜色检测)
- ✅ 关键点检测 (17个COCO关键点)
- ✅ 身体比例计算 (16种身体比例指标)
- ✅ 可视化骨架和关键点
- ⚡ 较高的计算负载

### 部分检测模式 (PARTIAL)
- ✅ 服装检测 (上衣/下装识别和颜色检测)
- ❌ 跳过关键点检测
- ❌ 跳过身体比例计算
- ❌ 无骨架可视化
- ⚡ 较低的计算负载，更高的FPS

## 使用方法

### 1. 参数配置

在启动节点时，可以配置模式切换话题：

```yaml
# config/integrated_person_params.yaml
mode_topic: "/integrated_person/detection_mode"  # 默认模式切换话题
```

### 2. 话题通信

#### 模式切换话题 (输入)
- **话题名**: `/integrated_person/detection_mode`
- **消息类型**: `std_msgs/String`
- **支持的命令**:
  - `"FULL"` 或 `"full"` 或 `"FULL_DETECTION"` - 切换到完整检测模式
  - `"PARTIAL"` 或 `"partial"` 或 `"PARTIAL_DETECTION"` - 切换到部分检测模式

#### 检测结果话题 (输出)
- **话题名**: `/person_detection/person_positions`
- **消息类型**: `std_msgs/String` (JSON格式)
- **新增字段**: `detection_mode` - 当前检测模式状态

### 3. 命令行控制

#### 快速切换模式
```bash
# 切换到完整检测模式
ros2 topic pub --once /integrated_person/detection_mode std_msgs/String "data: 'FULL'"

# 切换到部分检测模式
ros2 topic pub --once /integrated_person/detection_mode std_msgs/String "data: 'PARTIAL'"
```

#### 使用测试脚本
```bash
# 交互模式
./test_mode_switch.py

# 直接切换到完整检测模式
./test_mode_switch.py FULL

# 直接切换到部分检测模式
./test_mode_switch.py PARTIAL

# 自动测试模式切换
./test_mode_switch.py AUTO

# 监听模式状态
./test_mode_switch.py STATUS
```

### 4. 编程接口

#### Python示例
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')
        self.mode_pub = self.create_publisher(
            String, 
            '/integrated_person/detection_mode', 
            10
        )
    
    def set_full_mode(self):
        msg = String()
        msg.data = 'FULL'
        self.mode_pub.publish(msg)
    
    def set_partial_mode(self):
        msg = String()
        msg.data = 'PARTIAL'
        self.mode_pub.publish(msg)
```

#### C++示例
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ModeController : public rclcpp::Node {
public:
    ModeController() : Node("mode_controller") {
        mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/integrated_person/detection_mode", 10);
    }
    
    void setFullMode() {
        auto msg = std_msgs::msg::String();
        msg.data = "FULL";
        mode_pub_->publish(msg);
    }
    
    void setPartialMode() {
        auto msg = std_msgs::msg::String();
        msg.data = "PARTIAL";
        mode_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};
```

## 状态监控

### 可视化界面状态
在调试图像的顶部会显示当前模式：
```
Persons: 2, FPS: 28, Mode: FULL
```

### JSON消息状态
检测结果JSON中包含模式信息：
```json
{
  "timestamp": 1699123456789,
  "person_count": 2,
  "detection_mode": "FULL",
  "persons": [...]
}
```

## 应用场景

### 完整检测模式适用场景
- 需要详细人体分析的应用
- 身体比例测量
- 姿态识别和动作分析
- 人体建模和重建
- 医疗和健康监测

### 部分检测模式适用场景
- 实时人员跟踪
- 服装识别和分类
- 人流统计
- 视频监控
- 资源受限的设备

## 性能对比

| 模式 | 检测内容 | 平均FPS | CPU使用率 | 内存使用 |
|------|----------|---------|-----------|----------|
| FULL | 服装+关键点+比例 | ~20-25 | 高 | 高 |
| PARTIAL | 仅服装 | ~30-35 | 中等 | 中等 |

## 注意事项

1. **线程安全**: 模式切换使用互斥锁保护，可以在运行时安全切换
2. **状态一致性**: 切换模式后，输出的JSON格式保持一致，但关键点和身体比例字段在部分检测模式下为空
3. **性能优化**: 在部分检测模式下，YOLOv8 Pose模型仍会保持加载状态，只是跳过推理过程
4. **错误处理**: 无效的模式命令会被忽略，并记录警告日志

## 故障排除

### 常见问题

1. **模式切换无响应**
   - 检查话题名是否正确
   - 确认消息格式是否为字符串类型
   - 查看节点日志是否有错误信息

2. **性能没有明显提升**
   - 确认已成功切换到部分检测模式
   - 检查其他计算密集型操作
   - 监控系统资源使用情况

3. **JSON输出格式异常**
   - 在部分检测模式下，关键点和身体比例字段会被设置为false或空
   - 这是正常行为，不是错误

### 调试命令
```bash
# 查看模式切换话题
ros2 topic list | grep mode

# 监听模式切换消息
ros2 topic echo /integrated_person/detection_mode

# 查看检测结果
ros2 topic echo /person_detection/person_positions

# 查看节点日志
ros2 log view integrated_person_detection_node
``` 