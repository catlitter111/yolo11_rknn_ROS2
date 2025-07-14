#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import time

class CategoryTestNode(Node):
    def __init__(self):
        super().__init__('category_test_node')
        
        # 订阅检测结果
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10)
        
        self.get_logger().info('类别测试节点已启动，等待检测结果...')
        self.detection_count = 0
        
    def detection_callback(self, msg):
        self.detection_count += 1
        
        if len(msg.detections) > 0:
            self.get_logger().info(f'检测到 {len(msg.detections)} 个物体:')
            
            for i, detection in enumerate(msg.detections):
                if detection.results:
                    class_id = detection.results[0].hypothesis.class_id
                    confidence = detection.results[0].hypothesis.score
                    
                    self.get_logger().info(f'  物体 {i+1}: 类别="{class_id}", 置信度={confidence:.3f}')
                    
                    # 检查类别是否为数字（说明修复失败）
                    if class_id.isdigit():
                        self.get_logger().warn(f'    警告: 类别仍显示为数字ID ({class_id}) 而不是类别名称!')
                    else:
                        self.get_logger().info(f'    ✅ 类别名称显示正确: {class_id}')
        else:
            self.get_logger().info(f'第 {self.detection_count} 次检测: 未检测到物体')

def main(args=None):
    rclpy.init(args=args)
    
    node = CategoryTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 