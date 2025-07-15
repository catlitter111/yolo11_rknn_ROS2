#!/usr/bin/env python3
"""
YOLOv8 Pose 调试工具
用于监控节点状态、话题频率和关键点信息
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import time
import threading

class PoseDebugNode(Node):
    def __init__(self):
        super().__init__('pose_debug_node')
        
        # 统计信息
        self.image_count = 0
        self.detection_count = 0
        self.keypoint_count = 0
        self.last_image_time = 0
        self.last_detection_time = 0
        
        # 订阅话题
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolov8_pose/detections',
            self.detection_callback,
            10
        )
        
        self.output_image_sub = self.create_subscription(
            Image,
            '/yolov8_pose/image',
            self.output_image_callback,
            10
        )
        
        # 定时器打印统计信息
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("YOLOv8 Pose 调试节点已启动")
        self.get_logger().info("监控话题:")
        self.get_logger().info("  - /camera/color/image_raw (输入图像)")
        self.get_logger().info("  - /yolov8_pose/detections (检测结果)")
        self.get_logger().info("  - /yolov8_pose/image (输出图像)")

    def image_callback(self, msg):
        self.image_count += 1
        self.last_image_time = time.time()
        
        if self.image_count % 30 == 0:
            self.get_logger().info(f"接收到图像 #{self.image_count}, 尺寸: {msg.width}x{msg.height}")

    def detection_callback(self, msg):
        self.detection_count += 1
        self.last_detection_time = time.time()
        
        # 分析检测结果
        num_detections = len(msg.detections)
        if num_detections > 0:
            self.get_logger().info(f"检测到 {num_detections} 个目标")
            
            for i, detection in enumerate(msg.detections):
                if detection.results:
                    confidence = detection.results[0].hypothesis.score
                    class_id = detection.results[0].hypothesis.class_id
                    
                    bbox = detection.bbox
                    x = bbox.center.position.x
                    y = bbox.center.position.y
                    w = bbox.size_x
                    h = bbox.size_y
                    
                    self.get_logger().info(
                        f"  目标 {i+1}: {class_id} 置信度={confidence:.3f} "
                        f"位置=({x:.0f},{y:.0f}) 大小=({w:.0f}x{h:.0f})"
                    )

    def output_image_callback(self, msg):
        if self.detection_count % 10 == 0:
            self.get_logger().info(f"输出图像 #{self.detection_count}, 尺寸: {msg.width}x{msg.height}")

    def print_stats(self):
        current_time = time.time()
        
        # 计算帧率
        image_fps = "N/A"
        detection_fps = "N/A"
        
        if self.last_image_time > 0:
            image_age = current_time - self.last_image_time
            if image_age < 2.0:  # 如果最近2秒内有图像
                image_fps = f"{self.image_count / (current_time - (current_time - 30)):.1f}" if self.image_count > 0 else "0"
        
        if self.last_detection_time > 0:
            detection_age = current_time - self.last_detection_time
            if detection_age < 2.0:  # 如果最近2秒内有检测
                detection_fps = f"{self.detection_count / (current_time - (current_time - 30)):.1f}" if self.detection_count > 0 else "0"
        
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"📊 统计信息:")
        self.get_logger().info(f"  输入图像: {self.image_count} 帧")
        self.get_logger().info(f"  检测结果: {self.detection_count} 次")
        self.get_logger().info(f"  图像延迟: {current_time - self.last_image_time:.1f}s" if self.last_image_time > 0 else "  图像延迟: N/A")
        self.get_logger().info(f"  检测延迟: {current_time - self.last_detection_time:.1f}s" if self.last_detection_time > 0 else "  检测延迟: N/A")
        
        # 节点状态检查
        if current_time - self.last_image_time > 5.0 and self.last_image_time > 0:
            self.get_logger().warn("⚠️  输入图像流可能中断")
        
        if current_time - self.last_detection_time > 5.0 and self.last_detection_time > 0:
            self.get_logger().warn("⚠️  检测结果流可能中断")
        
        if self.image_count > 0 and self.detection_count == 0:
            self.get_logger().warn("⚠️  有输入图像但无检测结果，可能节点崩溃")

def main():
    rclpy.init()
    
    node = PoseDebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("调试节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 