#!/usr/bin/env python3
"""
测试脚本：检查YOLO11 + 双目相机系统的话题连接
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        
        # 监控各种话题
        self.topics_to_monitor = [
            '/stereo_camera/left/image_rectified',  # 双目相机左目
            '/stereo/left/image_raw',               # 可能的话题名称
            '/yolo_debug_image',                    # YOLO调试图像
            '/detections'                           # 检测结果
        ]
        
        # 计数器
        self.message_counts = {topic: 0 for topic in self.topics_to_monitor}
        
        # 创建订阅者
        self.image_subs = {}
        for topic in self.topics_to_monitor:
            if topic == '/detections':
                self.image_subs[topic] = self.create_subscription(
                    Detection2DArray, topic, 
                    lambda msg, t=topic: self.detection_callback(msg, t), 1)
            else:
                self.image_subs[topic] = self.create_subscription(
                    Image, topic, 
                    lambda msg, t=topic: self.image_callback(msg, t), 1)
        
        # 定时器 - 每5秒报告一次
        self.timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info('话题监控器已启动，监控以下话题:')
        for topic in self.topics_to_monitor:
            self.get_logger().info(f'  - {topic}')
    
    def image_callback(self, msg, topic):
        self.message_counts[topic] += 1
        if self.message_counts[topic] % 10 == 1:  # 每10条消息报告一次
            self.get_logger().info(f'收到 {topic} 消息 #{self.message_counts[topic]} (尺寸: {msg.width}x{msg.height})')
    
    def detection_callback(self, msg, topic):
        self.message_counts[topic] += 1
        detection_count = len(msg.detections)
        self.get_logger().info(f'收到 {topic} 消息 #{self.message_counts[topic]} (检测数量: {detection_count})')
    
    def report_status(self):
        self.get_logger().info('=== 话题状态报告 ===')
        for topic, count in self.message_counts.items():
            status = f'活跃 ({count} 消息)' if count > 0 else '无消息'
            self.get_logger().info(f'{topic}: {status}')
        self.get_logger().info('==================')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = TopicMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()