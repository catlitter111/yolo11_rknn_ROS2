#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通信优化测试脚本
测试RKNN YOLO11和Astra Camera的通信优化效果
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import time
import threading
from collections import deque
import numpy as np

class CommunicationOptimizationTest(Node):
    def __init__(self):
        super().__init__('communication_optimization_test')
        
        # 性能监控变量
        self.image_timestamps = deque(maxlen=100)
        self.detection_timestamps = deque(maxlen=100)
        self.detection_latencies = deque(maxlen=100)
        
        # 消息计数器
        self.image_count = 0
        self.detection_count = 0
        
        # 锁
        self.lock = threading.Lock()
        
        # 最近收到的图像时间戳
        self.last_image_timestamp = None
        
        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # 创建定时器进行性能报告
        self.timer = self.create_timer(2.0, self.report_performance)
        
        self.get_logger().info('通信优化测试节点已启动')
        self.get_logger().info('开始监控性能指标...')
        
    def image_callback(self, msg):
        current_time = time.time()
        
        with self.lock:
            self.image_timestamps.append(current_time)
            self.image_count += 1
            self.last_image_timestamp = msg.header.stamp
            
    def detection_callback(self, msg):
        current_time = time.time()
        
        with self.lock:
            self.detection_timestamps.append(current_time)
            self.detection_count += 1
            
            # 计算延迟 (如果有对应的图像时间戳)
            if self.last_image_timestamp is not None:
                image_time = self.last_image_timestamp.sec + self.last_image_timestamp.nanosec / 1e9
                detection_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                latency = detection_time - image_time
                self.detection_latencies.append(latency)
                
    def calculate_frequency(self, timestamps):
        """计算频率"""
        if len(timestamps) < 2:
            return 0.0
        
        time_diff = timestamps[-1] - timestamps[0]
        if time_diff > 0:
            return (len(timestamps) - 1) / time_diff
        return 0.0
        
    def calculate_latency_stats(self):
        """计算延迟统计"""
        if len(self.detection_latencies) == 0:
            return {'mean': 0, 'min': 0, 'max': 0, 'std': 0}
        
        latencies = np.array(self.detection_latencies)
        return {
            'mean': np.mean(latencies) * 1000,  # 转换为毫秒
            'min': np.min(latencies) * 1000,
            'max': np.max(latencies) * 1000,
            'std': np.std(latencies) * 1000
        }
        
    def report_performance(self):
        """定期报告性能"""
        with self.lock:
            image_freq = self.calculate_frequency(self.image_timestamps)
            detection_freq = self.calculate_frequency(self.detection_timestamps)
            latency_stats = self.calculate_latency_stats()
            
            self.get_logger().info('============= 通信优化性能报告 =============')
            self.get_logger().info(f'图像频率:     {image_freq:.1f} Hz')
            self.get_logger().info(f'检测频率:     {detection_freq:.1f} Hz')
            self.get_logger().info(f'图像总数:     {self.image_count}')
            self.get_logger().info(f'检测总数:     {self.detection_count}')
            
            if len(self.detection_latencies) > 0:
                self.get_logger().info(f'平均延迟:     {latency_stats["mean"]:.1f} ms')
                self.get_logger().info(f'最小延迟:     {latency_stats["min"]:.1f} ms')
                self.get_logger().info(f'最大延迟:     {latency_stats["max"]:.1f} ms')
                self.get_logger().info(f'延迟标准差:   {latency_stats["std"]:.1f} ms')
                
                # 性能评级
                mean_latency = latency_stats["mean"]
                if mean_latency < 30:
                    grade = "优秀"
                elif mean_latency < 50:
                    grade = "良好"
                elif mean_latency < 80:
                    grade = "一般"
                else:
                    grade = "需要改进"
                    
                self.get_logger().info(f'性能评级:     {grade}')
            
            # 效率分析
            if image_freq > 0:
                efficiency = (detection_freq / image_freq) * 100
                self.get_logger().info(f'处理效率:     {efficiency:.1f}%')
                
            self.get_logger().info('==========================================')

def main():
    rclpy.init()
    
    test_node = CommunicationOptimizationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 