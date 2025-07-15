#!/usr/bin/env python3
"""
集成人员检测节点测试脚本
测试YOLO11服装检测、服装配对和人体位置确定功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import json
import time

class IntegratedNodeTester(Node):
    
    def __init__(self):
        super().__init__('integrated_node_tester')
        
        # 订阅人员位置话题
        self.person_sub = self.create_subscription(
            String,
            '/person_detection/person_positions',
            self.person_callback,
            10
        )
        
        # 订阅调试图像话题
        self.debug_image_sub = self.create_subscription(
            Image,
            '/integrated_person/debug_image', 
            self.debug_image_callback,
            10
        )
        
        self.person_count = 0
        self.debug_image_count = 0
        self.last_log_time = time.time()
        
        self.get_logger().info("集成节点测试器启动")
        self.get_logger().info("等待人员检测数据...")
        
    def person_callback(self, msg):
        """处理人员位置消息"""
        try:
            data = json.loads(msg.data)
            self.person_count += 1
            
            current_time = time.time()
            if current_time - self.last_log_time > 2.0:  # 每2秒记录一次
                self.get_logger().info(f"收到人员数据 #{self.person_count}")
                self.get_logger().info(f"检测到人数: {data.get('person_count', 0)}")
                
                persons = data.get('persons', [])
                for i, person in enumerate(persons):
                    person_id = person.get('id', f'Unknown_{i}')
                    center = person.get('center', [0, 0])
                    distance = person.get('distance')
                    valid_distance = person.get('valid_distance', False)
                    
                    distance_str = f"{distance:.2f}m" if valid_distance else "N/A"
                    self.get_logger().info(f"  {person_id}: 中心({center[0]}, {center[1]}) 距离: {distance_str}")
                
                self.last_log_time = current_time
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"处理人员数据时出错: {e}")
    
    def debug_image_callback(self, msg):
        """处理调试图像消息"""
        self.debug_image_count += 1
        if self.debug_image_count % 30 == 0:  # 每30帧记录一次
            self.get_logger().info(f"收到调试图像 #{self.debug_image_count}")
            self.get_logger().info(f"图像尺寸: {msg.width}x{msg.height}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IntegratedNodeTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 