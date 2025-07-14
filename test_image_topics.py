#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageTopicTester(Node):
    def __init__(self):
        super().__init__('image_topic_tester')
        self.bridge = CvBridge()
        
        # 订阅原始图像
        self.subscription = self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('图像话题测试节点已启动，等待图像数据...')
        self.image_count = 0
    
    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count % 30 == 0:  # 每30帧打印一次
            self.get_logger().info(f'收到图像 #{self.image_count}, 尺寸: {msg.width}x{msg.height}')
            
            # 转换并显示图像
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow('Stereo Left Image', cv_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'图像转换错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageTopicTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()