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
            
        self.timer = self.create_timer(0.5, self.publish_image)  # 2Hz，降低频率
        self.get_logger().info("静态图像发布器已启动，发布到 /camera/color/image_raw")
        
    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.publisher.publish(msg)
        self.get_logger().info(f"发布图像: {self.image.shape[1]}x{self.image.shape[0]}")

def main():
    rclpy.init()
    node = StaticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("静态图像发布器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 