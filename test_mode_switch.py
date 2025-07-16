#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')
        
        # 创建模式切换发布者
        self.mode_pub = self.create_publisher(
            String, 
            '/integrated_person/detection_mode', 
            10
        )
        
        # 订阅人员位置话题以查看模式状态
        self.person_sub = self.create_subscription(
            String,
            '/person_detection/person_positions',
            self.person_callback,
            10
        )
        
        self.get_logger().info('模式控制器节点已启动')
        self.get_logger().info('支持的命令:')
        self.get_logger().info('  FULL - 切换到完整检测模式 (服装+关键点+身体比例)')
        self.get_logger().info('  PARTIAL - 切换到部分检测模式 (仅服装检测)')
        self.get_logger().info('  AUTO - 自动测试模式切换')
        self.get_logger().info('  STATUS - 查看当前模式状态')
        
        self.current_mode = "未知"
        
    def person_callback(self, msg):
        """接收人员位置信息，提取检测模式状态"""
        try:
            import json
            data = json.loads(msg.data)
            if 'detection_mode' in data:
                new_mode = data['detection_mode']
                if new_mode != self.current_mode:
                    self.current_mode = new_mode
                    self.get_logger().info(f'当前检测模式: {new_mode}')
        except Exception as e:
            pass  # 忽略JSON解析错误
    
    def send_mode_command(self, mode):
        """发送模式切换命令"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        
        mode_name = "完整检测" if mode == "FULL" else "部分检测"
        self.get_logger().info(f'发送模式切换命令: {mode} ({mode_name})')
    
    def auto_test(self):
        """自动测试模式切换"""
        self.get_logger().info('开始自动模式切换测试...')
        
        # 测试序列
        test_sequence = [
            ("FULL", "完整检测", 10),
            ("PARTIAL", "部分检测", 8),
            ("FULL", "完整检测", 8),
            ("PARTIAL", "部分检测", 6),
            ("FULL", "完整检测", 5)
        ]
        
        for mode, mode_name, duration in test_sequence:
            self.get_logger().info(f'切换到 {mode_name} 模式，持续 {duration} 秒')
            self.send_mode_command(mode)
            time.sleep(duration)
        
        self.get_logger().info('自动测试完成')

def main():
    rclpy.init()
    
    controller = ModeController()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].upper()
        
        if command == "FULL":
            controller.send_mode_command("FULL")
            controller.get_logger().info('已发送完整检测模式命令')
            
        elif command == "PARTIAL":
            controller.send_mode_command("PARTIAL")
            controller.get_logger().info('已发送部分检测模式命令')
            
        elif command == "AUTO":
            # 在单独的线程中运行自动测试
            import threading
            test_thread = threading.Thread(target=controller.auto_test)
            test_thread.start()
            
            # 运行节点以接收状态更新
            try:
                rclpy.spin(controller)
            except KeyboardInterrupt:
                controller.get_logger().info('测试被用户中断')
                
        elif command == "STATUS":
            controller.get_logger().info('监听模式状态中... (按Ctrl+C退出)')
            try:
                rclpy.spin(controller)
            except KeyboardInterrupt:
                controller.get_logger().info('状态监听结束')
                
        else:
            controller.get_logger().error(f'未知命令: {command}')
            controller.get_logger().info('支持的命令: FULL, PARTIAL, AUTO, STATUS')
    else:
        # 交互模式
        controller.get_logger().info('进入交互模式，输入命令：')
        
        def input_thread():
            while rclpy.ok():
                try:
                    command = input('请输入命令 (FULL/PARTIAL/AUTO/STATUS/QUIT): ').strip().upper()
                    if command == 'QUIT':
                        break
                    elif command == 'FULL':
                        controller.send_mode_command("FULL")
                    elif command == 'PARTIAL':
                        controller.send_mode_command("PARTIAL")
                    elif command == 'AUTO':
                        controller.auto_test()
                    elif command == 'STATUS':
                        controller.get_logger().info(f'当前模式: {controller.current_mode}')
                    else:
                        print('无效命令，请输入: FULL, PARTIAL, AUTO, STATUS, 或 QUIT')
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
        
        import threading
        thread = threading.Thread(target=input_thread)
        thread.daemon = True
        thread.start()
        
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            controller.get_logger().info('交互模式结束')
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 