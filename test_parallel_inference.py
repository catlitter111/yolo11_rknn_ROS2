#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys
import statistics

class ParallelInferenceTest(Node):
    def __init__(self):
        super().__init__('parallel_inference_test')
        
        # 订阅人员位置话题以监控性能
        self.position_sub = self.create_subscription(
            String,
            '/person_detection/person_positions',
            self.position_callback,
            10
        )
        
        # 发布模式切换命令
        self.mode_pub = self.create_publisher(
            String,
            '/integrated_person/detection_mode',
            10
        )
        
        # 性能监控数据
        self.frame_times = []
        self.last_timestamp = None
        self.mode_times = {
            'PARTIAL': [],
            'FULL': []
        }
        self.current_mode = None
        self.frame_count = 0
        
        self.get_logger().info('🚀 并行推理性能测试节点已启动')
        self.get_logger().info('开始性能测试...')
        
        # 延迟启动测试序列
        self.test_timer = self.create_timer(2.0, self.start_test_sequence)
        
    def position_callback(self, msg):
        """处理人员位置数据，计算性能指标"""
        try:
            data = json.loads(msg.data)
            current_timestamp = data.get('timestamp', 0)
            detection_mode = data.get('detection_mode', 'UNKNOWN')
            person_count = data.get('person_count', 0)
            
            # 计算帧间隔
            if self.last_timestamp is not None:
                frame_interval = current_timestamp - self.last_timestamp
                self.frame_times.append(frame_interval)
                
                # 记录不同模式下的性能
                if detection_mode in self.mode_times:
                    self.mode_times[detection_mode].append(frame_interval)
                
                self.frame_count += 1
                
                # 实时性能显示
                if self.frame_count % 10 == 0:
                    recent_times = self.frame_times[-10:]
                    avg_interval = statistics.mean(recent_times)
                    fps = 1000.0 / avg_interval if avg_interval > 0 else 0
                    
                    self.get_logger().info(
                        f'📊 模式: {detection_mode}, 人数: {person_count}, '
                        f'平均间隔: {avg_interval:.1f}ms, FPS: {fps:.1f}'
                    )
            
            self.last_timestamp = current_timestamp
            self.current_mode = detection_mode
            
        except Exception as e:
            self.get_logger().error(f'处理位置数据时出错: {e}')
    
    def send_mode_command(self, mode):
        """发送模式切换命令"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        
        mode_name = "完整检测(并行推理)" if mode == "FULL" else "部分检测(级联推理)"
        self.get_logger().info(f'🔄 切换到 {mode_name} 模式')
    
    def start_test_sequence(self):
        """开始测试序列"""
        self.test_timer.cancel()  # 取消启动定时器
        
        # 测试序列：部分模式 -> 完整模式 -> 部分模式
        test_sequence = [
            ("PARTIAL", "部分检测(级联推理)", 15),  # 15秒部分检测
            ("FULL", "完整检测(并行推理)", 20),    # 20秒并行推理
            ("PARTIAL", "部分检测(级联推理)", 10), # 10秒部分检测对比
            ("FULL", "完整检测(并行推理)", 15),    # 15秒并行推理确认
        ]
        
        self.get_logger().info('🧪 开始自动性能测试序列...')
        self.run_test_sequence(test_sequence, 0)
    
    def run_test_sequence(self, sequence, index):
        """运行测试序列"""
        if index >= len(sequence):
            # 测试完成，显示结果
            self.show_performance_results()
            return
        
        mode, mode_name, duration = sequence[index]
        
        # 清理当前模式的历史数据（保留至少10个样本用于预热）
        if mode in self.mode_times and len(self.mode_times[mode]) > 10:
            self.mode_times[mode] = self.mode_times[mode][-10:]
        
        self.get_logger().info(f'📋 测试阶段 {index+1}/{len(sequence)}: {mode_name}, 持续 {duration} 秒')
        self.send_mode_command(mode)
        
        # 延迟2秒让模式切换生效，然后开始下一阶段
        self.create_timer(duration, lambda: self.run_test_sequence(sequence, index + 1))
    
    def show_performance_results(self):
        """显示性能测试结果"""
        self.get_logger().info('📈 =============== 性能测试结果 ===============')
        
        for mode, times in self.mode_times.items():
            if len(times) < 5:  # 数据不足
                continue
                
            # 去除前5个样本（预热）
            valid_times = times[5:] if len(times) > 5 else times
            
            if not valid_times:
                continue
                
            avg_interval = statistics.mean(valid_times)
            min_interval = min(valid_times)
            max_interval = max(valid_times)
            std_interval = statistics.stdev(valid_times) if len(valid_times) > 1 else 0
            
            avg_fps = 1000.0 / avg_interval if avg_interval > 0 else 0
            max_fps = 1000.0 / min_interval if min_interval > 0 else 0
            min_fps = 1000.0 / max_interval if max_interval > 0 else 0
            
            mode_name = "完整检测(并行推理)" if mode == "FULL" else "部分检测(级联推理)"
            
            self.get_logger().info(f'🎯 {mode_name}:')
            self.get_logger().info(f'   平均帧间隔: {avg_interval:.1f}ms (±{std_interval:.1f}ms)')
            self.get_logger().info(f'   平均FPS: {avg_fps:.1f} Hz')
            self.get_logger().info(f'   FPS范围: {min_fps:.1f} - {max_fps:.1f} Hz')
            self.get_logger().info(f'   样本数量: {len(valid_times)}')
            
        # 计算性能提升
        if len(self.mode_times['PARTIAL']) > 5 and len(self.mode_times['FULL']) > 5:
            partial_avg = statistics.mean(self.mode_times['PARTIAL'][5:])
            full_avg = statistics.mean(self.mode_times['FULL'][5:])
            
            if partial_avg > 0 and full_avg > 0:
                latency_improvement = ((partial_avg - full_avg) / partial_avg) * 100
                fps_improvement = ((1000/full_avg) - (1000/partial_avg)) / (1000/partial_avg) * 100
                
                self.get_logger().info('🚀 并行推理性能提升:')
                self.get_logger().info(f'   延迟降低: {latency_improvement:.1f}%')
                self.get_logger().info(f'   FPS提升: {fps_improvement:.1f}%')
                
                if latency_improvement > 20:
                    self.get_logger().info('✅ 并行推理显著提升性能！')
                elif latency_improvement > 10:
                    self.get_logger().info('✅ 并行推理有效提升性能')
                else:
                    self.get_logger().info('⚠️  并行推理效果有限，可能需要进一步优化')
        
        self.get_logger().info('===============================================')
        
        # 继续监控
        self.get_logger().info('继续实时监控性能... (按Ctrl+C退出)')

def main():
    rclpy.init()
    
    test_node = ParallelInferenceTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('测试被用户中断')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 