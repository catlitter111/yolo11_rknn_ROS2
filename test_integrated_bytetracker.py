#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
整合人员检测和跟踪系统测试脚本
==============================

测试功能：
1. 验证integrated_person_detection_node是否正确发布人员位置数据
2. 验证bytetracker_node是否正确订阅并处理数据
3. 验证数据格式转换是否正确
4. 显示实时跟踪结果

使用方法：
    python3 test_integrated_bytetracker.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from custom_msgs.msg import TrackedPersonArray, TrackingResult
import json
import time
import threading
from collections import defaultdict

class IntegratedByteetrackerTester(Node):
    """整合人员检测和跟踪系统测试节点"""
    
    def __init__(self):
        super().__init__('integrated_bytetracker_tester')
        
        # 配置QoS
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅integrated_person_detection_node的输出
        self.person_positions_sub = self.create_subscription(
            String, '/person_detection/person_positions',
            self.person_positions_callback, qos_best_effort)
        
        # 订阅bytetracker_node的输出
        self.tracked_persons_sub = self.create_subscription(
            TrackedPersonArray, '/bytetracker/tracked_persons',
            self.tracked_persons_callback, qos_reliable)
        
        self.tracking_result_sub = self.create_subscription(
            TrackingResult, '/bytetracker/tracking_result',
            self.tracking_result_callback, qos_reliable)
        
        # 数据统计
        self.detection_count = 0
        self.tracking_count = 0
        self.last_detection_time = 0
        self.last_tracking_time = 0
        self.person_data_cache = {}
        
        # 创建定时器显示统计信息
        self.stats_timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('🔍 整合人员检测和跟踪系统测试节点已启动')
        self.get_logger().info('📊 监控以下话题：')
        self.get_logger().info('   - /person_detection/person_positions (检测数据)')
        self.get_logger().info('   - /bytetracker/tracked_persons (跟踪结果)')
        self.get_logger().info('   - /bytetracker/tracking_result (跟踪状态)')
    
    def person_positions_callback(self, msg):
        """处理人员检测数据"""
        self.detection_count += 1
        self.last_detection_time = time.time()
        
        try:
            # 解析JSON数据
            data = json.loads(msg.data)
            person_count = data.get('person_count', 0)
            
            self.get_logger().info(f'📍 检测数据 #{self.detection_count}: 发现 {person_count} 个人员')
            
            if 'persons' in data:
                for i, person in enumerate(data['persons']):
                    person_id = person.get('id', f'unknown_{i}')
                    bbox = person.get('bbox', [])
                    
                    # 提取服装信息
                    clothing_info = []
                    if 'clothing' in person:
                        clothing = person['clothing']
                        if 'upper' in clothing:
                            upper = clothing['upper']
                            color_rgb = upper.get('color_rgb', [128, 128, 128])
                            clothing_info.append(f"上衣: RGB{color_rgb}(置信度: {upper.get('confidence', 0):.2f})")
                        if 'lower' in clothing:
                            lower = clothing['lower']
                            color_rgb = lower.get('color_rgb', [128, 128, 128])
                            clothing_info.append(f"下装: RGB{color_rgb}(置信度: {lower.get('confidence', 0):.2f})")
                    
                    # 身体比例信息
                    body_ratios_count = 0
                    if 'body_ratios' in person and person['body_ratios']:
                        body_ratios_count = sum(1 for ratio in person['body_ratios'] if ratio > 0)
                    
                    self.get_logger().info(f'   👤 {person_id}: 边界框={bbox}, 服装={clothing_info}, 身体比例={body_ratios_count}/16')
                    
                    # 缓存人员数据以便与跟踪结果对比
                    self.person_data_cache[person_id] = {
                        'detection_time': time.time(),
                        'bbox': bbox,
                        'clothing': clothing_info,
                        'body_ratios_count': body_ratios_count
                    }
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ 处理检测数据错误: {e}')
    
    def tracked_persons_callback(self, msg):
        """处理跟踪结果"""
        self.tracking_count += 1
        self.last_tracking_time = time.time()
        
        person_count = len(msg.persons)
        self.get_logger().info(f'🎯 跟踪结果 #{self.tracking_count}: 跟踪 {person_count} 个目标')
        
        for person in msg.persons:
            track_id = person.track_id
            confidence = person.confidence
            is_target = person.is_target
            
            # 提取颜色信息
            upper_color = tuple(person.upper_color) if person.upper_color else None
            lower_color = tuple(person.lower_color) if person.lower_color else None
            
            # 身体比例数量
            body_ratios_count = len([r for r in person.body_ratios if r > 0]) if person.body_ratios else 0
            
            status_indicator = "🎯" if is_target else "👤"
            
            self.get_logger().info(f'   {status_indicator} Track ID {track_id}: '
                                 f'置信度={confidence:.2f}, '
                                 f'上衣颜色={upper_color}, 下装颜色={lower_color}, '
                                 f'身体比例={body_ratios_count}/16')
    
    def tracking_result_callback(self, msg):
        """处理跟踪状态信息"""
        mode = msg.mode
        total_tracks = msg.total_tracks
        target_detected = msg.target_detected
        tracking_status = msg.tracking_status
        fps = msg.fps
        
        self.get_logger().info(f'📈 跟踪状态: 模式={mode}, 目标数={total_tracks}, '
                             f'检测到目标={target_detected}, 状态={tracking_status}, FPS={fps:.1f}')
        
        if target_detected:
            self.get_logger().info(f'   🎯 目标位置: ({msg.target_x:.1f}, {msg.target_y:.1f}), '
                                 f'尺寸: {msg.target_width:.1f}x{msg.target_height:.1f}')
    
    def print_statistics(self):
        """打印统计信息"""
        current_time = time.time()
        
        # 计算消息频率
        detection_active = (current_time - self.last_detection_time) < 5.0 if self.last_detection_time > 0 else False
        tracking_active = (current_time - self.last_tracking_time) < 5.0 if self.last_tracking_time > 0 else False
        
        # 清理过期的缓存数据
        expired_keys = [k for k, v in self.person_data_cache.items() 
                       if current_time - v['detection_time'] > 10.0]
        for key in expired_keys:
            del self.person_data_cache[key]
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📊 系统统计 (运行时间: {current_time - self.start_time:.1f}s)')
        self.get_logger().info(f'   📍 检测消息: {self.detection_count} 条 {"✅" if detection_active else "❌"}')
        self.get_logger().info(f'   🎯 跟踪消息: {self.tracking_count} 条 {"✅" if tracking_active else "❌"}')
        self.get_logger().info(f'   💾 缓存人员: {len(self.person_data_cache)} 个')
        
        if detection_active and tracking_active:
            self.get_logger().info('   ✅ 系统运行正常 - 检测和跟踪数据都在更新')
        elif detection_active and not tracking_active:
            self.get_logger().warn('   ⚠️  只有检测数据，跟踪节点可能有问题')
        elif not detection_active and tracking_active:
            self.get_logger().warn('   ⚠️  只有跟踪数据，检测节点可能有问题')
        else:
            self.get_logger().error('   ❌ 检测和跟踪数据都不活跃，系统可能有问题')
        
        self.get_logger().info('=' * 60)
    
    def run(self):
        """运行测试"""
        self.start_time = time.time()
        self.get_logger().info('🚀 开始监控系统数据流...')
        self.get_logger().info('💡 提示: 确保相机和检测节点正在运行')
        self.get_logger().info('⏹️  按 Ctrl+C 停止测试')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    tester = IntegratedByteetrackerTester()
    tester.run()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 用户中断测试')
    except Exception as e:
        tester.get_logger().error(f'❌ 测试过程中发生错误: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print('\n✅ 测试完成')

if __name__ == '__main__':
    main() 