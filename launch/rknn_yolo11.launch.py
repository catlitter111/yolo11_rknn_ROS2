#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('rknn_yolo11_ros2'),
            'model',
            'cloth.rknn'
        ]),
        description='Path to RKNN model file'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/detections',
        description='Output detection topic'
    )
    
    debug_image_topic_arg = DeclareLaunchArgument(
        'debug_image_topic',
        default_value='/debug_image',
        description='Debug image output topic'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Confidence threshold for detections'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS threshold for detections'
    )
    
    enable_debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='true',
        description='Enable debug image output'
    )
    
    # RKNN YOLO11 节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'debug_image_topic': LaunchConfiguration('debug_image_topic'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_image': LaunchConfiguration('enable_debug_image'),
        }],
        remappings=[
            ('~/image_raw', LaunchConfiguration('input_topic')),
            ('~/detections', LaunchConfiguration('output_topic')),
            ('~/debug_image', LaunchConfiguration('debug_image_topic')),
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        input_topic_arg,
        output_topic_arg,
        debug_image_topic_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_image_arg,
        rknn_yolo11_node,
    ])