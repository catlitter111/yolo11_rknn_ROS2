#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    
    # RKNN YOLO11 节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': '/stereo_camera/left/image_rectified',
            'output_topic': '/detections',
            'debug_image_topic': '/debug_image',
            'confidence_threshold': 0.25,
            'nms_threshold': 0.45,
            'enable_debug_image': True,
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        rknn_yolo11_node,
    ])