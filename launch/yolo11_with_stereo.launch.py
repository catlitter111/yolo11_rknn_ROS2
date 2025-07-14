#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
    
    # 双目相机节点（直接定义，避免包依赖问题）
    stereo_camera_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_camera_ros2_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'camera_id': 1,
            'frame_width': 1280,
            'frame_height': 480,
            'process_width': 640,
            'process_height': 480,
            'publish_rate': 30.0,
            'camera_frame_id': 'stereo_camera',
        }]
    )
    
    # 双目显示节点（可选）
    stereo_display_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_display_node',
        name='stereo_display_node',
        output='screen',
        parameters=[{
            'window_name': 'Stereo Camera - Left View',
            'fps_buffer_size': 30,
            'font_scale': 0.7,
            'font_thickness': 2,
        }]
    )
    
    # RKNN YOLO11 目标检测节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': '/stereo/left/image_raw',
            'output_topic': '/detections',
            'debug_image_topic': '/yolo_debug_image',
            'confidence_threshold': 0.25,
            'nms_threshold': 0.45,
            'enable_debug_image': True,
        }]
    )
    
    # 图像显示节点
    image_display_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_display_node',
        name='image_display_node',
        output='screen',
        parameters=[{
            'input_topic': '/stereo/left/image_raw',
            'window_name': 'Stereo Camera - Left View',
            'enable_debug': True,
        }]
    )
    
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        
        # 信息输出
        LogInfo(msg='启动双目相机YOLO11检测系统'),
        LogInfo(msg=['YOLO11模型路径: ', LaunchConfiguration('model_path')]),
        
        # 节点
        stereo_camera_node,
        stereo_display_node,
        rknn_yolo11_node,
    ])