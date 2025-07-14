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
        description='RKNN模型文件路径'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目相机设备ID'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='检测置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.55',
        description='NMS阈值'
    )
    
    enable_debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='true',
        description='是否启用YOLO11调试图像'
    )
    
    # 双目相机节点
    stereo_camera_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_camera_ros2_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': 1280,
            'frame_height': 480,
            'process_width': 640,
            'process_height': 480,
            'publish_rate': 30.0,
            'camera_frame_id': 'stereo_camera',
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
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_image': LaunchConfiguration('enable_debug_image'),
        }]
    )
    
    # 智能显示节点 - 显示YOLO检测结果并叠加距离信息
    intelligent_display_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_display_node',
        name='intelligent_display_node',
        output='screen',
        parameters=[{
            'input_topic': '/stereo/left/image_raw',
            'detection_topic': '/detections',
            'distance_service': '/stereo/get_distance',
            'window_name': 'YOLO11 + 距离检测',
            'enable_distance': True,
            'enable_debug': True,
            'font_scale': 0.7,
            'line_thickness': 2,
        }]
    )
    
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        camera_id_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_image_arg,
        
        # 信息输出
        LogInfo(msg='启动YOLO11+距离检测系统'),
        LogInfo(msg=['相机ID: ', LaunchConfiguration('camera_id')]),
        LogInfo(msg=['YOLO11模型: ', LaunchConfiguration('model_path')]),
        LogInfo(msg=['置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        LogInfo(msg='功能说明:'),
        LogInfo(msg='  - 实时目标检测（YOLO11）'),
        LogInfo(msg='  - 距离测量（双目立体视觉）'),
        LogInfo(msg='  - 检测框+距离信息显示'),
        LogInfo(msg='  - 按q或ESC退出'),
        
        # 节点启动顺序
        stereo_camera_node,
        rknn_yolo11_node,
        intelligent_display_node,
    ]) 