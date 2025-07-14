#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目相机设备ID'
    )
    
    enable_display_arg = DeclareLaunchArgument(
        'enable_display',
        default_value='true',
        description='是否启用双目相机显示'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='检测置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS阈值'
    )
    
    enable_debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='true',
        description='是否启用YOLO11调试图像'
    )
    
    # 启动双目相机节点
    stereo_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_camera_cpp'),
                'launch',
                'stereo_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_id': LaunchConfiguration('camera_id'),
            'enable_display': LaunchConfiguration('enable_display'),
            'frame_width': '1280',
            'frame_height': '480',
            'process_width': '640',
            'process_height': '480',
            'publish_rate': '30.0',
        }.items()
    )
    
    # RKNN YOLO11 目标检测节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': '/stereo_camera/left/image_rectified',
            'output_topic': '/detections',
            'debug_image_topic': '/yolo_debug_image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_image': LaunchConfiguration('enable_debug_image'),
        }]
    )
    
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        camera_id_arg,
        enable_display_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_image_arg,
        
        # 信息输出
        LogInfo(msg=['启动双目相机YOLO11检测系统']),
        LogInfo(msg=['双目相机ID: ', LaunchConfiguration('camera_id')]),
        LogInfo(msg=['YOLO11模型路径: ', LaunchConfiguration('model_path')]),
        LogInfo(msg=['检测置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        
        # 启动双目相机
        stereo_camera_launch,
        
        # 启动YOLO11检测节点
        rknn_yolo11_node,
    ])