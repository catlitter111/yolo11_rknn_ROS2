#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

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
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='相机名称前缀'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.45',
        description='检测置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.55',
        description='NMS阈值'
    )
    
    enable_debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image',
        default_value='false',
        description='是否启用YOLO11调试图像'
    )
    
    use_depth_service_arg = DeclareLaunchArgument(
        'use_depth_service',
        default_value='true',
        description='是否启用深度服务节点'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    
    # Astra相机节点
    astra_camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='astra_camera_node',
        namespace=camera_name,
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'serial_number': 'ACRD233006M',
            'vendor_id': '0x2bc5',
            'product_id': '0x050f',
            'enable_depth': True,
            'enable_color': True,
            'enable_ir': False,
            'enable_point_cloud': False,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'use_uvc_camera': True,
            'uvc_vendor_id': 0x2bc5,
            'uvc_product_id': 0x050f,
            'uvc_camera_format': 'mjpeg',
            'publish_tf': True,
            'tf_publish_rate': 10.0,
            'connection_delay': 100,
        }]
    )
    
    # 深度服务节点
    depth_service_node = Node(
        package='astra_depth_reader',
        executable='depth_service',
        name='depth_service_node',
        output='screen',
        remappings=[
            ('/camera/color/image_raw', [camera_name, '/color/image_raw']),
            ('/camera/depth/image_raw', [camera_name, '/depth/image_raw']),
            ('/camera/depth/camera_info', [camera_name, '/depth/camera_info']),
        ],
        condition=IfCondition(LaunchConfiguration('use_depth_service'))
    )

    # RKNN YOLO11 目标检测节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': '/camera/color/image_raw',
            'output_topic': '/detections',
            'debug_image_topic': '/yolo_debug_image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_image': LaunchConfiguration('enable_debug_image'),
            # 🔧 优化19：通信优化参数
            'use_intra_process_comms': True,  # 启用进程内通信
            'enable_zero_copy': True,         # 启用零拷贝优化
        }],
        # 🔧 优化20：ROS2执行器优化
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )
    
    # 智能显示节点 - 显示YOLO检测结果并叠加距离信息
    intelligent_display_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_display_node',
        name='intelligent_display_node',
        output='screen',
        parameters=[{
            'input_topic': '/camera/color/image_raw',
            'detection_topic': '/detections',
            'distance_request_topic': '/depth_reader/get_depth_at',
            'distance_response_topic': '/depth_reader/depth_value',
            'window_name': 'YOLO11 + 距离检测 (优化版)',
            'enable_distance': True,
            'enable_debug': True,
            'font_scale': 0.7,
            'line_thickness': 2,
            # 🔧 优化21：显示优化参数
            'display_fps': 30,               # 显示帧率
            'max_detection_age': 5.0,        # 最大检测信息保留时间
            'distance_cache_size': 100,      # 距离缓存大小
        }],
        # 🔧 优化22：显示节点优化
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        camera_name_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_image_arg,
        use_depth_service_arg,
        
        # 信息输出
        LogInfo(msg='启动YOLO11+距离检测系统（使用Astra相机）'),
        LogInfo(msg=['相机名称: ', LaunchConfiguration('camera_name')]),
        LogInfo(msg=['YOLO11模型: ', LaunchConfiguration('model_path')]),
        LogInfo(msg=['置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        LogInfo(msg='功能说明:'),
        LogInfo(msg='  - 实时目标检测（YOLO11）'),
        LogInfo(msg='  - 距离测量（Astra深度相机）'),
        LogInfo(msg='  - 检测框+距离信息显示'),
        LogInfo(msg='  - 按q或ESC退出'),
        
        # 节点启动顺序
        astra_camera_node,
        depth_service_node,
        rknn_yolo11_node,
        intelligent_display_node,
    ]) 