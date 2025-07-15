#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    
    # 声明启动参数
    yolo11_model_path_arg = DeclareLaunchArgument(
        'yolo11_model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('rknn_yolo11_ros2'),
            'model',
            'cloth.rknn'
        ]),
        description='YOLO11服装检测模型文件路径'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='相机名称前缀'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='输入图像话题'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='检测置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.5',
        description='NMS阈值'
    )
    
    enable_debug_display_arg = DeclareLaunchArgument(
        'enable_debug_display',
        default_value='true',
        description='是否启用OpenCV调试显示'
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

    # 集成人员检测节点
    integrated_person_detection_node = Node(
        package='rknn_yolo11_ros2',
        executable='integrated_person_detection_node',
        name='integrated_person_detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('yolo11_model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'person_topic': '/person_detection/person_positions',
            'distance_query_topic': '/depth_reader/get_depth_at',
            'distance_result_topic': '/depth_reader/depth_value',
            'debug_image_topic': '/integrated_person/debug_image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_display': LaunchConfiguration('enable_debug_display'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # 启动参数
        yolo11_model_path_arg,
        camera_name_arg,
        input_topic_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_display_arg,
        use_depth_service_arg,
        
        # 信息输出
        LogInfo(msg='启动整合人员检测系统'),
        LogInfo(msg=['相机名称: ', LaunchConfiguration('camera_name')]),
        LogInfo(msg=['输入话题: ', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['YOLO11模型: ', LaunchConfiguration('yolo11_model_path')]),
        LogInfo(msg=['置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        LogInfo(msg='功能说明:'),
        LogInfo(msg='  - YOLO11服装检测（上衣、下装）'),
        LogInfo(msg='  - 服装匹配和人体位置确定'),
        LogInfo(msg='  - 距离查询和测量'),
        LogInfo(msg='  - OpenCV调试图像显示'),
        LogInfo(msg='  - 按q或ESC退出'),
        
        # 节点启动顺序
        astra_camera_node,
        depth_service_node,
        integrated_person_detection_node,
    ]) 