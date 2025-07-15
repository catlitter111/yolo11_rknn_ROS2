import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    package_name = 'rknn_yolo11_ros2'
    package_share_directory = get_package_share_directory(package_name)
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(package_share_directory, 'model', 'yolov8_pose.rknn'),
        description='Path to the YOLOv8 pose model file'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_directory, 'config', 'rknn_yolov8_pose_params.yaml'),
        description='Path to the configuration file'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='相机名称前缀'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/yolov8_pose/image',
        description='Output image topic'
    )
    
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/yolov8_pose/detections',
        description='Detection results topic'
    )
    
    show_fps_arg = DeclareLaunchArgument(
        'show_fps',
        default_value='true',
        description='Show FPS on output image'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization'
    )
    
    enable_display_arg = DeclareLaunchArgument(
        'enable_display',
        default_value='true',
        description='Enable OpenCV display window'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='启用Astra相机节点'
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
        }],
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    # YOLOv8 pose 检测节点
    yolov8_pose_node = Node(
        package=package_name,
        executable='rknn_yolov8_pose_node',
        name='rknn_yolov8_pose_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'show_fps': LaunchConfiguration('show_fps'),
                'enable_visualization': LaunchConfiguration('enable_visualization'),
            }
        ],
        emulate_tty=True,
    )
    
    # 显示节点
    display_node = Node(
        package=package_name,
        executable='pose_display_node',
        name='pose_display_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('output_topic'),
                'enable_display': LaunchConfiguration('enable_display'),
            }
        ],
        emulate_tty=True,
    )
    
    # RViz配置文件路径
    rviz_config_file = os.path.join(package_share_directory, 'rviz', 'yolov8_pose.rviz')
    
    # RViz节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    
    # 返回LaunchDescription
    return LaunchDescription([
        # 启动参数
        model_path_arg,
        config_file_arg,
        camera_name_arg,
        input_topic_arg,
        output_topic_arg,
        detection_topic_arg,
        show_fps_arg,
        enable_visualization_arg,
        enable_display_arg,
        use_rviz_arg,
        use_camera_arg,
        
        # 信息输出
        LogInfo(msg='启动YOLOv8姿态检测系统（使用Astra相机）'),
        LogInfo(msg=['相机名称: ', LaunchConfiguration('camera_name')]),
        LogInfo(msg=['YOLOv8模型: ', LaunchConfiguration('model_path')]),
        LogInfo(msg=['输入话题: ', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['输出话题: ', LaunchConfiguration('output_topic')]),
        LogInfo(msg='功能说明:'),
        LogInfo(msg='  - 实时人体姿态检测（YOLOv8 Pose）'),
        LogInfo(msg='  - 17个关键点检测'),
        LogInfo(msg='  - 骨架连接线可视化'),
        LogInfo(msg='  - 按q或ESC退出'),
        
        # 节点启动
        astra_camera_node,
        yolov8_pose_node,
        display_node,
        rviz_node,
    ]) 