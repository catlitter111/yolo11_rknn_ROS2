#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 声明摄像头参数
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video1',
        description='Camera device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    # USB摄像头节点 (使用usb_cam作为替代)
    usb_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_frame_id': 'camera_link',
            'pixel_format': 'mjpeg',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ]
    )
    
    # 包含RKNN YOLO11 launch文件
    rknn_yolo11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rknn_yolo11_ros2'),
                'launch',
                'rknn_yolo11.launch.py'
            ])
        ]),
        launch_arguments={
            'input_topic': '/camera/image_raw',
        }.items()
    )
    
    return LaunchDescription([
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_camera_node,
        rknn_yolo11_launch,
    ])