# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 package that integrates RKNN (Rockchip Neural Network) YOLO11 object detection with ROS2 Humble. The package is designed to run on Rockchip platforms (RK3588, RK3566, RV1106, RV1103) and provides real-time object detection capabilities using RKNN-optimized models.

## Core Architecture

The package follows a modular ROS2 architecture:

- **Main Node**: `RknnYolo11Node` - Core detection node that processes image streams
- **Display Node**: Separate node for image visualization and debugging
- **RKNN Integration**: Direct integration with RKNN Runtime library for hardware acceleration
- **Stereo Camera Support**: Integrated with `stereo_camera_cpp` package for stereo vision
- **Component-based Design**: Supports both standalone and composable node deployment

### Key Components

1. **RKNN Wrapper** (`yolo11.cc`): Handles RKNN model loading and inference
2. **Post-processing** (`postprocess.cc`): Implements YOLO11 detection result processing
3. **ROS2 Node** (`rknn_yolo11_node.cpp`): Bridges RKNN inference with ROS2 message system
4. **Utility Functions** (`src/utils/`): Simplified image processing utilities

## Build Commands

```bash
# Build the package
colcon build --packages-select rknn_yolo11_ros2

# Build with verbose output for debugging
colcon build --packages-select rknn_yolo11_ros2 --event-handlers console_direct+

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select rknn_yolo11_ros2
```

## Running the System

### Complete system with stereo camera
```bash
source install/setup.bash
ros2 launch rknn_yolo11_ros2 yolo11_with_stereo.launch.py
```

### YOLO11 detection only
```bash
ros2 launch rknn_yolo11_ros2 rknn_yolo11_simple.launch.py model_path:=/path/to/model.rknn
```

### Manual node execution
```bash
ros2 run rknn_yolo11_ros2 rknn_yolo11_ros2_node --ros-args --params-file config/rknn_yolo11_params.yaml
```

## Testing and Debugging

### Check detection output
```bash
ros2 topic echo /detections
```

### Monitor image topics
```bash
ros2 run rqt_image_view rqt_image_view /yolo_debug_image
ros2 run rqt_image_view rqt_image_view /stereo_camera/left/image_rectified
```

### Debug mode
```bash
ros2 run rknn_yolo11_ros2 rknn_yolo11_ros2_node --ros-args --log-level debug
```

## Configuration Management

- **Main config**: `config/rknn_yolo11_params.yaml`
- **Model path**: Update `model_path` parameter to point to your RKNN model
- **Camera topics**: Default expects `/stereo_camera/left/image_rectified` from stereo camera package
- **Detection thresholds**: Adjust `confidence_threshold` and `nms_threshold` for detection sensitivity

## Key Dependencies

- **RKNN Runtime**: Must be installed on target platform (`librknnrt`)
- **OpenCV**: For image processing
- **ROS2 Humble**: Core framework
- **Stereo Camera Package**: `stereo_camera_cpp` for camera input

## Platform-Specific Notes

### RKNN Library Path
The CMakeLists.txt expects RKNN Runtime at `/usr/lib` and `/usr/include`. Modify these paths if your installation differs:

```cmake
set(RKNN_RT_LIB_PATH "/usr/lib")
set(RKNN_RT_INCLUDE_PATH "/usr/include")
```

### Simplified Dependencies
The build system automatically uses simplified utility functions if third-party dependencies are not available:

```cmake
if(NOT EXISTS ${THIRD_PARTY_PATH})
    set(USE_SIMPLIFIED_DEPS TRUE)
endif()
```

## Model and Dataset

- **Default model**: `cloth.rknn` (13 clothing classes)
- **Label files**: `clothing_13_labels_list.txt`, `coco_80_labels_list.txt`
- **Test image**: `bus.jpg` for validation
- **Model download**: Use `model/download_model.sh` script

## Common Issues

1. **Model loading failures**: Verify RKNN Runtime installation and model path
2. **Camera access**: Check `/dev/video0` permissions and camera connection
3. **Performance**: Adjust image resolution and detection thresholds for optimal FPS
4. **Memory**: Monitor memory usage on resource-constrained platforms

## Message Interfaces

- **Input**: `sensor_msgs/Image` from stereo camera
- **Output**: `vision_msgs/Detection2DArray` with detection results
- **Debug**: `sensor_msgs/Image` with visualization overlay

## Component Registration

The package supports ROS2 composition through `rclcpp_components_register_nodes(${PROJECT_NAME}_component "RknnYolo11Node")` for efficient resource usage in multi-node systems.