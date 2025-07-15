cmake_minimum_required(VERSION 3.8)
project(rknn_yolo11_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(stereo_camera_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# 设置RKNN相关路径 - 需要根据实际安装路径调整
set(RKNN_RT_LIB_PATH "/usr/lib")
set(RKNN_RT_INCLUDE_PATH "/usr/include")

# 检查RKNN库是否存在
find_library(RKNN_RT_LIB rknnrt PATHS ${RKNN_RT_LIB_PATH})
if(NOT RKNN_RT_LIB)
    message(WARNING "RKNN Runtime library not found! Please install RKNN Runtime or set correct path.")
    set(RKNN_RT_LIB "rknnrt")  # 假设系统能找到
endif()

# 添加第三方依赖路径（需要适配）
set(THIRD_PARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty")
set(UTILS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../utils")

# 如果第三方库路径不存在，我们需要创建简化版本
if(NOT EXISTS ${THIRD_PARTY_PATH})
    message(STATUS "Third party path not found, using simplified implementation")
    set(USE_SIMPLIFIED_DEPS TRUE)
endif()

# 头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKNN_RT_INCLUDE_PATH}
    ${OpenCV_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# 如果使用简化依赖
if(USE_SIMPLIFIED_DEPS)
    # 创建简化的工具函数
    set(SIMPLIFIED_UTILS_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/image_utils_simple.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/file_utils_simple.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/image_drawing_simple.cpp
    )
else()
    # 使用原始第三方库
    include_directories(
        ${THIRD_PARTY_PATH}
        ${UTILS_PATH}
    )
endif()

# 源文件
set(RKNN_SRCS
    src/rknn_yolo11_node.cpp
    src/postprocess.cc
    src/yolo11.cc
    src/common_utils.cpp
)

if(USE_SIMPLIFIED_DEPS)
    list(APPEND RKNN_SRCS ${SIMPLIFIED_UTILS_SRCS})
endif()

# 主可执行文件
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    ${RKNN_SRCS}
)

# 图像显示节点
add_executable(${PROJECT_NAME}_display_node
    src/image_display_node.cpp
)

# 组件库
add_library(${PROJECT_NAME}_component SHARED
    ${RKNN_SRCS}
)

# ROS2依赖列表
set(ROS2_DEPENDENCIES
    rclcpp
    rclcpp_components
    sensor_msgs
    vision_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    image_transport
)

# 链接ROS2依赖
ament_target_dependencies(${PROJECT_NAME}_node ${ROS2_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME}_display_node 
    rclcpp 
    sensor_msgs 
    vision_msgs 
    std_msgs
    cv_bridge 
)
ament_target_dependencies(${PROJECT_NAME}_component ${ROS2_DEPENDENCIES})

# 链接系统库
target_link_libraries(${PROJECT_NAME}_node
    ${RKNN_RT_LIB}
    ${OpenCV_LIBS}
    dl
    pthread
)

target_link_libraries(${PROJECT_NAME}_component
    ${RKNN_RT_LIB}
    ${OpenCV_LIBS}
    dl
    pthread
)

target_link_libraries(${PROJECT_NAME}_display_node
    ${OpenCV_LIBS}
    ${JSONCPP_LIBRARIES}
)

# 添加jsoncpp编译标志
target_compile_options(${PROJECT_NAME}_display_node PRIVATE ${JSONCPP_CFLAGS_OTHER})

# 如果不使用简化依赖，链接原始工具库
if(NOT USE_SIMPLIFIED_DEPS)
    target_link_libraries(${PROJECT_NAME}_node
        imageutils
        fileutils
        imagedrawing
    )
    target_link_libraries(${PROJECT_NAME}_component
        imageutils
        fileutils
        imagedrawing
    )
endif()

# 注册组件
rclcpp_components_register_nodes(${PROJECT_NAME}_component "RknnYolo11Node")

# 设置目标属性
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17)

# 安装
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_display_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()