#!/usr/bin/env python3
"""
calibration_data_collection.launch.py

Purpose: Lightweight launch file dedicated to camera-LiDAR calibration data collection
Collects:
  - /hesai_ros_driver/pandar_points (Hesai QT128 point cloud)
  - /nedo/left/capture (left camera raw image)
  - /nedo/left/camera_info (left camera calibration info)

Optimizations:
  - Single camera only (left)
  - No compression nodes
  - No unnecessary processing
  - Multithreaded container for better performance
  - Livox driver disabled
"""
#このlaunchファイルの位置: ros_ws/src/nedo_bringup/launch/
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))
# _ws_dir → リポジトリルート
third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()
    
    # Third-party workspace environment setup
    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    ld.add_action(SetEnvironmentVariable(
        name='AMENT_PREFIX_PATH',
        value=f"{third_party_install_prefix}:" + os.environ.get('AMENT_PREFIX_PATH', '')
    ))
    ld.add_action(SetEnvironmentVariable(
        name='CMAKE_PREFIX_PATH',
        value=f"{third_party_install_prefix}:" + os.environ.get('CMAKE_PREFIX_PATH', '')
    ))
    
    # Configuration paths
    package_share_dir = get_package_share_directory('nedo_bringup')
    default_left_calib_path = os.path.join(package_share_dir, 'config', 'left_camera_ost_1920x1080.yaml')
    
    # Launch arguments
    left_calib_file = LaunchConfiguration('left_calib_file')
    arg_left_calib_file = DeclareLaunchArgument(
        'left_calib_file',
        default_value='file://' + default_left_calib_path,
        description='Left camera calibration YAML file'
    )
    
    topic_ns = 'nedo'
    container_name = 'calibration_container'
    
    # ====================================================================
    # Hesai QT128 LiDAR Driver
    # ====================================================================
    hesai_config_file = os.path.join(third_party_install_prefix, 'HesaiLidar_ROS_2.0', 'config', 'config.yaml')
    hesai_lib_dir = os.path.join(third_party_install_prefix, 'hesai_ros_driver', 'lib')
    hesai_build_lib_dir = os.path.join(third_party_install_prefix, 'hesai_ros_driver', 'build', 'HesaiLidar_SDK_2.0', 'libhesai')
    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    hesai_ld_path = ':'.join(filter(None, [hesai_lib_dir, hesai_build_lib_dir, existing_ld_path]))
    
    hesai_launch = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        namespace='hesai_ros_driver',
        output='screen',
        parameters=[{'config_path': hesai_config_file}],
        additional_env={'LD_LIBRARY_PATH': hesai_ld_path}
    )
    
    # Static TF: base_link -> lidar_link (Hesai QT128 mounting position)
    hesai_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hesai_to_base_link_tf',
        arguments=[
            '0.36', '0.25', '1.9',                      # x, y, z (meters)
            '0', '0.3826834323650898', '0.9238795325112867', '0',  # quaternion: roll=45°, yaw=180°
            'base_link', 'lidar_link'
        ],
        output='screen'
    )
    
    # ====================================================================
    # Camera Container (Multithreaded for better performance)
    # ====================================================================
    container = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_mt',  # Multithreaded container
        output='screen',
    )
    
    # ====================================================================
    # Left Camera Components (Elgato Facecam Pro)
    # ====================================================================
    left_device = '/dev/video4'  # Elgato Facecam Pro
    
    capture_node_descriptions = []
    
    if os.path.exists(left_device):
        capture_node_descriptions.extend([
            # Camera capture node
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::NedoCapNode',
                name='left_cap',
                namespace=topic_ns + '/left',
                parameters=[
                    {'device': left_device},
                    {'frame_id': 'left_camera_frame'},
                    # High-quality settings for calibration (1920x1080 @ 30fps)
                    {'pipeline': f'v4l2src device={left_device} ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! appsink'}
                ]
            ),
            # Camera info node (publishes calibration parameters)
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CaminfoNode',
                name='left_caminfo',
                namespace=topic_ns + '/left',
                parameters=[{'calib_file': left_calib_file}]
            )
        ])
        ld.add_action(LogInfo(msg=f'[CALIB] Left camera (Elgato Facecam Pro) detected at {left_device}'))
    else:
        ld.add_action(LogInfo(msg=f'[ERROR] Left camera device {left_device} not found!'))
        ld.add_action(LogInfo(msg='[ERROR] Calibration cannot proceed without camera. Please check device connection.'))
    
    # Load camera components into container
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=capture_node_descriptions
    )
    
    # ====================================================================
    # Static TF: map -> odom -> base_link (for frame consistency)
    # ====================================================================
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0', '0', '0', '1',
            'map', 'odom'
        ],
        output='screen'
    )
    
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0', '0', '0', '1',
            'odom', 'base_link'
        ],
        output='screen'
    )
    
    # ====================================================================
    # Add all actions to launch description
    # ====================================================================
    ld.add_action(LogInfo(msg='========================================'))
    ld.add_action(LogInfo(msg='Camera-LiDAR Calibration Data Collection'))
    ld.add_action(LogInfo(msg='========================================'))
    ld.add_action(LogInfo(msg='Output topics:'))
    ld.add_action(LogInfo(msg='  - /hesai_ros_driver/pandar_points (sensor_msgs/PointCloud2)'))
    ld.add_action(LogInfo(msg='  - /nedo/left/capture (sensor_msgs/Image)'))
    ld.add_action(LogInfo(msg='  - /nedo/left/camera_info (sensor_msgs/CameraInfo)'))
    ld.add_action(LogInfo(msg='========================================'))
    
    ld.add_action(arg_left_calib_file)
    
    # Launch Hesai driver
    ld.add_action(hesai_launch)
    ld.add_action(hesai_static_tf)
    
    # Launch camera
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    
    # Launch static TFs
    ld.add_action(map_to_odom_tf)
    ld.add_action(odom_to_base_tf)
    
    ld.add_action(LogInfo(msg='[CALIB] System ready for data collection'))
    ld.add_action(LogInfo(msg='[CALIB] To record data: ros2 bag record /hesai_ros_driver/pandar_points /nedo/left/capture /nedo/left/camera_info'))
    
    return ld
