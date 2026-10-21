import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

#このlaunchファイルの位置: ros_ws/src/nedo_bringup/launch/
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))
# _ws_dir → リポジトリルート
third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')

def generate_launch_description():

    ld = LaunchDescription()
    
    # third_party ワークスペースのパッケージ（Hesai/Livoxなど）を検出できるように環境変数を拡張
    ld.add_action(SetEnvironmentVariable(
        name='AMENT_PREFIX_PATH',
        value=f"{third_party_install_prefix}:" + os.environ.get('AMENT_PREFIX_PATH', '')
    ))
    ld.add_action(SetEnvironmentVariable(
        name='CMAKE_PREFIX_PATH',
        value=f"{third_party_install_prefix}:" + os.environ.get('CMAKE_PREFIX_PATH', '')
    ))
    
    package_share_dir = get_package_share_directory('nedo_bringup')
    # 4K解像度 (3840x2160) を使用
    default_front_calib_path = os.path.join(package_share_dir, 'config', 'front_camera_ost.yaml')
    default_left_calib_path = os.path.join(package_share_dir, 'config', 'left_camera_ost.yaml')
    default_right_calib_path = os.path.join(package_share_dir, 'config', 'right_camera_ost.yaml')
    #default_calib_path = os.path.join(get_package_share_directory('nedo_bringup'),'config','ost.yaml')

    # Launch引数として各カメラのキャリブレーションファイルを設定
    front_calib_file = LaunchConfiguration('front_calib_file')
    left_calib_file = LaunchConfiguration('left_calib_file')
    right_calib_file = LaunchConfiguration('right_calib_file')
    use_hesai = LaunchConfiguration('use_hesai')
    
    #calib_file =LaunchConfiguration('calib_file')
    #arg_calib_file = DeclareLaunchArgument('calib_file',
    #                                       default_value='file://'+default_calib_path,
    #                                       description='calibration yaml file')
    arg_front_calib_file = DeclareLaunchArgument(
        'front_calib_file',
        default_value='file://' + default_front_calib_path,
        description='Front camera calibration YAML file'
    )
    arg_left_calib_file = DeclareLaunchArgument(
        'left_calib_file',
        default_value='file://' + default_left_calib_path,
        description='Left camera calibration YAML file'
    )
    arg_right_calib_file = DeclareLaunchArgument(
        'right_calib_file',
        default_value='file://' + default_right_calib_path,
        description='Right camera calibration YAML file'
    )
    arg_use_hesai = DeclareLaunchArgument(
        'use_hesai',
        default_value='false',
        description='Enable Hesai QT128 LiDAR (set to true for Hesai calibration)'
    )

    topic_ns = 'nedo'

    container_name = 'data_collection'

    # 右側LiDARのみを使用する設定ファイル
    # 注意: 通常の設定ファイルには両方のLiDARが有効になっているため、
    # キャリブレーション用には右側のみの設定ファイルを使用
    livox_right_only_config = os.path.join(package_share_dir, 'config', 'livox_right_only_config.json')
    
    # Livox Avia LiDAR driver - 右側のみ (Right side only)
    # 右側Livox: 3JEDM4R0014V101, frame_id: livox_right_frame
    livox_launch = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_right_frame'},  # 右側LiDARのフレーム
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': livox_right_only_config},  # 右側のみの設定ファイル
            {'cmdline_input_bd_code': '3JEDM4R0014V101'}  # 右側LiDARのシリアル番号
        ]
    )
    ld.add_action(LogInfo(msg='[INFO] Livox Avia LiDAR driver enabled (RIGHT side ONLY: 3JEDM4R0014V101).'))
    
    # Hesai QT128 driver: Conditional (disabled by default for Livox calibration)
    # 将来的にHesai QT128キャリブレーション用に残しておく
    # 使用する場合: use_hesai:=true を引数で指定
    hesai_launch = None
    hesai_static_tf = None
    hesai_config_file = os.path.join(_ws_dir, 'third_party', 'src', 'HesaiLidar_ROS_2.0', 'config', 'config.yaml')  
    
    # Debug: print to stderr (visible in launch output)
    import sys
    print(f'[LAUNCH_DEBUG] use_hesai parameter check (will be evaluated at runtime)', file=sys.stderr)
    ld.add_action(LogInfo(msg='[INFO] Hesai QT128 LiDAR: use_hesai=false (default). Set use_hesai:=true to enable.'))
    
    if os.path.exists(hesai_config_file):
        # Try direct path first, then fallback to package search
        hesai_share_dir = None
        hesai_install_dir = os.path.join(third_party_install_prefix, 'hesai_ros_driver')
        if os.path.exists(hesai_install_dir):
            hesai_share_dir = os.path.join(hesai_install_dir, 'share', 'hesai_ros_driver')
        else:
            try:
                hesai_share_dir = get_package_share_directory('hesai_ros_driver')
            except PackageNotFoundError as e:
                ld.add_action(LogInfo(msg=f'[LAUNCH_WARN] hesai_ros_driver package not found: {e}. Hesai will be unavailable.'))
        
        if hesai_share_dir and os.path.exists(hesai_share_dir):
            try:
                hesai_lib_dir = os.path.join(hesai_share_dir, '..', '..', 'lib')
                hesai_build_lib_dir = os.path.join(third_party_install_prefix, 'build', 'hesai_ros_driver', 'src', 'driver', 'HesaiLidar_SDK_2.0', 'libhesai')
                existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
                ld_path = ':'.join(filter(None, [hesai_lib_dir, hesai_build_lib_dir, existing_ld_path]))
                
                # Set AMENT_PREFIX_PATH for hesai_ros_driver package discovery
                existing_ament_path = os.environ.get('AMENT_PREFIX_PATH', '')
                hesai_ament_path = f"{third_party_install_prefix}:{existing_ament_path}"
                
                # Hesai launch with condition
                from launch.conditions import IfCondition
                hesai_launch = Node(
                    package='hesai_ros_driver',
                    executable='hesai_ros_driver_node',
                    namespace='hesai_ros_driver',
                    output='screen',
                    parameters=[{'config_path': hesai_config_file}],
                    additional_env={
                        'LD_LIBRARY_PATH': ld_path,
                        'AMENT_PREFIX_PATH': hesai_ament_path
                    },
                    condition=IfCondition(use_hesai)  # 条件付き起動
                )
                
                hesai_static_tf = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='hesai_to_base_link_tf',
                    arguments=[
                        '0.36', '0.25', '1.9',
                        '0', '0.3826834323650898', '0.9238795325112867', '0',  # roll=45°, yaw=180°
                        'base_link', 'lidar_link'
                    ],
                    output='screen',
                    condition=IfCondition(use_hesai)  # 条件付き起動
                )
                ld.add_action(LogInfo(msg='[INFO] Hesai driver configured (will start only if use_hesai:=true)'))
            except Exception as e:
                ld.add_action(LogInfo(msg=f'[LAUNCH_ERROR] Failed to configure Hesai driver: {e}'))
                import traceback
                error_msg = traceback.format_exc()
                ld.add_action(LogInfo(msg=f'[LAUNCH_ERROR] Traceback: {error_msg}'))
        else:
            ld.add_action(LogInfo(msg=f'[LAUNCH_WARN] Hesai share dir not found: {hesai_share_dir}'))
    else:
        ld.add_action(LogInfo(msg=f'[LAUNCH_WARN] Hesai config file not found at {hesai_config_file}. Hesai unavailable.'))

    container = Node(
        name= container_name,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        )

    capture_node_descriptions = [
        ComposableNode(
            package='nedo_capture',
            plugin='nedo_capture::CompressNode',
            name='cap_comp',
            namespace=topic_ns
        )
    ]

    # Front camera: Skipped (using right camera for this calibration)
    # front_device = '/dev/nedo_cam_front'
    ld.add_action(LogInfo(msg='[INFO] Front camera skipped (using right camera for calibration).'))

    # Left camera: Skipped (not used in this configuration)
    # left_device = '/dev/video4'
    ld.add_action(LogInfo(msg='[INFO] Left camera skipped (not used).'))

    # Right camera: Elgato Facecam Pro (右側)
    # 注意: /dev/nedo_cam_right はudevルールで作成されるシンボリックリンク
    # キャリブレーション用なので、デフォルトトピック名 (/nedo/capture, /nedo/camera_info) を使用
    right_device = '/dev/nedo_cam_right'
    if os.path.exists(right_device):
        capture_node_descriptions.extend([
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::NedoCapNode',
                name='right_cap',
                namespace=topic_ns,
                parameters=[
                    {'device': right_device},  # udevルールによる固定シンボリックリンク
                    {'frame_id': 'right_camera_frame'},  # 右カメラフレーム
                    {'pipeline': f'v4l2src device={right_device} io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'}
                ]
                # remappingなし: /nedo/capture として配信
            ),
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CaminfoNode',
                name='right_caminfo',
                namespace=topic_ns,
                parameters=[{'calib_file': right_calib_file}]
                # remappingなし: /nedo/camera_info として配信
            )
        ])
        ld.add_action(LogInfo(msg=f'Detected Elgato Facecam Pro (right camera) at {right_device}.'))
    else:
        ld.add_action(LogInfo(msg=f'[WARN] Right camera device {right_device} not found. Skipping right camera startup.'))

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=capture_node_descriptions
    )  

    #ld.add_action(arg_calib_file)
    ld.add_action(arg_front_calib_file)
    ld.add_action(arg_left_calib_file)
    ld.add_action(arg_right_calib_file)
    ld.add_action(arg_use_hesai)
    if livox_launch is not None:
        ld.add_action(livox_launch)
    if hesai_launch is not None:
        ld.add_action(hesai_launch)

    # Static TF for Livox Avia - 右側 (Right side)
    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_right_to_base_link_tf',
        arguments=[
            '0.0', '-0.28', '0.87',     # x y z (右側Livox Aviaの位置: 右に0.28m, 高さ0.87m)
            '0', '0', '-0.707', '0.707', # qx qy qz qw (右向き-90度回転)
            'base_link', 'livox_right_frame'  # parent: base_link, child: livox_right_frame
        ],
        output='screen'
    )

    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    
    # Add Livox static TF
    if livox_launch is not None:
        ld.add_action(LogInfo(msg='Publishing static TF base_link -> livox_right_frame'))
        ld.add_action(livox_static_tf)
    
    # Hesai nodes are already added above with IfCondition(use_hesai)
    # No additional action needed here
    
    return ld