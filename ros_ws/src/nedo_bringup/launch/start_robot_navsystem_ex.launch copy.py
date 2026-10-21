import os
import yaml

# start_robot_navsystem.launch.py (統合版)
# 目的:
# - センサ（Livox Avia / Hesai QT128）・認識・自己位置推定・ナビゲーションを起動
# - 引数で構成切り替え（bag再生/実機、Contour/Detector、Hesai/Livoxなど）
# - enable_scrap_detection=true でスクラップ検出（円筒フィルタ＋最高点検出）を追加起動
# ポイント:
# - Landmark座標は landmark_map_server_node が /landmark_map に配信（Localizer 等が購読）
# - use_livox/use_hesai を両方 true にすれば2系統同時に点群を取得
# - スクラップ検出は scrap_detection.launch.py からの統合（単体起動も引き続き可能）

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,LogInfo,ExecuteProcess,SetEnvironmentVariable
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration,PythonExpression,AndSubstitution,NotSubstitution
from launch_ros.actions import Node,LoadComposableNodes
from launch_ros.descriptions import ComposableNode

#このlaunchファイルの位置: ros_ws/src/nedo_bringup/launch/
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))

# _ws_dir → リポジトリルート

# =====================================================================
# スクラップ検出用ヘルパー関数（scrap_detection.launch.py から統合）
# =====================================================================
def load_marker_coordinates(config_path, marker_id_1='marker0', marker_id_2='marker8'):
    """Load marker coordinates from params.yaml and calculate cylinder center and radius from specified markers.
    
    Args:
        config_path: Path to params.yaml
        marker_id_1: First marker ID (default: 'marker0')
        marker_id_2: Second marker ID (default: 'marker8')
    
    Returns:
        Dictionary with marker1_x, marker1_y, marker2_x, marker2_y, or None if not found
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        params = config.get('/**', {}).get('ros__parameters', {})
        lm_ids = params.get('LM_ids', [])
        lm_xs = params.get('LM_xs', [])
        lm_ys = params.get('LM_ys', [])
        
        # Find specified marker coordinates
        marker1_x, marker1_y = None, None
        marker2_x, marker2_y = None, None
        
        for i, lm_id in enumerate(lm_ids):
            if lm_id == marker_id_1 and i < len(lm_xs) and i < len(lm_ys):
                marker1_x, marker1_y = lm_xs[i], lm_ys[i]
            elif lm_id == marker_id_2 and i < len(lm_xs) and i < len(lm_ys):
                marker2_x, marker2_y = lm_xs[i], lm_ys[i]
        
        if marker1_x is not None and marker2_x is not None:
            return {
                'marker1_id': marker_id_1,
                'marker1_x': float(marker1_x),
                'marker1_y': float(marker1_y),
                'marker2_id': marker_id_2,
                'marker2_x': float(marker2_x),
                'marker2_y': float(marker2_y),
            }
        else:
            print(f"[scrap_detection] Warning: Could not find markers '{marker_id_1}' and/or '{marker_id_2}' in params.yaml")
    except Exception as e:
        print(f"[scrap_detection] Warning: Failed to load marker coordinates: {e}")
    
    return None


def generate_launch_description():
    ld = LaunchDescription()

    # ROS2環境の初期化（ros2 topic listを安定動作させるため）
    # - システム共通の環境変数やセットアップを行う外部スクリプト
    ros2_env_setup = ExecuteProcess(
        cmd=[os.path.join(_ws_dir, 'scripts', 'setup_ros2_environment.sh')],
        output='screen',
        shell=True
    )
    # 共有設定ファイル（パッケージのshareから既定パスを構築）
    # 4K解像度 (3840x2160) を使用
    default_calib_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'left_camera_ost.yaml'  # left カメラ 4K用キャリブレーション
    )
    default_calib_front_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'front_camera_ost.yaml'  # front カメラ 4K用キャリブレーション
    )
    default_tf_json_left_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'calib_left.json'
    )
    default_tf_json_front_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'calib_front.json'
    )
    # 右カメラ用キャリブレーションパス
    default_calib_right_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'right_camera_ost.yaml'  # right カメラ 4K用キャリブレーション
    )
    default_tf_json_right_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'calib_right.json'
    )
    # Landmark map (YAML) default path
    # - params.yaml（LM_ids/LM_xs/LM_ys 形式）を既定とする
    default_landmark_yaml_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'params.yaml'
    )
    robot_localization_config = os.path.join(get_package_share_directory('nedo_bringup'),
                                              'config', 'robot_localization.yaml')

    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    livox_launch_path = os.path.join(third_party_install_prefix, 'livox_ros2_avia', 'share', 'livox_ros2_avia')

    # third_party ワークスペースのパッケージ（Hesai, Livox等）を検出できるように環境変数を拡張
    # isolated install layout のため、各パッケージのプレフィックスを個別に追加する必要がある
    _third_party_pkg_prefixes = ':'.join(
        os.path.join(third_party_install_prefix, d)
        for d in sorted(os.listdir(third_party_install_prefix))
        if os.path.isdir(os.path.join(third_party_install_prefix, d))
    )
    # Python評価時に os.environ を更新（Node() のパッケージ解決に必要）
    os.environ['AMENT_PREFIX_PATH'] = _third_party_pkg_prefixes + ':' + os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['CMAKE_PREFIX_PATH'] = _third_party_pkg_prefixes + ':' + os.environ.get('CMAKE_PREFIX_PATH', '')
    # 子プロセスにも伝搬
    ld.add_action(SetEnvironmentVariable(
        name='AMENT_PREFIX_PATH',
        value=os.environ['AMENT_PREFIX_PATH']
    ))
    ld.add_action(SetEnvironmentVariable(
        name='CMAKE_PREFIX_PATH',
        value=os.environ['CMAKE_PREFIX_PATH']
    ))

    # Launch 引数（後段の条件分岐に使用）
    use_bag = LaunchConfiguration('use_bag')
    is_contour = LaunchConfiguration('is_contour')
    use_livox = LaunchConfiguration('use_livox')
    use_hesai = LaunchConfiguration('use_hesai')
    calib_file = LaunchConfiguration('calib_file')
    calib_file_front = LaunchConfiguration('calib_file_front')
    calib_file_right = LaunchConfiguration('calib_file_right')  # 右カメラ用
    json_file_left = LaunchConfiguration('json_file_left')
    json_file_front = LaunchConfiguration('json_file_front')
    json_file_right = LaunchConfiguration('json_file_right')  # 右カメラ用
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    enable_landmark_marker = LaunchConfiguration('enable_landmark_marker')
    enable_realtime_visualizer = LaunchConfiguration('enable_realtime_visualizer')
    enable_autonomous = LaunchConfiguration('enable_autonomous')
    use_right_sensors = LaunchConfiguration('use_right_sensors')  # 右側センサ有効化フラグ
    enable_scrap_detection = LaunchConfiguration('enable_scrap_detection')  # スクラップ検出有効化フラグ

    # ==== 引数宣言（CLIから上書き可能）====
    declare_bag_cmd = DeclareLaunchArgument('use_bag',
                                            default_value='false',
                                            description='use bag or not')

    declare_contour_cmd = DeclareLaunchArgument('is_contour',
                                                default_value='true')

    declare_use_livox_cmd = DeclareLaunchArgument('use_livox',
                                                  default_value='true',  # Livox をデフォルト有効
                                                  description='enable Livox Avia LiDAR (default: enabled)')

    # Hesai（QT128）を既定で無効。
    # use_bag=false かつ use_livox/use_hesai が true なら両系統同時に起動して点群取得する。
    declare_use_hesai_cmd = DeclareLaunchArgument('use_hesai',
                                                  default_value='false',
                                                  description='enable Hesai QT128 LiDAR')

    # Optional feature flags (for nodes that may not be built in this workspace)
    declare_enable_landmark_marker_cmd = DeclareLaunchArgument('enable_landmark_marker',
                                                              default_value='false',
                                                              description='enable RViz landmark marker publisher (requires executable)')
    declare_enable_realtime_visualizer_cmd = DeclareLaunchArgument('enable_realtime_visualizer',
                                                                   default_value='false',
                                                                   description='enable realtime visualizer node (requires executable)')
    declare_enable_autonomous_cmd = DeclareLaunchArgument('enable_autonomous',
                                                          default_value='false',
                                                          description='enable simple autonomous sequence node (requires executable)')

    declare_calib_file_cmd = DeclareLaunchArgument('calib_file',
                                           default_value='file://'+default_calib_path,
                                           description='left camera calibration yaml file')
    
    declare_calib_file_front_cmd = DeclareLaunchArgument('calib_file_front',
                                           default_value='file://'+default_calib_front_path,
                                           description='front camera calibration yaml file')
    
    declare_tf_json_left_cmd = DeclareLaunchArgument('json_file_left',
                                                default_value=default_tf_json_left_path,
                                                description='left camera to LiDAR calib.json path')
    
    declare_tf_json_front_cmd = DeclareLaunchArgument('json_file_front',
                                                default_value=default_tf_json_front_path,
                                                description='front camera to LiDAR calib.json path (absolute path or relative to package share dir)')

    # 右カメラ用引数宣言
    declare_calib_file_right_cmd = DeclareLaunchArgument('calib_file_right',
                                           default_value='file://'+default_calib_right_path,
                                           description='right camera calibration yaml file')
    
    declare_tf_json_right_cmd = DeclareLaunchArgument('json_file_right',
                                                default_value=default_tf_json_right_path,
                                                description='right camera to LiDAR calib.json path')

    declare_use_right_sensors_cmd = DeclareLaunchArgument('use_right_sensors',
                                                          default_value='true',
                                                          description='enable right side sensors (camera + LiDAR)')

    # Landmark map YAML (legacy keys: LM_ids, LM_xs, LM_ys)
    declare_landmark_map_file_cmd = DeclareLaunchArgument(
        'landmark_map_file',
        default_value=default_landmark_yaml_path,
        description='YAML file containing landmark coordinates (legacy format supported)'
    )

    # ==== スクラップ検出用引数宣言 ====
    declare_enable_scrap_detection_cmd = DeclareLaunchArgument(
        'enable_scrap_detection',
        default_value='false',
        description='enable scrap detection system (Hesai region filter + highest point)')
    
    declare_hesai_input_topic_cmd = DeclareLaunchArgument(
        'hesai_input_topic',
        default_value='/lidar_points',
        description='Hesai LiDAR input topic for scrap detection')
    
    declare_scrap_marker_id_1_cmd = DeclareLaunchArgument(
        'scrap_marker_id_1',
        default_value='marker0',
        description='First marker ID for scrap detection cylinder region')
    
    declare_scrap_marker_id_2_cmd = DeclareLaunchArgument(
        'scrap_marker_id_2',
        default_value='marker8',
        description='Second marker ID for scrap detection cylinder region')

    topic_ns = 'nedo'  # Composableノード群の名前空間（画像・LiDAR処理系）

    # ==== Livox Avia LiDAR ドライバ ====
    # 条件: Not(use_bag) AND use_livox
    # multi_topic=1: 複数LiDARを別々のトピックで配信 (/livox/lidar_XXXXXXXX)
    livox_launch = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        condition = IfCondition(
            AndSubstitution(
                NotSubstitution(use_bag),
                use_livox
            )
        ),
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 1},  # 1: 各LiDARを個別トピックで配信
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': os.path.join(livox_launch_path, 'config', 'livox_lidar_config.json')},
            {'cmdline_input_bd_code': 'livox0000000001'}
        ]
    )

    # ==== Hesai QT128 LiDAR ドライバ====
    # 条件: Not(use_bag) AND use_hesai
    hesai_config_file = os.path.join(_ws_dir, 'third_party', 'src', 'HesaiLidar_ROS_2.0', 'config', 'config.yaml')
    hesai_share_dir = os.path.join(third_party_install_prefix, 'hesai_ros_driver', 'share', 'hesai_ros_driver')
    hesai_lib_dir = os.path.join(third_party_install_prefix, 'hesai_ros_driver', '..', '..', 'lib')
    hesai_build_lib_dir = os.path.join(_ws_dir, 'third_party', 'build', 'hesai_ros_driver', 'src', 'driver', 'HesaiLidar_SDK_2.0', 'libhesai')
    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    hesai_ld_path = ':'.join(filter(None, [hesai_lib_dir, hesai_build_lib_dir, existing_ld_path]))
    
    hesai_launch = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        namespace='hesai_ros_driver',
        output='screen',
        parameters=[{'config_path': hesai_config_file}],
        additional_env={'LD_LIBRARY_PATH': hesai_ld_path},
        condition = IfCondition(
            AndSubstitution(
                NotSubstitution(use_bag),
                use_hesai
            )
        )
    )

    # グループ化された設定
    bag_group = GroupAction(
        actions=[
            #container for capture 
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='capture_container',
                output='screen',
            ),
            #container for localization
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='lidar_container',
                output='screen',
            ),
            #load capture components
            LoadComposableNodes(
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp',
                        namespace=topic_ns
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo',
                        namespace=topic_ns,
                        parameters=[
                            {'calib_file':calib_file_front}  # 前カメラのキャリブレーションファイル
                        ]
                    ),
                    # Livox Avia用のDataSyncNode設定（前方LiDAR）
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DataSyncNode',
                        name='syncronizer',
                        namespace=topic_ns,
                        parameters=[
                            {'livox_topic': '/livox/lidar_3JEDM4V001DY881'},  # 前方Livox Aviaの点群トピック
                            {'use_livox': use_livox}  # LaunchConfigurationから取得
                        ]
                    )
                ]
            ),
            # 右側LiDAR用のDataSyncNode
            LoadComposableNodes(
                condition=IfCondition(AndSubstitution(use_right_sensors, use_livox)),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DataSyncNode',
                        name='syncronizer_right',
                        namespace=topic_ns,
                        parameters=[
                            {'livox_topic': '/livox/lidar_3JEDM4R0014V101'},
                            {'use_livox': True},
                            {'output_frame_id': 'livox_right_frame'}
                        ],
                        remappings=[
                            ('capture', 'capture_right'),
                            ('sync_image', 'sync_image_right'),
                            ('sync_point', 'sync_lidar_right')
                        ]
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo_right',
                        namespace=topic_ns,
                        parameters=[
                            {'calib_file': calib_file_right}
                        ],
                        remappings=[
                            ('camera_info', 'camera_info_right')
                        ]
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp_right',
                        namespace=topic_ns,
                        remappings=[
                            ('debug_image', 'debug_image_right'),
                            ('image_pub', 'image_pub_right')
                        ]
                    )
                ]
            ),
            # 前カメラ（Elgato Facecam Pro）
            LoadComposableNodes(
                condition=UnlessCondition(use_bag),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::NedoCapNode',
                        name='capture_hardware',
                        namespace=topic_ns,
                        parameters=[
                            {'device': '/dev/nedo_cam_front'},
                            {'frame_id': 'front_camera_frame'},
                            {'pipeline': 'v4l2src device=/dev/nedo_cam_front io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'}
                        ]
                    )
                ]
            ),
            # 右カメラ（Elgato Facecam Pro 2台目）
            LoadComposableNodes(
                condition=IfCondition(
                    AndSubstitution(
                        NotSubstitution(use_bag),
                        use_right_sensors
                    )
                ),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::NedoCapNode',
                        name='capture_hardware_right',
                        namespace=topic_ns,
                        parameters=[
                            {'device': '/dev/nedo_cam_right'},
                            {'frame_id': 'right_camera_frame'},
                            {'pipeline': 'v4l2src device=/dev/nedo_cam_right io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'}
                        ],
                        remappings=[
                            ('capture', 'capture_right'),
                            ('camera_info', 'camera_info_right')
                        ]
                    )
                ]
            ),
            LoadComposableNodes(
                condition=UnlessCondition(is_contour),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DetectorNode',
                        name='point_detector',
                        namespace=topic_ns
                    )
                ]
            ),
            LoadComposableNodes(
                condition=IfCondition(is_contour),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourNode',
                        name='contour_detector',
                        namespace=topic_ns
                    )
                ]
            ),
            #load localization components
            LoadComposableNodes(
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection',
                        namespace=topic_ns,
                        parameters=[
                            {'lidar_frame': 'livox_frame'},
                            {'camera_frame': 'front_camera_frame'}
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::LidarCamTF',
                        name='lidar_cam_tf_front',
                        namespace=topic_ns,
                        parameters=[
                            {'json_file_path': json_file_front},
                            {'camera_frame': 'front_camera_frame'},
                            {'lidar_frame': 'livox_frame'}
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourMatcher',
                        name='contour_matcher',
                        namespace=topic_ns
                    )
                ]
            ),
            # 右側センサ用のlocalizationコンポーネント
            LoadComposableNodes(
                condition=IfCondition(use_right_sensors),
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection_right',
                        namespace=topic_ns,
                        parameters=[
                            {'lidar_frame': 'livox_right_frame'},
                            {'camera_frame': 'right_camera_frame'}
                        ],
                        remappings=[
                            ('sync_point', 'sync_lidar_right'),
                            ('camera_info', 'camera_info_right'),
                            ('projected', 'projected_right')
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::LidarCamTF',
                        name='lidar_cam_tf_right',
                        namespace=topic_ns,
                        parameters=[
                            {'json_file_path': json_file_right},
                            {'camera_frame': 'right_camera_frame'},
                            {'lidar_frame': 'livox_right_frame'}
                        ]
                    )
                ]
            ),
            LoadComposableNodes(
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_viz',
                        plugin='nedo_viz::ShowProjectionNode',
                        name='debug_img',
                        namespace=topic_ns,
                        remappings=[
                            ('sync_image', 'sync_image'),
                            ('projected', 'projected'),
                            ('detected_contours', 'detected_contours'),
                            ('debug_image_front', 'debug_image_front'),
                            ('sync_image_right', 'sync_image_right'),
                            ('projected_right', 'projected_right'),
                            ('detected_contours_right', 'detected_contours_right'),
                            ('debug_image_right', 'debug_image_right'),
                        ]
                    )
                ]
            ),
            Node(
                package='nedo_localization',
                executable = 'localizer_node',
                parameters=[
                    landmark_map_file,
                    {
                        'lpf_rate': 0.7,
                        'use_tf_pub': True
                    }
                ],
                output='screen'
            ),
            # リアルタイム可視化ノード（任意: 実行ファイルがある場合のみ）
            Node(
                package='nedo_localization',
                executable='realtime_visualizer_node',
                name='realtime_visualizer_node',
                condition=IfCondition(enable_realtime_visualizer),
                parameters=[{
                    'publish_rate': 10.0,
                    'trajectory_history_size': 1000
                }]
            ),
        ]
    )

    # ランドマークマーカー表示ノード（bag_groupの外で独立して起動）
    landmark_map_server = Node(
        package='nedo_localization',
        executable='landmark_map_server_node',
        name='landmark_map_server_node',
        output='screen',
        parameters=[
            landmark_map_file,
            {
                'publish_rate_hz': 1.0
            }
        ]
    )

    landmark_marker_publisher = Node(
        package='nedo_localization',
        executable='landmark_marker_publisher',
        name='landmark_marker_publisher',
        output='screen',
        condition=IfCondition(enable_landmark_marker),
        parameters=[
            landmark_map_file
        ]
    )

    # Pose Bridge ノード: /robot_pose (PoseStamped) → /robot_pose2d (Pose2D) 変換
    pose_bridge_node = Node(
        package='nedo_bringup',
        executable='pose_bridge.py',
        name='pose_bridge_node',
        output='screen'
    )

    # robot_localization EKFノード
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_config],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )

    # LiDAR frame transforms - 前方Livox
    static_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_link_tf',
        arguments=[
            '0.40', '0.0', '0.70',
            '0', '0', '0', '1',
            'base_link', 'livox_frame'
        ],
        output='screen'
    )

    # LiDAR frame transforms - 右側Livox
    static_livox_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_right_to_base_link_tf',
        arguments=[
            '0.0', '-0.28', '0.87',
            '0', '0', '-0.707', '0.707',
            'base_link', 'livox_right_frame'
        ],
        output='screen',
        condition=IfCondition(use_right_sensors)
    )

    # QT128 LiDAR座標変換ノード（正確な設置位置と角度を反映）
    lidar_tf_publisher = Node(
        package='nedo_localization',
        executable='lidar_tf_publisher_node',
        name='lidar_tf_publisher_node',
        output='screen',
        condition=IfCondition(use_hesai)
    )

    # Map frame transform (fixed world frame)
    static_map_tf = Node(
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

    # ==== TFツリーの分岐 ====
    # enable_scrap_detection=false: odom → base_link (従来通り)
    # enable_scrap_detection=true:  odom → base_footprint → base_link (地面基準フレーム追加)
    
    # 従来のTF: odom → base_link (スクラップ検出無効時のみ)
    static_odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0', '0', '0', '1',
            'odom', 'base_link'
        ],
        output='screen',
        condition=UnlessCondition(enable_scrap_detection)
    )

    # スクラップ検出用TF: odom → base_footprint (スクラップ検出有効時のみ)
    static_odom_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'odom', 'base_footprint'
        ],
        output='screen',
        condition=IfCondition(enable_scrap_detection)
    )

    # スクラップ検出用TF: base_footprint → base_link (Z=0.3m上)
    static_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=[
            '0', '0', '0.3', '0', '0', '0', '1',
            'base_footprint', 'base_link'
        ],
        output='screen',
        condition=IfCondition(enable_scrap_detection)
    )

    # =====================================================================
    # スクラップ検出ノード群（enable_scrap_detection=true のとき起動）
    # =====================================================================
    # ソースディレクトリ相対で設定ファイルを参照 (再ビルド不要で反映)
    this_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.normpath(os.path.join(this_dir, '..', 'config'))
    params_yaml_path = os.path.join(config_dir, 'params.yaml')

    # マーカー座標を読み込み（環境変数から上書き可能）
    marker_id_1_str = os.environ.get('SCRAP_MARKER_ID_1', 'marker0')
    marker_id_2_str = os.environ.get('SCRAP_MARKER_ID_2', 'marker8')
    marker_coords = load_marker_coordinates(params_yaml_path, marker_id_1_str, marker_id_2_str)

    # 円筒領域フィルタのパラメータ
    region_filter_params = {
        'input_topic': LaunchConfiguration('hesai_input_topic'),
        'input_frame': 'lidar_link',
        'output_frame': 'base_link',
        'ground_frame': 'base_footprint',
        'cylinder_z_min': 0.0,
        'cylinder_z_max': 1.5,
        'debug_output': True,
        'publish_transform_info': False,
        'enabled_on_startup': True,
        'enable_max_z_output': True,
        'max_z_topic': '/scrap_pile_max_height',
        'enable_ground_relative': False,
        'ground_relative_min_height': 0.2,
        'ground_relative_max_height': 2.0,
        'enable_clustering': False,
        'cluster_tolerance': 0.1,
        'min_cluster_size': 200
    }

    if marker_coords:
        region_filter_params['use_dynamic_marker'] = True
        region_filter_params['use_marker_diameter'] = False
        region_filter_params['robot_pose_topic'] = '/robot_pose'
        region_filter_params['marker0_world_x'] = marker_coords['marker1_x']
        region_filter_params['marker0_world_y'] = marker_coords['marker1_y']
        region_filter_params['marker7_world_x'] = marker_coords['marker2_x']
        region_filter_params['marker7_world_y'] = marker_coords['marker2_y']
        region_filter_params['robot_pose_timeout'] = 2.0
    else:
        region_filter_params['use_dynamic_marker'] = False
        region_filter_params['use_marker_diameter'] = False
        region_filter_params['forward_offset'] = 0.0
        region_filter_params['left_offset'] = 1.5
        region_filter_params['fixed_radius'] = 1.0

    scrap_region_filter = Node(
        package='nedo_localization',
        executable='hesai_region_filter_node',
        name='hesai_region_filter_node',
        output='screen',
        parameters=[region_filter_params],
        condition=IfCondition(enable_scrap_detection)
    )

    scrap_highest_point = Node(
        package='nedo_localization',
        executable='highest_point_node',
        name='highest_point_node',
        output='screen',
        parameters=[{'input_topic': '/filtered_points'}],
        condition=IfCondition(enable_scrap_detection)
    )

    # ==== ここから LaunchDescription へアクション登録 ====
    ld.add_action(declare_bag_cmd)
    ld.add_action(declare_contour_cmd)
    ld.add_action(declare_use_livox_cmd)
    ld.add_action(declare_use_hesai_cmd)
    ld.add_action(declare_calib_file_cmd)
    ld.add_action(declare_calib_file_front_cmd)
    ld.add_action(declare_tf_json_left_cmd)
    ld.add_action(declare_tf_json_front_cmd)
    ld.add_action(declare_use_right_sensors_cmd)
    ld.add_action(declare_calib_file_right_cmd)
    ld.add_action(declare_tf_json_right_cmd)
    ld.add_action(declare_landmark_map_file_cmd)
    ld.add_action(LogInfo(msg=["is_contour: ", is_contour]))
    ld.add_action(declare_enable_landmark_marker_cmd)
    ld.add_action(declare_enable_realtime_visualizer_cmd)
    ld.add_action(declare_enable_autonomous_cmd)
    # スクラップ検出用引数
    ld.add_action(declare_enable_scrap_detection_cmd)
    ld.add_action(declare_hesai_input_topic_cmd)
    ld.add_action(declare_scrap_marker_id_1_cmd)
    ld.add_action(declare_scrap_marker_id_2_cmd)
    #ekfノード
    ld.add_action(ekf_node)

    # ==== 静的TF ====
    ld.add_action(static_livox_tf)
    ld.add_action(static_livox_right_tf)
    ld.add_action(lidar_tf_publisher)
    ld.add_action(static_map_tf)
    # TFツリー分岐: スクラップ検出の有無で odom→base_link の経路が変わる
    ld.add_action(static_odom_base_tf)          # スクラップ検出OFF: odom → base_link
    ld.add_action(static_odom_footprint_tf)     # スクラップ検出ON:  odom → base_footprint
    ld.add_action(static_base_footprint_tf)     # スクラップ検出ON:  base_footprint → base_link

    # ==== LiDAR ドライバ ====
    ld.add_action(livox_launch)
    ld.add_action(hesai_launch)

    # ==== メイン処理グループ ====
    ld.add_action(bag_group)
    ld.add_action(landmark_map_server)
    ld.add_action(landmark_marker_publisher)
    ld.add_action(pose_bridge_node)

    # ==== スクラップ検出ノード群 ====
    # enable_scrap_detection=true の場合のみ起動
    if marker_coords:
        ld.add_action(LogInfo(msg=f"[scrap_detection] Using dynamic marker mode with /robot_pose"))
        ld.add_action(LogInfo(msg=f"[scrap_detection] Marker '{marker_coords['marker1_id']}' at ({marker_coords['marker1_x']:.3f}, {marker_coords['marker1_y']:.3f})"))
        ld.add_action(LogInfo(msg=f"[scrap_detection] Marker '{marker_coords['marker2_id']}' at ({marker_coords['marker2_x']:.3f}, {marker_coords['marker2_y']:.3f})"))
    else:
        ld.add_action(LogInfo(msg='[scrap_detection] Marker coordinates not loaded (will use fixed cylinder if scrap detection enabled)'))

    ld.add_action(scrap_region_filter)
    ld.add_action(scrap_highest_point)
    
    return ld