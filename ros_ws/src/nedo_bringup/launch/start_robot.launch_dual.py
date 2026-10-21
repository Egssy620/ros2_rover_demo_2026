import os

# start_robot.launch.py
# 目的:
# - センサ（Livox Avia / Hesai QT128）・認識・自己位置推定・ナビゲーションを起動
# - 引数で構成切り替え（bag再生/実機、Contour/Detector、Hesai/Livoxなど）
# ポイント:
# - Landmark座標は landmark_map_server_node が /landmark_map に配信（Localizer 等が購読）
# - use_livox/use_hesai を両方 true にすれば2系統同時に点群を取得

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
    # backカメラ用キャリブレーションパス
    default_calib_back_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'back_camera_ost.yaml'  # back カメラ 4K用キャリブレーション
    )
    default_tf_json_back_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'calib_back.json'
    )
    # Landmark map (YAML) default path
    # - params.yaml（LM_ids/LM_xs/LM_ys 形式）を既定とする
    default_landmark_yaml_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'params.yaml'
    )
    # Region filter (Hesai) parameter YAML: 切り離されたため不要
    # default_region_filter_yaml_path = os.path.join(...)

    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    livox_launch_path = os.path.join(third_party_install_prefix, 'livox_ros2_avia', 'share', 'livox_ros2_avia')

    # Launch 引数（後段の条件分岐に使用）
    use_bag = LaunchConfiguration('use_bag')
    is_contour = LaunchConfiguration('is_contour')
    use_livox = LaunchConfiguration('use_livox')
    use_hesai = LaunchConfiguration('use_hesai')
    calib_file = LaunchConfiguration('calib_file')
    calib_file_front = LaunchConfiguration('calib_file_front')
    calib_file_back = LaunchConfiguration('calib_file_back')  # backカメラ用
    json_file_left = LaunchConfiguration('json_file_left')
    json_file_front = LaunchConfiguration('json_file_front')
    json_file_back = LaunchConfiguration('json_file_back')  # backカメラ用
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    # region_filter_param_file: 切り離されたため不要
    enable_landmark_marker = LaunchConfiguration('enable_landmark_marker')
    enable_realtime_visualizer = LaunchConfiguration('enable_realtime_visualizer')
    enable_autonomous = LaunchConfiguration('enable_autonomous')
    use_back_sensors = LaunchConfiguration('use_back_sensors')  # back側センサ有効化フラグ

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

    # backカメラ用引数宣言
    declare_calib_file_back_cmd = DeclareLaunchArgument('calib_file_back',
                                           default_value='file://'+default_calib_back_path,
                                           description='back camera calibration yaml file')
    
    declare_tf_json_back_cmd = DeclareLaunchArgument('json_file_back',
                                                default_value=default_tf_json_back_path,
                                                description='back camera to LiDAR calib.json path')

    declare_use_back_sensors_cmd = DeclareLaunchArgument('use_back_sensors',
                                                          default_value='true',
                                                          description='enable back side sensors (camera + LiDAR)')

    # Landmark map YAML (legacy keys: LM_ids, LM_xs, LM_ys)
    declare_landmark_map_file_cmd = DeclareLaunchArgument(
        'landmark_map_file',
        default_value=default_landmark_yaml_path,
        description='YAML file containing landmark coordinates (legacy format supported)'
    )
    # Region filter (Hesai) YAML: 切り離されたため不要
    # declare_region_filter_param_file_cmd = DeclareLaunchArgument(...)

    topic_ns = 'nedo'  # Composableノード群の名前空間（画像・LiDAR処理系）

    # ==== Livox Avia LiDAR ドライバ ====
    # 条件: Not(use_bag) AND use_livox
    # multi_topic=1: 複数LiDARを別々のトピックで配信 (/livox/lidar_XXXXXXXX)
    livox_launch = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        # namespace='livox',  # コメントアウト: 二重namespace回避
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
    
    # ==== QT128 region filter node ====
    # 注意: hesai_region_filter_nodeは切り離されました
    # 別途起動する場合は専用のlaunchファイルまたは手動起動してください
    # ros2 run nedo_localization hesai_region_filter_node --ros-args -p input_topic:=/hesai_ros_driver/pandar_points

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
                    # multi_topic=1で各LiDARが個別トピック(/livox/lidar_XXXXXXXX)で配信される
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
            # back側LiDAR用のDataSyncNode
            LoadComposableNodes(
                condition=IfCondition(AndSubstitution(use_back_sensors, use_livox)),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DataSyncNode',
                        name='syncronizer_back',
                        namespace=topic_ns,
                        parameters=[
                            {'livox_topic': '/livox/lidar_3JEDM4R0014V101'},  # back側Livox Aviaの点群トピック
                            {'use_livox': True},
                            {'output_frame_id': 'livox_back_frame'}  # back側LiDARのframe_idに変更
                        ],
                        remappings=[
                            ('capture', 'capture_back'),           # backカメラの画像を購読
                            ('sync_image', 'sync_image_back'),     # back側同期画像を出力
                            ('sync_point', 'sync_lidar_back')      # back側同期点群を出力
                        ]
                    ),
                    # back側カメラ用のCameraInfo発行ノード
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo_back',
                        namespace=topic_ns,
                        parameters=[
                            {'calib_file': calib_file_back}  # backカメラのキャリブレーションファイル
                        ],
                        remappings=[
                            ('camera_info', 'camera_info_back')
                        ]
                    ),
                    # back側カメラ用の画像圧縮ノード
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp_back',
                        namespace=topic_ns,
                        remappings=[
                            ('debug_image', 'debug_image_back'),
                            ('image_pub', 'image_pub_back')
                        ]
                    )
                ]
            ),
            # カメラキャプチャコンポーネント（bag_group内で起動）
            # 前カメラ（Elgato Facecam Pro）
            # 注意: /dev/nedo_cam_front はudevルールで作成されるシンボリックリンク
            #       udevルールが未適用の場合: sudo cp scripts/99-nedo-cameras.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
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
                            {'device': '/dev/nedo_cam_front'},  # udevルールによる固定シンボリックリンク
                            {'frame_id': 'front_camera_frame'},  # 前カメラフレーム
                            # 4K解像度 (3840x2160) - キャリブレーションと同じ解像度を使用
                            # GStreamer MJPEG format specification
                            {'pipeline': 'v4l2src device=/dev/nedo_cam_front io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'}
                        ]
                    )
                ]
            ),
            # backカメラ（Elgato Facecam Pro 2台目）
            # 条件: use_back_sensors=true AND use_bag=false
            LoadComposableNodes(
                condition=IfCondition(
                    AndSubstitution(
                        NotSubstitution(use_bag),
                        use_back_sensors
                    )
                ),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::NedoCapNode',
                        name='capture_hardware_back',
                        namespace=topic_ns,
                        parameters=[
                            {'device': '/dev/nedo_cam_back'},  # backカメラ用シンボリックリンク
                            {'frame_id': 'back_camera_frame'},  # backカメラフレーム
                            # 4K解像度 (3840x2160) - キャリブレーションと同じ解像度を使用
                            # GStreamer MJPEG format specification
                            {'pipeline': 'v4l2src device=/dev/nedo_cam_back io-mode=2 ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true'}
                        ],
                        remappings=[
                            ('capture', 'capture_back'),
                            ('camera_info', 'camera_info_back')
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
                condition=IfCondition(AndSubstitution(NotSubstitution(is_contour), use_back_sensors)),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DetectorNode',
                        name='point_detector_back',
                        namespace=topic_ns,
                        remappings=[
                            ('image_pub', 'image_pub_back'),
                            ('detected_points', 'detected_points_back')
                        ]
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
            LoadComposableNodes(
                condition=IfCondition(AndSubstitution(is_contour, use_back_sensors)),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourNode',
                        name='contour_detector_back',
                        namespace=topic_ns,
                        remappings=[
                            ('sync_image', 'sync_image_back'),
                            ('image_pub', 'image_pub_back'),
                            ('detected_contours', 'detected_contours_back')
                            
                        ]
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
                            {'lidar_frame': 'livox_frame'},  # Livox Aviaのフレーム名
                            {'camera_frame': 'front_camera_frame'}  # 前カメラフレーム
                        ]
        # 自己位置推定（YAML or /landmark_map を基に処理）
                    ),
                    # 前カメラのみ使用（左カメラのTFノードは削除）
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::LidarCamTF',
                        name='lidar_cam_tf_front',
                        namespace=topic_ns,
                        parameters=[
                            {'json_file_path': json_file_front},
                            {'camera_frame': 'front_camera_frame'},
                            {'lidar_frame': 'livox_frame'}  # Livox Aviaのフレーム名
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourMatcher',
                        name='contour_matcher',
                        namespace=topic_ns,
                        remappings=[
                            ('projected', 'projected'),
                            ('detected_contours', 'detected_contours'),
                            ('detected_points', 'detected_points'),
                            ('/matched_marker', '/matched_marker_front')  # Localizerが購読する同じトピックに出力
                        ]
                    )
                ]
            ),
            # back側センサ用のlocalizationコンポーネント
            LoadComposableNodes(
                condition=IfCondition(use_back_sensors),
                target_container='lidar_container',
                composable_node_descriptions=[
                    # back側Projection
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection_back',
                        namespace=topic_ns,
                        parameters=[
                            {'lidar_frame': 'livox_back_frame'},  # back側Livoxのフレーム名
                            {'camera_frame': 'back_camera_frame'}  # backカメラフレーム
                        ],
                        remappings=[
                            ('sync_point', 'sync_lidar_back'),     # back側同期点群を購読
                            ('camera_info', 'camera_info_back'),   # backカメラ情報を購読
                            ('projected', 'projected_back')        # back側投影結果を出力
                        ]
                    ),
                    # back側LiDAR-カメラTF
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::LidarCamTF',
                        name='lidar_cam_tf_back',
                        namespace=topic_ns,
                        parameters=[
                            {'json_file_path': json_file_back},
                            {'camera_frame': 'back_camera_frame'},
                            {'lidar_frame': 'livox_back_frame'}  # back側Livoxのフレーム名
                        ]
                    ),
                    # backカメラ用ランドマーク（ContourMatcher）
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourMatcher',
                        name='contour_matcher_back',
                        namespace=topic_ns,
                        remappings=[
                            ('projected', 'projected_back'),
                            ('detected_contours', 'detected_contours_back'),
                            ('detected_points', 'detected_points_back'),
                            ('/matched_marker', '/matched_marker_back')  # Localizerが購読する同じトピックに出力
                        ]
                    )
                ]
            ),
            # --- DEBUG IMAGE ---
            # To reduce processing load, ShowProjectionNode is disabled.
            LoadComposableNodes(
            #     # ShowProjectionNode - Front and Back debug windows
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_viz',
                        plugin='nedo_viz::ShowProjectionNode',
                        name='debug_img',
                        namespace=topic_ns,
                        remappings=[
                             # Front camera/LiDAR (無効化するためにダミーにリマップ)
                            ('sync_image', 'sync_image'),
                            ('projected', 'projected'),
                            ('detected_contours', 'detected_contours'),
                            # Back camera/LiDAR (C++ node expects 'right' topics)
                            ('sync_image_right', 'dummy_sync_image_back'),
                            ('projected_right', 'dummy_projected_back'),
                            ('detected_contours_right', 'dummy_detected_contours_back'),
                            ('debug_image_right', 'dummy_debug_image_back'),
                         ]
                     )
                 ]
             ),
            Node(
                package='nedo_localization',
                executable='matched_marker_aggregator_node',
                name='matched_marker_aggregator_node',
                output='screen',
                parameters=[{
                    'front_topic': '/matched_marker_front',
                    'back_topic': '/matched_marker_back',
                    'output_topic': '/matched_marker',
                    'merge_window_ms': 50,
                    'timer_period_ms': 10
                }]
            ),
            Node(
                package='nedo_localization',
                executable = 'localizer_node',
                parameters=[
                    # Landmark map from YAML
                    landmark_map_file,
    # カメラキャプチャコンポーネント（bag_groupの外で独立して起動）
    # - 実機カメラ入力を個別に追加
                    {
                        'lpf_rate': 0.7,
                        'use_tf_pub': True
                    }
                ],
                output='screen'
            ),
            # リアルタイム可視化ノードも追加
                # リアルタイム可視化ノード（任意: 実行ファイルがある場合のみ）
                Node(
                    package='nedo_localization',
                    executable='realtime_visualizer_node',
                    name='realtime_visualizer_node',
    # Landmark マップ配信ノード（YAML→/landmark_map）
                    condition=IfCondition(enable_realtime_visualizer),
                    parameters=[{
                        'publish_rate': 10.0,
                        'trajectory_history_size': 1000
                    }]
                ),
            Node(
                condition = UnlessCondition(use_bag),
                package='nedo_navigation',
                executable='nedo_controller',
                output='screen',
                parameters=[{
                    'max_vel': 2.78,         # 最大並進速度 (m/s) - 時速1.25km
                    'max_angle': 0.1,         # 最大角速度 (rad/s) - 約5.7度/秒
                    'scale_factor': 0.02,     #  - デフォルト0.05から0.02に削減
                    'distance_thresh': 0.3,   # 到着判定距速度スケーリング係数離 (m) - より正確な位置決め
                    'angle_thresh': 0.087     # 到着判定角度 (rad, 約5度) - より正確な姿勢
                }]
            )
        ]
    )
    # 既存の MarkerArray パブリッシャ（任意）
    # - 現在は landmark_map_server_node + 可視化側での購読を推奨
    # カメラキャプチャコンポーネントはbag_group内に移動（capture_containerと同じグループ内でロード）
    # ==== 静的TF & フレーム関係 ====

    # ランドマークマーカー表示ノード（bag_groupの外で独立して起動）
    landmark_map_server = Node(
        package='nedo_localization',
        executable='landmark_map_server_node',
        name='landmark_map_server_node',
        output='screen',
        parameters=[
            # Landmark map from YAML (LM_ids/LM_xs/LM_ys)
            landmark_map_file,
            {
                'publish_rate_hz': 1.0
            }
        ]
    )

    # 既存のマーカーパブリッシャー（任意機能）は残すが、標準はトピック配信へ移行
    landmark_marker_publisher = Node(
        package='nedo_localization',
        executable='landmark_marker_publisher',
        name='landmark_marker_publisher',
        output='screen',
        condition=IfCondition(enable_landmark_marker),
        parameters=[
            # Landmark map from YAML (LM_ids/LM_xs/LM_ys)
            landmark_map_file
        ]
    )

    # カメラの位置関係はキャリブレーション経由（base_link → lidar_link → camera）のみで定義
    # 直接変換（base_link → camera）は削除し、LidarCamTFノードがcalib.jsonから読み込む変換を使用
    # static_camera_tf = Node(...)  # 削除: キャリブレーション経由のみを使用

    # LiDAR frame transforms - 前方Livox
    static_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_link_tf',
        arguments=[
            '0.40', '0.0', '0.70',      # x y z (Livox Avia position from base_link)
            '0', '0', '0', '1',         # qx qy qz qw (no rotation)
            'base_link', 'livox_frame'  # parent: base_link, child: livox_frame
        ],
        output='screen'
    )

    # LiDAR frame transforms - 右側Livox
    static_livox_back_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_back_to_base_link_tf',
        arguments=[
            '-0.5106', '-0.0217', '0.6065',             # X, Y, Z (計算された真の位置)
            '-0.0026', '-0.0027', '1.0000', '0.0007',   # Qx, Qy, Qz, Qw (ピッチ/ロールの微小な傾きを含む)
            'base_link', 'livox_back_frame'
        ],
        output='screen',
        condition=IfCondition(use_back_sensors)
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
            '0.0', '0.0', '0.0',        # x y z
            '0', '0', '0', '1',         # qx qy qz qw (no rotation)
            'map', 'odom'               # parent: map, child: odom
        ],
        output='screen'
    )

    # Odom to base_link transform (初期位置、localizerが動的に更新)
    static_odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=[
            '0.0', '0.0', '0.0',        # x y z
            '0', '0', '0', '1',         # qx qy qz qw (no rotation)
            'odom', 'base_link'         # parent: odom, child: base_link
        ],
        output='screen'
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
    ld.add_action(declare_use_back_sensors_cmd)
    ld.add_action(declare_calib_file_back_cmd)
    ld.add_action(declare_tf_json_back_cmd)
    ld.add_action(declare_landmark_map_file_cmd)
    # declare_region_filter_param_file_cmd: 切り離されたため不要
    ld.add_action(LogInfo(msg=["is_contour: ", is_contour]))
    ld.add_action(declare_enable_landmark_marker_cmd)
    ld.add_action(declare_enable_realtime_visualizer_cmd)
    ld.add_action(declare_enable_autonomous_cmd)
    # static_camera_tf は削除: カメラの位置関係はキャリブレーション経由（LidarCamTFノード）のみで定義
    ld.add_action(static_livox_tf)
    ld.add_action(static_livox_back_tf)
    ld.add_action(lidar_tf_publisher)
    ld.add_action(static_map_tf)
    ld.add_action(static_odom_base_tf)
    ld.add_action(livox_launch)
    ld.add_action(hesai_launch)
    # qt128_region_filter は切り離されました（上記コメント参照）
    ld.add_action(bag_group)
    ld.add_action(landmark_map_server)
    ld.add_action(landmark_marker_publisher)
    
    # 自律走行シーケンス管理ノード（簡易版）
    autonomous_sequence_node = Node(
        package='nedo_navigation',
        executable='simple_autonomous_node',
        name='simple_autonomous_node',
        output='screen',
            condition=IfCondition(AndSubstitution(NotSubstitution(use_bag), enable_autonomous)),
        parameters=[{
            'arrival_threshold': 1.0,  # 到着判定距離閾値 (m)
            'wait_duration': 5.0       # 待機時間 (s)
        }]
    )
    ld.add_action(autonomous_sequence_node)
    # camera_capture_componentはbag_group内に移動済み
    
    return ld