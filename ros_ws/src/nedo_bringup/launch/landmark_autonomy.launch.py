"""
landmark_autonomy.launch.py

目的
- ランドマークに基づく自己位置推定と自律走行に必要な最小構成のみを起動

想定ユースケース
- 実機またはbag再生で、Livoxを主に使いながら（任意）認識・投影・自己位置推定・制御を確認したいとき。

主要なLaunch引数（上書き可）
- use_bag (bool): bag再生かどうか（falseで実機）。
- is_contour (bool): Contour系かDetector系かの選択。
- use_livox (bool): Livoxドライバを起動するか。
- calib_file (str): カメラキャリブ（front_camera_ost.yaml）。
- json_file_path (str): カメラ- LiDAR TF（calib.json）。
- landmark_map_file (str): ランドマップYAML（params.yaml, LM_* 形式）。
- enable_landmark_marker (bool): 既存のMarkerArrayパブリッシャを有効化。
- enable_realtime_visualizer (bool): リアルタイム可視化ノードを有効化。
- enable_autonomous (bool): 簡易自律走行ノードを有効化。

起動ノード概要
- Livoxドライバ（任意, Not(use_bag) ∧ use_livox）
- capture_container（Compress/Caminfo/NedoCap/DataSync, is_contourでDetector/Contourを分岐）
- lidar_container（Projection/LidarCamTF/ContourMatcher, bag時のみShowProjection）
- localizer_node（landmark_map_fileを参照; /landmark_map購読）
- landmark_map_server_node（YAML→/landmark_map を transient_local で配信）
- （任意）landmark_marker_publisher / realtime_visualizer_node / simple_autonomous_node
- 静的TF（camera→base_link, livox_frame→base_link, map→odom, odom→base_link）

主なトピック
- /livox/lidar（Livox点群, 任意）
- /landmark_map（ランドマーク配信）

備考
- Hesai(QT128)や /filtered_points を用いるスクラップ検出は scrap_detection.launch.py を使用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # LaunchDescription: 本launchで実行する全アクション（ノード起動/引数宣言など）のコンテナ
    ld = LaunchDescription()

    # =============================
    # デフォルトファイルパス定義
    # - カメラキャリブYAML
    # - LiDAR-カメラ外部パラメータJSON
    # - ランドマークマップYAML（LM_* 定義を含む）
    # =============================
    default_calib_path = os.path.join(
        get_package_share_directory('nedo_bringup'), 'config', 'front_camera_ost.yaml'
    )
    default_tf_json_path = os.path.join(
        get_package_share_directory('nedo_bringup'), 'config', 'calib.json'
    )
    default_landmark_yaml_path = os.path.join(
        get_package_share_directory('nedo_bringup'), 'config', 'params.yaml'
    )

    # =============================
    # Launch引数（上書き可）
    # - use_bag: bag再生かどうか
    # - is_contour: Contour系(=true) / Detector系(=false) の分岐
    # - use_livox: 実機でLivoxドライバを起動するか
    # - calib_file/json_file_path/landmark_map_file: 各種設定ファイルパス
    # - enable_*: 可視化/自律の任意ノードトグル
    # =============================
    use_bag = LaunchConfiguration('use_bag')
    is_contour = LaunchConfiguration('is_contour')
    use_livox = LaunchConfiguration('use_livox')
    calib_file = LaunchConfiguration('calib_file')
    json_file = LaunchConfiguration('json_file_path')
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    enable_landmark_marker = LaunchConfiguration('enable_landmark_marker')
    enable_realtime_visualizer = LaunchConfiguration('enable_realtime_visualizer')
    enable_autonomous = LaunchConfiguration('enable_autonomous')

    ld.add_action(DeclareLaunchArgument('use_bag', default_value='false'))
    ld.add_action(DeclareLaunchArgument('is_contour', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_livox', default_value='true'))
    ld.add_action(DeclareLaunchArgument('calib_file', default_value='file://' + default_calib_path))
    ld.add_action(DeclareLaunchArgument('json_file_path', default_value=default_tf_json_path))
    ld.add_action(DeclareLaunchArgument('landmark_map_file', default_value=default_landmark_yaml_path))
    ld.add_action(DeclareLaunchArgument('enable_landmark_marker', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_realtime_visualizer', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_autonomous', default_value='false'))
    ld.add_action(LogInfo(msg=["[landmark_autonomy] is_contour: ", is_contour]))

    # すべてのComposableNodeに共通で付与する名前空間
    topic_ns = 'nedo'

    # =============================
    # Livoxドライバ（任意）
    # - 条件: 実機(Not use_bag) かつ use_livox==true のときのみ起動
    # - 出力: /livox/lidar（frame_id=livox_frame）
    # - 主なパラメータ:
    #   * publish_freq: パブリッシュ周波数(Hz)
    #   * frame_id: 点群の基準フレーム
    #   * xfer_format/multi_topic/data_src/output_data_type などはLivox固有設定
    # =============================
    livox_launch = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(AndSubstitution(NotSubstitution(use_bag), use_livox)),
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'}
        ]
    )

    # =============================
    # コンポーネントコンテナ
    # - capture_container: 画像系/同期系/特徴抽出(Contour or Detector)
    # - lidar_container  : 投影/TF/マッチング/（bag時のみ可視化）
    # =============================
    capture_container = Node(
        package='rclcpp_components', executable='component_container_mt', name='capture_container', output='screen'
    )
    lidar_container = Node(
        package='rclcpp_components', executable='component_container_mt', name='lidar_container', output='screen'
    )

    # capture系Composableのロード
    # - CompressNode: 画像圧縮
    # - CaminfoNode : calib_file から CameraInfo を提供
    # - NedoCapNode : カメラ等ハードウェアのキャプチャ
    # - DataSyncNode: LiDARとカメラの時刻同期（/livox/lidar, use_livoxで分岐）
    load_capture = LoadComposableNodes(
        target_container='capture_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_capture', plugin='nedo_capture::CompressNode', name='cap_comp', namespace=topic_ns
            ),
            ComposableNode(
                package='nedo_capture', plugin='nedo_capture::CaminfoNode', name='caminfo', namespace=topic_ns,
                parameters=[{'calib_file': calib_file}]
            ),
            ComposableNode(
                package='nedo_capture', plugin='nedo_capture::NedoCapNode', name='capture_hardware', namespace=topic_ns
            ),
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::DataSyncNode', name='syncronizer', namespace=topic_ns,
                parameters=[{'livox_topic': '/livox/lidar'}, {'use_livox': use_livox}]
            )
        ]
    )

    # 特徴抽出ブランチの切替え
    # - is_contour=false → DetectorNode をロード
    load_detector = LoadComposableNodes(
        condition=UnlessCondition(is_contour),
        target_container='capture_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::DetectorNode', name='point_detector', namespace=topic_ns
            )
        ]
    )
    # - is_contour=true → ContourNode をロード
    load_contour = LoadComposableNodes(
        condition=IfCondition(is_contour),
        target_container='capture_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::ContourNode', name='contour_detector', namespace=topic_ns
            )
        ]
    )

    # LiDAR系Composableのロード
    # - ProjectionNode: 可視化/後段処理向けの投影前処理
    # - LidarCamTF    : json_file_path からカメラ- LiDAR の外部パラメータを適用
    # - ContourMatcher: 検出結果とランドマークの対応付け
    load_lidar = LoadComposableNodes(
        target_container='lidar_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::ProjectionNode', name='projection', namespace=topic_ns
            ),
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::LidarCamTF', name='lidar_cam_tf', namespace=topic_ns,
                parameters=[{'json_file_path': json_file}]
            ),
            ComposableNode(
                package='nedo_localization', plugin='nedo_localization::ContourMatcher', name='contour_matcher', namespace=topic_ns
            )
        ]
    )

    # bag再生時のみの投影可視化（画像デバッグ）
    load_show_projection_if_bag = LoadComposableNodes(
        condition=IfCondition(use_bag),
        target_container='lidar_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_viz', plugin='nedo_viz::ShowProjectionNode', name='debug_img', namespace=topic_ns
            )
        ]
    )

    # =============================
    # 位置推定/ランドマーク配信/可視化
    # - localizer: ランドマークに基づく自己位置推定（lpf_rateで平滑化、TF出力）
    # - landmark_map_server: YAMLのLM_*を /landmark_map に transient_local で配信
    # - landmark_marker_publisher: RViz向けMarkerArray（任意）
    # - realtime_visualizer: 経路/軌跡の簡易可視化（任意）
    # =============================
    localizer = Node(
        package='nedo_localization', executable='localizer_node', output='screen',
        parameters=[landmark_map_file, {'lpf_rate': 0.7, 'use_tf_pub': True}]
    )

    landmark_map_server = Node(
        package='nedo_localization', executable='landmark_map_server_node', name='landmark_map_server_node', output='screen',
        parameters=[landmark_map_file, {'publish_rate_hz': 1.0}]
    )

    landmark_marker_publisher = Node(
        package='nedo_localization', executable='landmark_marker_publisher', output='screen',
        condition=IfCondition(enable_landmark_marker), parameters=[landmark_map_file]
    )

    realtime_visualizer = Node(
        package='nedo_localization', executable='realtime_visualizer_node', name='realtime_visualizer_node', output='screen',
        condition=IfCondition(enable_realtime_visualizer), parameters=[{'publish_rate': 10.0, 'trajectory_history_size': 1000}]
    )

    # 実機時のみの車両コントローラ（bag再生では無効）
    controller = Node(
        condition=UnlessCondition(use_bag), package='nedo_navigation', executable='nedo_controller', output='screen'
    )

    # =============================
    # 静的TFの配信
    # - camera→base_link: カメラの取付位置
    # - livox_frame→base_link: Livoxの取付位置
    # - map→odom, odom→base_link: 初期は0姿勢の仮置き（localizerが更新/利用）
    #   引数は x y z qx qy qz qw の順
    # =============================
    static_camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_to_base_link_tf', output='screen',
        arguments=['1.5', '0.0', '1.0', '0', '0', '0', '1', 'base_link', 'camera']
    )
    static_livox_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='livox_to_base_link_tf', output='screen',
        arguments=['0.0', '0.0', '0.5', '0', '0', '0', '1', 'base_link', 'livox_frame']
    )
    static_map_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_tf', output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'map', 'odom']
    )
    static_odom_base_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='odom_to_base_link_tf', output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    # 自律走行シーケンス（任意）
    # - 条件: 実機(Not use_bag) かつ enable_autonomous==true
    # - arrival_threshold: 目標到達判定[m]
    # - wait_duration: 到達後のウェイト[s]
    autonomous_sequence_node = Node(
        package='nedo_navigation', executable='simple_autonomous_node', name='simple_autonomous_node', output='screen',
        condition=IfCondition(AndSubstitution(NotSubstitution(use_bag), enable_autonomous)),
        parameters=[{'arrival_threshold': 1.0, 'wait_duration': 5.0}]
    )

    # =============================
    # アクション登録順序
    # - コンテナ → Composable群 → 推定/配信/可視化 → コントローラ/TF/自律
    # =============================
    ld.add_action(livox_launch)
    ld.add_action(capture_container)
    ld.add_action(lidar_container)
    ld.add_action(load_capture)
    ld.add_action(load_detector)
    ld.add_action(load_contour)
    ld.add_action(load_lidar)
    ld.add_action(load_show_projection_if_bag)
    ld.add_action(localizer)
    ld.add_action(landmark_map_server)
    ld.add_action(landmark_marker_publisher)
    ld.add_action(realtime_visualizer)
    ld.add_action(controller)
    ld.add_action(static_camera_tf)
    ld.add_action(static_livox_tf)
    ld.add_action(static_map_tf)
    ld.add_action(static_odom_base_tf)
    ld.add_action(autonomous_sequence_node)

    return ld
