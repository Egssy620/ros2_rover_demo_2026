"""
bin_landmark_localization.launch.py

目的
- BIN方式のランドマーク検出器 (`cylinder_landmark_detector_bin_node`) を入力源に、
  自己位置推定 (localizer_node) と地図配信 (landmark_map_server_node) を最小構成で起動する。
- カメラ系・ContourMatcher・aggregator・dual-LiDAR は一切使わない。

LiDAR センサ切替 (sensor_type)
- `sensor_type:=avia`: Livox Avia (livox_ros2_avia, リポジトリ同梱フォーク)
    package           : livox_ros2_avia
    topic             : /livox/lidar_3JEDM4V001DY881  (multi_topic=1)
    frame_id          : livox_frame
    設定 JSON         : third_party/install/livox_ros2_avia/share/livox_ros2_avia/config/livox_lidar_config.json
- `sensor_type:=ot128` (デフォルト): Hesai OT128 (hesai_ros_driver)
    package           : hesai_ros_driver
    namespace / topic : /hesai_ot128/lidar_points
    frame_id          : hesai_ot128
    設定 YAML         : config/config_hesai_ot128.yaml
- `sensor_type:=qt128`: Hesai QT128 (hesai_ros_driver)
    package           : hesai_ros_driver
    namespace / topic : /hesai_qt128/lidar_points
    frame_id          : hesai_qt128
    設定 YAML         : config/config_hesai_qt128.yaml
両 Hesai とも デバイスIP 192.168.1.201 / UDP 2368 / PTC 9347 (工場出荷時のまま)。
差分は frame_id と namespace だけなので、1台ずつ差し替える運用では
sensor_type を切替えるだけでよい。
Avia ドライバは third_party/install/livox_ros2_avia を参照 (start_robot.launch.py と同経路)。

QT128 と OT128 の同時運用について
- namespace・トピック・frame_id は既に分離済みだが、**現状の config では
  2台同時起動はできない**。両機とも工場出荷時の 192.168.1.201 / UDP 2368 を
  指しているため。also_start_qt128:=true 等を立てる前に下記の変更が必要。
- 手順: QT128 実機を Web UI (http://192.168.1.201) で 192.168.1.202 /
  UDP 2369 に変更し、config_hesai_qt128.yaml の [同時運用] と書かれた
  2箇所を同じ値に書き換える。OT128 は .201/2368 のまま据え置く。
- この変更をせずに両方を 2368 に向けたままにすると、libhesai の受信ソケットは
  INADDR_ANY:2368 に SO_REUSEADDR で bind し、送信元IPによる選別は
  device_udp_src_port>=1024 指定時しか効かないため、両ドライバが両LiDARの
  パケットを混ぜてパースし点群が破綻する (config_hesai_*.yaml 冒頭に詳述)。

起動ノード
- LiDAR ドライバ              : sensor_type で選んだ1台 (+ also_start_* で併走)
- BIN ランドマーク検出器     : /matched_marker を base_link 系で publish
- landmark_map_server_node   : params.yaml から /landmark_map を配信
- localizer_node             : SVD で /robot_pose と TF(map→base_link) を出力
- 静的TF                     : base_link→各LiDARフレーム, map→odom, odom→base_link
- landmark_map_visualizer    : enable_landmark_viz=true で起動 (RViz用 /landmark_map_markers)
- hesai_region_filter        : enable_scrap_detection=true でスクラップROIフィルタ起動
- highest_point              : enable_scrap_detection=true で /filtered_points の最高点抽出

主要 LaunchArgument
- sensor_type (str)            : 'avia' | 'qt128' | 'ot128' (デフォルト 'ot128')
                                 検出器の入力に使う LiDAR を選ぶ
- lidar_input_topic (str)      : 検出器が購読するLiDAR点群トピック
                                  未指定なら sensor_type から自動決定
- lidar_input_frame (str)      : 点群フレーム
                                  未指定なら sensor_type から自動決定
- landmark_map_file (str)      : ランドマーク座標YAML (LM_ids / LM_xs / LM_ys 形式)
- enable_landmark_viz (bool)   : landmark_map_visualizer_node を起動するか (デフォルト true)
- enable_scrap_detection (bool): hesai_region_filter + highest_point を起動するか (デフォルト false)
- detector_params_file (str)   : 検出器パラメータYAML (実験中の調整用)
- also_start_avia/qt128/ot128   : sensor_type 以外のドライバも併走させるか
- avia_config_file (str)       : Livox Avia (SDK2) JSON 設定
- qt128_config_file (str)      : Hesai QT128 YAML 設定
- ot128_config_file (str)      : Hesai OT128 YAML 設定
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


# このローンチは install 後の launch ディレクトリから実行されるため、
# 4つ上がリポジトリルート (start_robot.launch.py と同じ流儀)。
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))


def generate_launch_description():
    ld = LaunchDescription()

    # ---- ワークスペース絶対パス ----
    pkg_share = get_package_share_directory('nedo_bringup')

    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    livox_share_dir = os.path.join(
        third_party_install_prefix, 'livox_ros2_avia', 'share', 'livox_ros2_avia')

    default_landmark_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    default_detector_yaml = os.path.join(pkg_share, 'config', 'detector_params.yaml')
    default_avia_json     = os.path.join(livox_share_dir, 'config', 'livox_lidar_config.json')
    default_qt128_yaml    = os.path.join(pkg_share, 'config', 'config_hesai_qt128.yaml')
    default_ot128_yaml    = os.path.join(pkg_share, 'config', 'config_hesai_ot128.yaml')

    # ---- LaunchArgument 宣言 ----
    sensor_type = LaunchConfiguration('sensor_type')
    lidar_input_topic = LaunchConfiguration('lidar_input_topic')
    lidar_input_frame = LaunchConfiguration('lidar_input_frame')
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    detector_params_file = LaunchConfiguration('detector_params_file')
    avia_config_file = LaunchConfiguration('avia_config_file')
    qt128_config_file = LaunchConfiguration('qt128_config_file')
    ot128_config_file = LaunchConfiguration('ot128_config_file')
    enable_landmark_viz = LaunchConfiguration('enable_landmark_viz')
    enable_scrap_detection = LaunchConfiguration('enable_scrap_detection')
    detector_output_topic = LaunchConfiguration('detector_output_topic')
    scrap_lidar_input_topic = LaunchConfiguration('scrap_lidar_input_topic')
    scrap_lidar_input_frame = LaunchConfiguration('scrap_lidar_input_frame')

    ld.add_action(DeclareLaunchArgument(
        'detector_output_topic', default_value='/matched_marker',
        description='検出器が PoseStampedArray を publish するトピック (localizer 入力)'
    ))
    # スクラップROIは 360° 走査前提なので Hesai 系 (現状 QT128, 将来 OT128) を想定。
    # 検出器入力 (auto_topic) とは独立にして、sensor_type=avia でも Hesai を別
    # ドライバで併用する構成を許す。
    #
    # default は本launch内で OT128 ドライバを起動する場合のトピック名
    # (namespace='hesai_ot128' を付与しているため /hesai_ot128/lidar_points)。
    # QT128 側でROIを取りたい場合は
    #   scrap_lidar_input_topic:=/hesai_qt128/lidar_points
    #   scrap_lidar_input_frame:=hesai_qt128
    # 既存の「別ターミナルで ros2 run hesai_ros_driver」(namespaceなし) 運用なら
    #   scrap_lidar_input_topic:=/lidar_points
    # で上書きすること。
    ld.add_action(DeclareLaunchArgument(
        'scrap_lidar_input_topic', default_value='/hesai_ot128/lidar_points',
        description='スクラップROIフィルタの入力 PointCloud2 (既定は OT128 @.201)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scrap_lidar_input_frame', default_value='hesai_ot128',
        description='スクラップROIフィルタの入力フレーム (QT128=hesai_qt128 / OT128=hesai_ot128)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'sensor_type', default_value='ot128',
        description=("検出器の入力に使う LiDAR ('avia' | 'qt128' | 'ot128')。"
                     "デフォルトは ot128 (Hesai OT128 @192.168.1.201, 工場出荷時のまま)。")
    ))
    # 検出器入力は sensor_type が決めるが、他のドライバも併走させたい場合のフラグ。
    # 例: sensor_type=qt128 (検出はQT128) のまま also_start_ot128=true で OT128 点群も
    #     /hesai_ot128/lidar_points に publish させる (記録/比較/スクラップ検出用)。
    # 注意: Hesai 2台 (qt128/ot128) は現状どちらも .201/2368 を指しているため
    # 同時起動できない。同時運用する場合は冒頭 docstring の手順で QT128 を
    # .202/2369 に移すこと。Avia との併走は制約なし。
    ld.add_action(DeclareLaunchArgument(
        'also_start_avia', default_value='false',
        description='sensor_type に関わらず Livox Avia ドライバも同時起動するか'
    ))
    ld.add_action(DeclareLaunchArgument(
        'qt128_static_tf', default_value='true',
        description=('base_link→hesai_qt128 の仮TF (0.40, 0, 0.70) を出すか。'
                     'scrap_pickup_test / scrap_pile_skirt と同時起動する時は false '
                     '(lidar_tf_publisher_node の実測値を使う)')
    ))
    ld.add_action(DeclareLaunchArgument(
        'also_start_qt128', default_value='false',
        description='sensor_type に関わらず Hesai QT128 ドライバも同時起動するか'
    ))
    ld.add_action(DeclareLaunchArgument(
        'also_start_ot128', default_value='false',
        description='sensor_type に関わらず Hesai OT128 ドライバも同時起動するか'
    ))
    # lidar_input_topic / lidar_input_frame は sensor_type から自動決定。
    # 明示的に上書きしたい場合のみユーザが指定する (空文字なら自動)。
    ld.add_action(DeclareLaunchArgument(
        'lidar_input_topic', default_value='',
        description='LiDAR PointCloud2 topic. 空なら sensor_type から自動。'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_input_frame', default_value='',
        description='LiDAR frame id. 空なら sensor_type から自動。'
    ))
    ld.add_action(DeclareLaunchArgument(
        'avia_config_file', default_value=default_avia_json,
        description='Livox Avia (livox_ros2_avia) 設定 JSON (third_party 同梱の livox_lidar_config.json)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'qt128_config_file', default_value=default_qt128_yaml,
        description='Hesai QT128 (hesai_ros_driver) 設定 YAML'
    ))
    ld.add_action(DeclareLaunchArgument(
        'ot128_config_file', default_value=default_ot128_yaml,
        description='Hesai OT128 (hesai_ros_driver) 設定 YAML'
    ))

    # sensor_type → デフォルト topic/frame の自動決定
    # avia 側は livox_ros2_avia (multi_topic=1) のシリアル付きトピック名に合わせる。
    # 既存 start_robot_detector.launch.py のデフォルトと同値。
    _TOPIC_BY_SENSOR = {
        'avia': '/livox/lidar_3JEDM4V001DY881',
        'qt128': '/hesai_qt128/lidar_points',
        'ot128': '/hesai_ot128/lidar_points',
    }
    _FRAME_BY_SENSOR = {
        'avia': 'livox_frame',
        'qt128': 'hesai_qt128',
        'ot128': 'hesai_ot128',
    }
    auto_topic = PythonExpression([
        "'", lidar_input_topic, "' if '", lidar_input_topic, "' else ",
        repr(_TOPIC_BY_SENSOR), ".get('", sensor_type, "', ",
        repr(_TOPIC_BY_SENSOR['ot128']), ")"
    ])
    auto_frame = PythonExpression([
        "'", lidar_input_frame, "' if '", lidar_input_frame, "' else ",
        repr(_FRAME_BY_SENSOR), ".get('", sensor_type, "', ",
        repr(_FRAME_BY_SENSOR['ot128']), ")"
    ])
    # ドライバ起動条件: sensor_type で選ばれた側 OR also_start_* が true。
    also_start_avia = LaunchConfiguration('also_start_avia')
    also_start_qt128 = LaunchConfiguration('also_start_qt128')
    also_start_ot128 = LaunchConfiguration('also_start_ot128')
    start_avia = PythonExpression([
        "'", sensor_type, "' == 'avia' or '", also_start_avia, "'.lower() == 'true'"])
    start_qt128 = PythonExpression([
        "'", sensor_type, "' == 'qt128' or '", also_start_qt128, "'.lower() == 'true'"])
    start_ot128 = PythonExpression([
        "'", sensor_type, "' == 'ot128' or '", also_start_ot128, "'.lower() == 'true'"])
    ld.add_action(DeclareLaunchArgument(
        'landmark_map_file',
        default_value=default_landmark_yaml,
        description='YAML containing LM_ids / LM_xs / LM_ys'
    ))
    ld.add_action(DeclareLaunchArgument(
        'detector_params_file',
        default_value=default_detector_yaml,
        description='検出器パラメータYAML (実験中の調整用)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'enable_landmark_viz',
        default_value='true',
        description='landmark_map_visualizer_node を起動するか'
    ))
    ld.add_action(DeclareLaunchArgument(
        'enable_scrap_detection',
        default_value='false',
        description='hesai_region_filter + highest_point を起動するか'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_dynamic_marker',
        default_value='false',
        description=('hesai_region_filter の ROI 中心を両端ランドマーク座標で動的に追従するか。 '
                     'true 時は landmark_map_file から両端 LM の world 座標を読み込んで '
                     'region_filter に DI する (LM 座標を二箇所に書かないため)。')
    ))
    ld.add_action(LogInfo(msg=['[bin_landmark_localization] map=', landmark_map_file]))
    ld.add_action(LogInfo(msg=['[bin_landmark_localization] sensor_type=', sensor_type,
                                ' topic=', auto_topic, ' frame=', auto_frame]))

    # ---- Livox Avia ドライバ (livox_ros2_avia, 本リポジトリ同梱) ----
    # ドライバ実体: third_party/install/livox_ros2_avia
    # 既存 start_robot.launch.py / start_robot_detector.launch.py と同じ参照方式。
    # multi_topic=1 でデバイス毎に topic を分離 (/livox/lidar_3JEDM4V001DY881)。
    # namespace は付けない (Hesai 側で namespace='hesai' を付けているため衝突なし)。
    livox_driver = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(start_avia),
        parameters=[
            {'xfer_format': 0},      # PointCloud2 出力
            {'multi_topic': 1},      # デバイス毎に topic を分離
            {'data_src': 0},         # ライブ受信
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': avia_config_file},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # ---- Hesai QT128 ドライバ ----
    # namespace 'hesai_qt128' で Avia / OT128 と完全分離
    # (/hesai_qt128/lidar_points, /hesai_qt128/lidar_imu)。
    # デバイスIP 192.168.1.201 / UDP 2368 / PTC 9347 は config_hesai_qt128.yaml 側で固定。
    qt128_driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        namespace='hesai_qt128',
        output='screen',
        condition=IfCondition(start_qt128),
        parameters=[{'config_path': qt128_config_file}],
    )

    # ---- Hesai OT128 ドライバ ----
    # namespace 'hesai_ot128' で Avia / QT128 と完全分離
    # (/hesai_ot128/lidar_points, /hesai_ot128/lidar_imu)。
    # デバイスIP 192.168.1.202 / UDP 2369 / PTC 9347 は config_hesai_ot128.yaml 側で固定。
    # ノード名は QT128 と同じ 'hesai_ros_driver_node' だが namespace が異なるため
    # 完全修飾名 (/hesai_qt128/... と /hesai_ot128/...) が衝突しない。
    ot128_driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        namespace='hesai_ot128',
        output='screen',
        condition=IfCondition(start_ot128),
        parameters=[{'config_path': ot128_config_file}],
    )

    # ---- BIN ランドマーク検出器 ----
    # parameters の評価順: yaml → inline dict の順で上書き。
    # I/O系 (topic/frame) は launch arg から渡したいため inline で後段に置く。
    # 検出器の物理仕様・フィルタ閾値などは detector_params_file (yaml) で管理し、
    # 実験中は yaml を書き換えるだけで再起動で反映できる。
    bin_detector = Node(
        package='nedo_localization',
        executable='cylinder_landmark_detector_bin_node',
        name='cylinder_landmark_detector_bin',
        output='screen',
        parameters=[
            detector_params_file,
            {
                'input_topic': auto_topic,
                'input_frame_id': auto_frame,
                'output_topic': detector_output_topic,
                'output_frame_id': 'base_link',
            },
        ],
    )

    # ---- ランドマーク地図配信 ----
    map_server = Node(
        package='nedo_localization',
        executable='landmark_map_server_node',
        name='landmark_map_server_node',
        output='screen',
        parameters=[landmark_map_file, {'publish_rate_hz': 1.0}],
    )

    # ---- 自己位置推定 (SVD) ----
    localizer = Node(
        package='nedo_localization',
        executable='localizer_node',
        output='screen',
        parameters=[landmark_map_file, {'lpf_rate': 0.7}],
    )

    # ---- NEDO Controller ----
    # nedo_navigation/src/nedo_controller.cpp: /robot_pose と /goal_pose を購読し
    # /cmd_vel を publish する。トピック名はノード内でハードコード。
    # 動作には別途 /goal_pose の publisher が必要 (このlaunchには含まない)。
    # 現在は無効化中。有効化する場合はこのブロックと末尾の
    # ld.add_action(nedo_controller) の両方のコメントを外すこと
    # (片方だけ外すと NameError で launch 全体が失敗する)。
    # nedo_controller = Node(
    #     package='nedo_navigation',
    #     executable='nedo_controller',
    #     name='nedo_controller',
    #     output='screen',
    #     parameters=[{
    #         'distance_thresh': 0.05,
    #         'angle_thresh': 1.0 * 3.14159265 / 180.0,
    #         'scale_factor': 0.05,
    #         'max_vel': 0.5,
    #         'max_angle': 0.5,
    #     }],
    # )

    # ---- 静的TF ----
    # base_link → <LiDARフレーム> (取付位置)
    # 取付値は既存 start_robot.launch.py と同値。LiDAR ごとにフレーム名のみ差し替え。
    # NOTE: ノード名は LiDAR ごとに別名にすること。以前は avia/ot128 とも
    # 'lidar_to_base_link_tf' だったため、2台を併走させるとノード名が重複した。
    static_lidar_tf_avia = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='avia_to_base_link_tf',
        arguments=['0.40', '0.0', '0.70', '0', '0', '0', '1',
                   'base_link', 'livox_frame'],
        output='screen',
        condition=IfCondition(start_avia),
    )
    # qt128_static_tf:=false のとき出さない。scrap_pile_skirt.launch.py (lidar_tf_publisher_node,
    # 2026-09-24 実測値) と同時起動すると base_link→hesai_qt128 が2つ出て、どちらが効くか決まらないため。
    static_lidar_tf_qt128 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='qt128_to_base_link_tf',
        arguments=['0.40', '0.0', '0.70', '0', '0', '0', '1',
                   'base_link', 'hesai_qt128'],
        output='screen',
        condition=IfCondition(PythonExpression([
            start_qt128, " and '", LaunchConfiguration('qt128_static_tf'), "'.lower() == 'true'"])),
    )
    # OT128 は機体に対して yaw +90° で取り付けられている (2026-09-28 実機で確認:
    # 台車を左へ動かすと LM が前後方向に、後ろへ動かすと左右方向にずれた = 検出が −90° 回っていた)。
    # 旧値 yaw 0 のままだと /robot_pose の移動方向が 90° 回り、移動ノードが目標に収束しない。
    # 並進 (0.40, 0, 0.70) は未実測の仮値のまま。四元数 (0, 0, sin45°, cos45°) = yaw +90°
    static_lidar_tf_ot128 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ot128_to_base_link_tf',
        arguments=['0.40', '0.0', '0.70', '0', '0', '0.70710678', '0.70710678',
                   'base_link', 'hesai_ot128'],
        output='screen',
        condition=IfCondition(start_ot128),
    )
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen',
    )
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link'],
        output='screen',
    )

    # ---- ランドマーク地図可視化 (RViz用) ----
    # /landmark_map を購読し、/landmark_map_markers (MarkerArray) を1Hzで配信。
    #
    # 旧実装は landmark_ids/region_ids を `[]` (空list) で上書きしてデフォルト
    # marker1..marker8 / region1 のフォールバック描画を抑止していたが、
    # ROS2 Humble の launch dict は空list の型推論ができず起動が失敗する
    # (TypeError: got '()' of type tuple)。
    #
    # 代わりに、map_server が起動直後に /landmark_map を transient_local で
    # publish する (landmark_map_server_node.cpp 修正済) ため、visualizer は
    # /landmark_map を即座に受信して params のフォールバックは事実上動かない。
    # 起動瞬間の僅かな時間だけ (0,0,0) に marker1..8 が描かれる可能性はある。
    landmark_map_viz = Node(
        package='nedo_localization',
        executable='landmark_map_visualizer_node',
        name='landmark_map_visualizer_node',
        output='screen',
        condition=IfCondition(enable_landmark_viz),
    )

    # ---- スクラップ山ROIフィルタ ----
    # use_dynamic_marker:=false (default): legacy固定円柱モード (forward/left_offset=0,
    #   fixed_radius=5m)。配線確認用。
    # use_dynamic_marker:=true: 両端ランドマーク座標 (landmark_map_file 由来) を
    #   region_filter に DI して ROI を追従。LM 座標を二箇所に書かない (R10 解消)。
    #
    # parameters を context 依存で組み立てる必要があるため、Node 生成は
    # OpaqueFunction の中で行う (Static な Node 構文では yaml 動的読み込みが書けない)。
    def _make_region_filter(context, *args, **kwargs):
        use_dyn = LaunchConfiguration('use_dynamic_marker').perform(context).lower() == 'true'
        params = {
            'input_topic': LaunchConfiguration('scrap_lidar_input_topic').perform(context),
            'input_frame': LaunchConfiguration('scrap_lidar_input_frame').perform(context),
            'output_frame': 'base_link',
            # NOTE: 障害物検知系 (lidar_pkg/scan_shadow_filter_node) も /filtered_points を
            # publish するため、スクラップ系は別名 (/scrap_filtered_points) を使う。
            'output_topic': '/scrap_filtered_points',
            'use_dynamic_marker': use_dyn,
            'use_marker_diameter': False,
            'forward_offset': 0.0,
            'left_offset': 0.0,
            'fixed_radius': 5.0,
            'cylinder_z_min': -1.0,
            'cylinder_z_max': 3.0,
            'enable_ground_relative': False,
            'enabled_on_startup': True,
            'debug_output': False,
        }
        actions = []
        if use_dyn:
            yaml_path = LaunchConfiguration('landmark_map_file').perform(context)
            try:
                with open(yaml_path) as f:
                    doc = yaml.safe_load(f) or {}
                ros_params = doc.get('/**', {}).get('ros__parameters', {})
                lm_xs = ros_params.get('LM_xs') or []
                lm_ys = ros_params.get('LM_ys') or []
                lm_ids = ros_params.get('LM_ids') or []
                if len(lm_xs) >= 2 and len(lm_ys) >= 2:
                    # ペア端 = LM_ids の先頭と末尾 (アンカー両端)
                    params['marker0_world_x'] = float(lm_xs[0])
                    params['marker0_world_y'] = float(lm_ys[0])
                    params['marker7_world_x'] = float(lm_xs[-1])
                    params['marker7_world_y'] = float(lm_ys[-1])
                    head = lm_ids[0] if lm_ids else '?'
                    tail = lm_ids[-1] if lm_ids else '?'
                    actions.append(LogInfo(msg=(
                        f'[bin_landmark_localization] region_filter LM injected: '
                        f'{head}=({lm_xs[0]:.2f},{lm_ys[0]:.2f}) '
                        f'{tail}=({lm_xs[-1]:.2f},{lm_ys[-1]:.2f})'
                    )))
                else:
                    actions.append(LogInfo(msg=(
                        '[bin_landmark_localization] use_dynamic_marker:=true だが '
                        'landmark_map_file から LM_xs/LM_ys を取得できなかった。 '
                        'fixed (-1.4, 1.4) / (-3.4, 1.4) にフォールバックします。'
                    )))
            except Exception as e:
                actions.append(LogInfo(msg=(
                    f'[bin_landmark_localization] failed to read landmark_map_file '
                    f'for region_filter DI: {e}. fallback to defaults.'
                )))

        actions.append(Node(
            package='nedo_localization',
            executable='hesai_region_filter_node',
            name='hesai_region_filter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_scrap_detection')),
            parameters=[params],
        ))
        return actions

    region_filter = OpaqueFunction(function=_make_region_filter)

    # ---- ROI内最高点抽出 ----
    highest_point = Node(
        package='nedo_localization',
        executable='highest_point_node',
        name='highest_point_node',
        output='screen',
        condition=IfCondition(enable_scrap_detection),
        parameters=[{
            # スクラップROI後の点群を入力に、最高点を抽出
            'input_topic': '/scrap_filtered_points',
            'output_topic': '/filtered_points_highest_point',
        }],
    )

    # ---- 起動順 (依存的順序はROS2では明示不要だが見やすさのため) ----
    ld.add_action(static_lidar_tf_avia)
    ld.add_action(static_lidar_tf_qt128)
    ld.add_action(static_lidar_tf_ot128)
    ld.add_action(static_map_to_odom)
    ld.add_action(static_odom_to_base)
    ld.add_action(livox_driver)
    ld.add_action(qt128_driver)
    ld.add_action(ot128_driver)
    ld.add_action(bin_detector)
    ld.add_action(map_server)
    ld.add_action(localizer)
    #ld.add_action(nedo_controller)
    ld.add_action(landmark_map_viz)
    ld.add_action(region_filter)
    ld.add_action(highest_point)

    return ld
