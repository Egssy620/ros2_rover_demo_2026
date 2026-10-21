"""
ekf_bin_localization.launch.py

目的
- bin_landmark_localization.launch.py の自己位置推定パイプラインに
  robot_localization (単一 EKF) を組み込み、LM が 2〜3 秒途切れても
  cmd_vel 疑似速度 + IMU ヨーレートのデッドレコニングで走行を継続させる構成。

データの流れ
  cylinder_landmark_detector_bin → /matched_marker
    → localizer_node → /raw_robot_pose (PoseStamped, map系, publish_tf:=false)
      → pose_converter → /robot_pose_cov (PoseWithCovarianceStamped, 共分散付与)
  /cmd_vel (Twist)
    → cmd_vel_to_twist_node → /cmd_vel_cov (TwistWithCovarianceStamped)
  /livox/imu (Imu, ヨーレートのみ使用)
  これら 3 入力を EKF が融合 → /odometry/filtered (Odometry) + TF odom→base_link
    → odom_to_pose   → /robot_pose   (PoseStamped, 下流の controller 用)
    → pose_bridge_ekf → /robot_pose2d (Pose2D)
  ※ EKF出力は /odometry/filtered を正とし、/robot_pose 等へは変換ノードで出す
    (remap で別型を同名に乗せない / フィードバックループを作らない)。

TF トポロジ (単一 EKF, world_frame=odom)
- ekf_filter_node : odom→base_link を配信 (cmd_vel+IMU+LM の連続推定)
- static          : map→odom = identity (map と odom を一致させる)
- static          : <lidar_frame>→base_link (取付位置)
- localizer       : publish_tf:=false にして map→base_link 直接配信を停止
- 従来の static odom→base_link は配信しない (EKF が所有)

注意
- robot_localization (ros-humble-robot-localization) が必要。
- nedo_controller は enable_controller:=true (デフォルト) で起動する。入力 /robot_pose
  は odom_to_pose 経由の EKF 連続出力 (生 localizer ではない) なので、LM が途切れて
  EKF が coast する間も /robot_pose が出続け、3秒ウォッチドッグで停止しない。
  走行には別途 /goal_pose publisher が必要。

速度源の切替 (use_wheel_odom)
- false            : cmd_vel 疑似速度の「単一EKF」。cmd_vel_to_twist + ekf_node、
                      static map→odom identity。車輪オドメ不要。
- true (デフォルト): 車輪オドメトリの「デュアルEKF」。ekf_local(odom→base_link)
                      + ekf_global(map→odom)、ekf_dual_localization.yaml を使用。
                      rover_odo_to_twist が /rover_odo (TwistStamped) を
                      /rover_odo_cov (TwistWithCovarianceStamped) に変換して twist0 へ。
                      cmd_vel_to_twist と static map→odom は起動しない。
                      別途 /rover_odo publisher (車輪ドライバ) が必要。

  ※ robot_localization が購読できるのは odomN=nav_msgs/Odometry,
     poseN=PoseWithCovarianceStamped, twistN=TwistWithCovarianceStamped,
     imuN=sensor_msgs/Imu の 4 種のみ。TwistStamped は remap しても型が違うため
     接続が成立せず、エラーも出ずに無入力になる (=変換ノードが必須)。
  ※ rover_odom_to_pose2d_node も odom→base_link TF を出すので、EKF 構成では
     同時起動しないこと (ekf_local と TF が競合する)。

主要 LaunchArgument
- sensor_type (str)            : 'avia' | 'qt128' | 'ot128' (デフォルト 'ot128')
                                 OT128=/hesai_ot128/lidar_points (frame hesai_ot128)
                                 QT128=/hesai_qt128/lidar_points (frame hesai_qt128)
                                 両 Hesai とも IP .201 / UDP 2368 (工場出荷時) のため
                                 同時起動は不可。差し替え運用では sensor_type で切替。
                                 同時運用の手順は config_hesai_qt128.yaml 冒頭を参照。
- lidar_input_topic/frame (str): 未指定なら sensor_type から自動決定
- landmark_map_file (str)      : ランドマーク座標YAML (LM_ids / LM_xs / LM_ys 形式)
- detector_params_file (str)   : 検出器パラメータYAML
- ekf_params_file (str)        : 単一EKF設定 YAML (use_wheel_odom:=false 時)
- ekf_dual_params_file (str)   : デュアルEKF設定 YAML (use_wheel_odom:=true 時)
- use_wheel_odom (bool)        : 車輪オドメ/odom を使うか (デフォルト false)
- enable_landmark_viz (bool)   : landmark_map_visualizer_node を起動するか
- enable_controller (bool)     : nedo_controller を起動するか (デフォルト true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# install 後の launch ディレクトリから 4 つ上がリポジトリルート。
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('nedo_bringup')

    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    livox_share_dir = os.path.join(
        third_party_install_prefix, 'livox_ros2_avia', 'share', 'livox_ros2_avia')

    default_landmark_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    default_detector_yaml = os.path.join(pkg_share, 'config', 'detector_params.yaml')
    default_avia_json     = os.path.join(livox_share_dir, 'config', 'livox_lidar_config.json')
    default_qt128_yaml    = os.path.join(pkg_share, 'config', 'config_hesai_qt128.yaml')
    default_ot128_yaml    = os.path.join(pkg_share, 'config', 'config_hesai_ot128.yaml')
    default_ekf_yaml      = os.path.join(pkg_share, 'config', 'ekf_bin_localization.yaml')
    default_ekf_dual_yaml = os.path.join(pkg_share, 'config', 'ekf_dual_localization.yaml')

    # ---- LaunchArgument ----
    sensor_type = LaunchConfiguration('sensor_type')
    lidar_input_topic = LaunchConfiguration('lidar_input_topic')
    lidar_input_frame = LaunchConfiguration('lidar_input_frame')
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    detector_params_file = LaunchConfiguration('detector_params_file')
    avia_config_file = LaunchConfiguration('avia_config_file')
    qt128_config_file = LaunchConfiguration('qt128_config_file')
    ot128_config_file = LaunchConfiguration('ot128_config_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    ekf_dual_params_file = LaunchConfiguration('ekf_dual_params_file')
    enable_landmark_viz = LaunchConfiguration('enable_landmark_viz')
    enable_controller = LaunchConfiguration('enable_controller')
    detector_output_topic = LaunchConfiguration('detector_output_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_wheel_odom = LaunchConfiguration('use_wheel_odom')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')

    ld.add_action(DeclareLaunchArgument(
        'sensor_type', default_value='ot128',
        description=("検出器の入力に使う LiDAR ('avia' | 'qt128' | 'ot128')。"
                     "デフォルトは ot128 (Hesai OT128 @192.168.1.201, 工場出荷時のまま)。")))
    # sensor_type 以外のドライバも併走させたい場合のフラグ。
    # 注意: Hesai 2台 (qt128/ot128) は現状どちらも .201/2368 を指しているため
    # 同時起動できない。同時運用の手順は config_hesai_qt128.yaml 冒頭を参照。
    # Avia との併走は制約なし。
    ld.add_action(DeclareLaunchArgument(
        'also_start_avia', default_value='false',
        description='sensor_type に関わらず Livox Avia ドライバも同時起動するか'))
    ld.add_action(DeclareLaunchArgument(
        'also_start_qt128', default_value='false',
        description='sensor_type に関わらず Hesai QT128 ドライバも同時起動するか'))
    ld.add_action(DeclareLaunchArgument(
        'also_start_ot128', default_value='false',
        description='sensor_type に関わらず Hesai OT128 ドライバも同時起動するか'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_input_topic', default_value='',
        description='LiDAR PointCloud2 topic. 空なら sensor_type から自動。'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_input_frame', default_value='',
        description='LiDAR frame id. 空なら sensor_type から自動。'))
    ld.add_action(DeclareLaunchArgument(
        'detector_output_topic', default_value='/matched_marker',
        description='検出器が PoseStampedArray を publish するトピック (localizer 入力)'))
    ld.add_action(DeclareLaunchArgument(
        'avia_config_file', default_value=default_avia_json,
        description='Livox Avia (livox_ros2_avia) 設定 JSON'))
    ld.add_action(DeclareLaunchArgument(
        'qt128_config_file', default_value=default_qt128_yaml,
        description='Hesai QT128 (hesai_ros_driver) 設定 YAML'))
    ld.add_action(DeclareLaunchArgument(
        'ot128_config_file', default_value=default_ot128_yaml,
        description='Hesai OT128 (hesai_ros_driver) 設定 YAML'))
    ld.add_action(DeclareLaunchArgument(
        'landmark_map_file', default_value=default_landmark_yaml,
        description='YAML containing LM_ids / LM_xs / LM_ys'))
    ld.add_action(DeclareLaunchArgument(
        'detector_params_file', default_value=default_detector_yaml,
        description='検出器パラメータYAML'))
    ld.add_action(DeclareLaunchArgument(
        'ekf_params_file', default_value=default_ekf_yaml,
        description='単一EKF (cmd_vel疑似速度) 設定 YAML。use_wheel_odom:=false 時に使用'))
    ld.add_action(DeclareLaunchArgument(
        'ekf_dual_params_file', default_value=default_ekf_dual_yaml,
        description='デュアルEKF (wheel odom) 設定 YAML。use_wheel_odom:=true 時に使用'))
    # 速度源の切替: false=cmd_vel疑似速度(単一EKF), true=wheel odom(/odom, デュアルEKF)
    ld.add_action(DeclareLaunchArgument(
        'use_wheel_odom', default_value='true',
        description=('車輪オドメトリ(/rover_odo)を使うか。'
                     'false=cmd_vel疑似速度の単一EKF / '
                     'true=車輪オドメのデュアルEKF(ekf_local+ekf_global)。'
                     'true時は別途 車輪ドライバ(/rover_odo publisher) が必要。')))
    ld.add_action(DeclareLaunchArgument(
        'wheel_odom_topic', default_value='/body_twist',
        description=('車輪オドメトリのトピック (geometry_msgs/TwistStamped)。'
                     'rover_odo_to_twist_node が /rover_odo_cov '
                     '(TwistWithCovarianceStamped) に変換して EKF の twist0 に渡す。')))
    ld.add_action(DeclareLaunchArgument(
        'enable_landmark_viz', default_value='true',
        description='landmark_map_visualizer_node を起動するか'))
    ld.add_action(DeclareLaunchArgument(
        'enable_controller', default_value='true',
        description=('nedo_controller を起動するか。/robot_pose(EKF連続出力)と '
                     '/goal_pose を購読し /cmd_vel を出す。別途 /goal_pose publisher が必要。')))
    # IMU トピック。空なら sensor_type から自動 (avia は multi_topic=1 でシリアル付き)。
    ld.add_action(DeclareLaunchArgument(
        'imu_topic', default_value='',
        description='IMU topic. 空なら sensor_type から自動 (avia=/livox/imu_<serial>)。'))

    # sensor_type → topic/frame 自動決定 (bin_landmark_localization と同流儀)
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
    _IMU_BY_SENSOR = {
        'avia': '/livox/imu_3JEDM4V001DY881',
        'qt128': '/hesai_qt128/lidar_imu',
        'ot128': '/hesai_ot128/lidar_imu',
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
    is_avia = PythonExpression([
        "'", sensor_type, "' == 'avia' or '", also_start_avia, "'.lower() == 'true'"])
    is_qt128 = PythonExpression([
        "'", sensor_type, "' == 'qt128' or '", also_start_qt128, "'.lower() == 'true'"])
    is_ot128 = PythonExpression([
        "'", sensor_type, "' == 'ot128' or '", also_start_ot128, "'.lower() == 'true'"])

    # IMU トピック自動決定。avia は multi_topic=1 のため livox/imu_<serial>。
    # lddc.cpp が livox/imu_<broadcast_code> に publish する (lidar と同流儀)。
    auto_imu = PythonExpression([
        "'", imu_topic, "' if '", imu_topic, "' else ",
        repr(_IMU_BY_SENSOR), ".get('", sensor_type, "', ",
        repr(_IMU_BY_SENSOR['ot128']), ")"
    ])

    ld.add_action(LogInfo(msg=['[ekf_bin_localization] sensor_type=', sensor_type,
                               ' topic=', auto_topic, ' frame=', auto_frame]))

    # ---- Livox Avia ドライバ ----
    livox_driver = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(is_avia),
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 1},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': avia_config_file},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # ---- Hesai QT128 ドライバ ----
    # ns 'hesai_qt128' / IP 192.168.1.201 / UDP 2368 (config_hesai_qt128.yaml)
    qt128_driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        namespace='hesai_qt128',
        output='screen',
        condition=IfCondition(is_qt128),
        parameters=[{'config_path': qt128_config_file}],
    )

    # ---- Hesai OT128 ドライバ ----
    # ns 'hesai_ot128' / IP 192.168.1.202 / UDP 2369 (config_hesai_ot128.yaml)
    # ノード名は QT128 と同名だが namespace が違うため完全修飾名は衝突しない。
    ot128_driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        namespace='hesai_ot128',
        output='screen',
        condition=IfCondition(is_ot128),
        parameters=[{'config_path': ot128_config_file}],
    )

    # ---- BIN ランドマーク検出器 ----
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
                # 検出器のIMUヨー補償も同じ実トピックを使う (multi_topic=1のシリアル付き)
                'imu_topic': auto_imu,
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

    # ---- 自己位置推定 (SVD) : EKF併用のため TF は配信しない ----
    localizer = Node(
        package='nedo_localization',
        executable='localizer_node',
        output='screen',
        parameters=[landmark_map_file, {'lpf_rate': 0.7, 'publish_tf': False}],
        remappings=[('/robot_pose', '/raw_robot_pose')],
    )

    # ---- /robot_pose → /robot_pose_cov (共分散付与) ----
    pose_converter = Node(
        package='nedo_localization',
        executable='pose_converter',
        name='pose_to_covariance_node',
        output='screen',
        parameters=[{
            'input_topic': '/raw_robot_pose',
            'output_topic': '/robot_pose_cov',
            'position_variance': 0.0004,
            'orientation_variance': 0.000005,
        }],
    )

    # ---- /cmd_vel → /cmd_vel_cov (疑似速度 + 共分散) ----
    # use_wheel_odom:=false のときだけ起動 (wheel odom 使用時は不要)。
    cmd_vel_to_twist = Node(
        package='nedo_navigation',
        executable='cmd_vel_to_twist_node',
        name='cmd_vel_to_twist_node',
        output='screen',
        condition=UnlessCondition(use_wheel_odom),
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_cov',
            'frame_id': 'base_link',
            'linear_x_variance': 0.02,
            'linear_y_variance': 0.02,
            'angular_z_variance': 0.02,
        }],
    )

    # ---- /rover_odo → /rover_odo_cov (車輪オドメ速度 + 共分散) ----
    # 実機の車輪オドメは geometry_msgs/TwistStamped で、robot_localization が
    # odomN で受け付ける nav_msgs/Odometry ではない。型が違うと remap しても接続
    # されず無入力になるため、TwistWithCovarianceStamped へ変換して twist0 に渡す。
    # use_wheel_odom:=true のときだけ起動。
    # kvx/kvy/kth は rover_odom_to_pose2d_node と同じ較正値を使うこと。
    rover_odo_to_twist = Node(
        package='nedo_navigation',
        executable='rover_odo_to_twist_node',
        name='rover_odo_to_twist_node',
        output='screen',
        condition=IfCondition(use_wheel_odom),
        parameters=[{
            'input_topic': wheel_odom_topic,
            'output_topic': '/rover_odo_cov',
            'frame_id': 'base_link',
            'kvx': 1.0,
            'kvy': 1.0,
            'kth': 1.0,
            'linear_x_variance': 0.01,
            'linear_y_variance': 0.02,
            'angular_z_variance': 0.01,
        }],
    )

    # ---- IMU 再スタンプ (デバイス時計 → ホストROS時刻) ----
    # Livox IMU は enable_timesync:=false でデバイス時計のまま。EKF の他入力
    # (odom/pose_cov/cmd_vel_cov) はホスト時刻なので、再スタンプして領域を揃える。
    # 入力は実トピック (auto_imu)、出力を EKF が購読する。
    # ※ 検出器のヨー補償は「生IMU (auto_imu)」を使う (点群と同じデバイス時計で
    #    整合しているため)。再スタンプ後を渡してはいけない。
    imu_restamp = Node(
        package='nedo_localization',
        executable='imu_restamp_node',
        name='imu_restamp_node',
        output='screen',
        parameters=[{
            'input_topic': auto_imu,
            'output_topic': '/livox/imu_restamped',
        }],
    )

    # =========================================================
    # 速度源切替: use_wheel_odom で単一EKF or デュアルEKF を選択
    # =========================================================

    # ---- [単一EKF] cmd_vel疑似速度 (use_wheel_odom:=false) ----
    # world_frame=odom。odom→base_link を配信。map→odom は static identity を別途配信。
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=UnlessCondition(use_wheel_odom),
        parameters=[ekf_params_file],
        remappings=[
            # yaml の imu0:/livox/imu を再スタンプ済みトピックへ (ホスト時刻に統一)
            ('/livox/imu', '/livox/imu_restamped'),
        ],
    )

    # ---- [デュアルEKF] wheel odom /odom (use_wheel_odom:=true) ----
    # ローカルEKF: /odom + IMU → odom→base_link (連続)。出力は /odometry/local。
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        condition=IfCondition(use_wheel_odom),
        parameters=[ekf_dual_params_file],
        remappings=[
            ('/livox/imu', '/livox/imu_restamped'),
            ('/odometry/filtered', '/odometry/local'),
        ],
    )
    # グローバルEKF: /odom + IMU + LM → map→odom。絶対pose出力は /odometry/filtered
    # (デフォルトのまま)。odom_to_pose が PoseStamped /robot_pose に変換する。
    # ローカル側は /odometry/local に分離済みなので衝突しない。
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        condition=IfCondition(use_wheel_odom),
        parameters=[ekf_dual_params_file],
        remappings=[
            ('/livox/imu', '/livox/imu_restamped'),
        ],
    )

    # ---- 静的TF ----
    # NOTE: ノード名は LiDAR ごとに別名にすること。以前は avia/ot128 とも
    # 'lidar_to_base_link_tf' だったため、2台を併走させるとノード名が重複した。
    # 取付値 (並進/回転) は本launchの既存値をそのまま踏襲している。
    static_lidar_tf_avia = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='avia_to_base_link_tf',
        arguments=['0.0', '0.0', '1.0', '0', '0', '0', '1',
                   'base_link', 'livox_frame'],
        output='screen',
        condition=IfCondition(is_avia),
    )
    static_lidar_tf_qt128 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='qt128_to_base_link_tf',
        arguments=['0.0', '0.0', '0.0', '0', '0', '-0.7071', '0.7071',
                   'base_link', 'hesai_qt128'],
        output='screen',
        condition=IfCondition(is_qt128),
    )
    static_lidar_tf_ot128 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ot128_to_base_link_tf',
        arguments=['0.0', '0.0', '0.0', '0', '0', '-0.7071', '0.7071',
                   'base_link', 'hesai_ot128'],
        output='screen',
        condition=IfCondition(is_ot128),
    )
    # map→odom = identity。単一EKF時のみ static で配信 (map と odom を一致)。
    # デュアルEKF時は ekf_global が map→odom を動的配信するので static は出さない。
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen',
        condition=UnlessCondition(use_wheel_odom),
    )

    # ---- ランドマーク地図可視化 (RViz用) ----
    landmark_map_viz = Node(
        package='nedo_localization',
        executable='landmark_map_visualizer_node',
        name='landmark_map_visualizer_node',
        output='screen',
        condition=IfCondition(enable_landmark_viz),
    )

    # ---- odom_to_pose : EKF出力(Odometry) → /robot_pose (PoseStamped) ----
    # EKF 絶対pose は /odometry/filtered (単一=ekf_node, デュアル=ekf_global)。
    # nedo_controller など PoseStamped /robot_pose を購読する下流向けに変換。
    # 両モードとも /odometry/filtered が絶対pose なので無条件で起動。
    odom_to_pose = Node(
        package='nedo_bringup',
        executable='odom_to_pose.py',
        name='odom_to_pose_node',
        output='screen',
        parameters=[{
            'input_topic': '/odometry/filtered',
            'output_topic': '/robot_pose',
        }],
    )

    # ---- pose_bridge_ekf : EKF出力(Odometry) → /robot_pose2d (Pose2D) ----
    # /odometry/filtered を直接購読 (remap 不要)。Pose2D が必要な下流向け。
    pose_bridge = Node(
        package='nedo_bringup',
        executable='pose_bridge_ekf.py',
        name='pose_bridge_node',
        output='screen',
    )
    # ---- nedo_controller : /robot_pose + /goal_pose → /cmd_vel ----
    # /robot_pose は odom_to_pose 経由の EKF 連続出力(PoseStamped)。LM が途切れても
    # EKF が coast して /robot_pose は出続けるため、3秒ウォッチドッグで停止しない。
    # 別途 /goal_pose publisher が必要 (本launchには含まない)。
    nedo_controller = Node(
        package='nedo_navigation',
        executable='nedo_controller',
        name='nedo_controller',
        output='screen',
        condition=IfCondition(enable_controller),
        parameters=[{
            'distance_thresh': 0.5,
            'angle_thresh': 1.0 * 3.14159265 / 180.0,
            'scale_factor': 0.05,
            'max_vel': 0.5,
            'max_angle': 0.5,
        }],
    )

    ld.add_action(static_lidar_tf_avia)
    ld.add_action(static_lidar_tf_qt128)
    ld.add_action(static_lidar_tf_ot128)
    ld.add_action(static_map_to_odom)
    ld.add_action(livox_driver)
    ld.add_action(qt128_driver)
    ld.add_action(ot128_driver)
    ld.add_action(bin_detector)
    ld.add_action(map_server)
    ld.add_action(localizer)
    ld.add_action(pose_converter)
    ld.add_action(cmd_vel_to_twist)
    ld.add_action(rover_odo_to_twist)
    ld.add_action(imu_restamp)
    ld.add_action(ekf_node)
    ld.add_action(ekf_local)
    ld.add_action(ekf_global)
    ld.add_action(landmark_map_viz)
    ld.add_action(odom_to_pose)
    ld.add_action(pose_bridge)
    ld.add_action(nedo_controller)

    return ld
