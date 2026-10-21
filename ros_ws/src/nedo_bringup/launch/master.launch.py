"""
master.launch.py: マスターで最初から最後まで動かすための全部入り (八輪台車 + 林研の経路計画)

大本 = ekf_bin_localization.launch.py (2026-10-16 時点 / main ae66187)。八輪台車と林研の経路計画で実際に巡回走行した構成。
その中身をこのファイルに丸ごとコピーし、後ろにマスターまわりを足した (ekf_bin_localization.launch.py 自体は変えない)。
ekf 側から変えたのは次の2つだけ:
  - enable_controller の既定を false: nedo_controller は /goal_pose を受けて /cmd_vel を出すので、林研の経路計画と取り合う
  - landmark_map_file の既定を params_yard.yaml: 目標姿勢の計算に使う marker_2〜7 が params.yaml (marker_3/4 だけ) に無い
  ekf 側を直したら、こちらにも同じ変更を入れること

足したもの (このファイルの後ろ半分)
- 経路計画 (林研, nedo_test_ws) : manage_node_master / heading_test_node / drive_node / cmd_vel_mux
    manage_node_master が /goal_pose を受け、/navigation/result に planned / arrived / blocked を返す
    障害物の点群 /filtered_points_shuttle は Ouster → depth_clustering → points_projection (enable_ouster:=true)
- スクラップ山 (QT128) : QT128 ドライバ (2台同時用の設定) / scrap_pile_skirt_node / pickup_planner_node
    ランドマークが見えていて自己位置がある前提 (2026-10-16 本人指示)。scrap_pile_skirt_node はランドマーク地図で
    円柱を家電と間違えないよう除き、物体の点を map 系で出す。止めた状態で起動し、マスターが山の前で動かす
    pickup_planner_node の出力は /pickup_pose (マスターが受けて /goal_pose へ中継する)
- 機構部 : mechanism_node.py (use_mechanism:=true)。依頼に自動で答える簡易ノード
- マスター : master_node / master_ui.py (use_ui:=true)。別ターミナルで ros2 run nedo_master master_log.py

地図と目標は現地で作って渡す (既定の params_yard.yaml / master_goals.yaml はヤード計画値の見本):
  ros2 launch nedo_bringup master.launch.py landmark_map_file:=<現地で測った地図.yaml> goals_file:=<目標の選び方.yaml>
山の前と荷下ろし地点の目標姿勢は goals_file の選び方から、起動時にランドマーク地図で計算して master_node に渡す
(起動ログに [goals] で出る)

注意
- OT128 と QT128 を同時に使うので、QT128 実機を 192.168.1.202 / UDP 2369 に変えておくこと
  (config_hesai_qt128_dual.yaml。同じポートのままだと両方の点群が壊れる)
- QT128 は下の「足したもの」で起動する。ekf 由来の sensor_type:=qt128 / also_start_qt128:=true は使わない
  (ドライバと取付 TF が二重になる)
- QT128 の取付位置は未定。取り付けたら測って qt128_x / qt128_y / qt128_z / qt128_roll_deg / qt128_pitch_deg / qt128_yaw_deg で渡す
  (base_link からの値。base_link = ekf の OT128 取付: OT128 が原点・yaw −90°)。既定値は仮 (Ranger の 2026-09-24 実測)
- rover_odom_to_pose2d は起動しない (EKF と odom→base_link の TF を取り合うため)

----- ここから ekf_bin_localization.launch.py の説明 (そのまま) -----
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

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# install 後の launch ディレクトリから 4 つ上がリポジトリルート。
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))

# ===== ここから master.launch.py で足したもの (ekf にはない) =====
_test_ws_dir = os.path.join(_ws_dir, 'ros_ws', 'src', 'nedo_test_ws')

# 機体の面 → 機体の先頭 (base_link +x) から反時計回りの角度 [deg]
_FACE_DEG = {'front': 0.0, 'left': 90.0, 'back': 180.0, 'right': -90.0}
# 目標が A→B の線のどちら側か → A→B の向きから反時計回りの角度 [deg]
_SIDE_DEG = {'left': 90.0, 'right': -90.0}

# アーム寸法 (仮値)。scrap_pile_skirt_node と pickup_planner_node に同じ値を渡す
_ARM_ARGS = [
    ('arm_base_x', '0.0', 'アーム根元の x [m, base_footprint]。長辺上の位置は未定のため仮値=中央'),
    ('arm_base_y', '0.4', 'アーム根元の y [m, base_footprint] = 左の長辺'),
    ('arm_yaw_deg', '90.0', 'アームの向き [deg, 機体+xから反時計回り]。アームは旋回しない'),
    ('arm_length', '0.7', 'r: アーム根元からグラップル中心までの最大水平距離 [m]'),
    ('approach_limit', '0.3', 'D: 機体と家電の最小許容距離 [m]'),
]


# QT128 の取付 (base_link → hesai_qt128)。★取付位置は未定なので仮値。取り付けたら測って launch 引数で渡す
#   仮値 = Ranger 台車での 2026-09-24 実測 (lidar_tf_publisher_node の既定値と同じ)
_QT128_TF_ARGS = [
    ('qt128_x', '0.36', 'QT128 の x [m, base_link]。★未定・仮値'),
    ('qt128_y', '0.25', 'QT128 の y [m, base_link]。★未定・仮値'),
    ('qt128_z', '1.639', 'QT128 の z [m, base_link]。★未定・仮値'),
    ('qt128_roll_deg', '50.37', 'QT128 の roll [deg]。★未定・仮値'),
    ('qt128_pitch_deg', '5.06', 'QT128 の pitch [deg]。★未定・仮値'),
    ('qt128_yaw_deg', '172.95', 'QT128 の yaw [deg]。★未定・仮値'),
]


def _goal_pose(goal, landmarks, name):
    """master_goals.yaml の1項目 → [x, y, yaw_deg] (map 系)"""
    a, b = goal['landmarks']
    for lm in (a, b):
        if lm not in landmarks:
            raise RuntimeError(f'master_goals.yaml {name}: {lm} がランドマーク地図にない ({sorted(landmarks)})')
    (xa, ya), (xb, yb) = landmarks[a], landmarks[b]
    if goal['target_side'] not in _SIDE_DEG or goal['facing'] not in _FACE_DEG:
        raise RuntimeError(f'master_goals.yaml {name}: target_side は left|right、facing は front|left|back|right')
    theta = math.degrees(math.atan2(yb - ya, xb - xa))      # A→B の向き
    toward = theta + _SIDE_DEG[goal['target_side']]         # 目標 (山・荷下ろし先) へ向かう向き
    yaw = toward - _FACE_DEG[goal['facing']]                # その面を目標へ向けたときの機体の向き
    yaw = (yaw + 180.0) % 360.0 - 180.0
    # standoff: 中点から目標と反対側へ下がる距離 [m] (省略時 0)
    back = math.radians(toward)
    standoff = float(goal.get('standoff', 0.0))
    return [0.5 * (xa + xb) - standoff * math.cos(back), 0.5 * (ya + yb) - standoff * math.sin(back), yaw]


def _master(context):
    with open(LaunchConfiguration('landmark_map_file').perform(context)) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    landmarks = {i: (x, y) for i, x, y in zip(p['LM_ids'], p['LM_xs'], p['LM_ys'])}
    with open(LaunchConfiguration('goals_file').perform(context)) as f:
        goals = yaml.safe_load(f)

    pile = _goal_pose(goals['pile_front'], landmarks, 'pile_front')
    drops = [_goal_pose(g, landmarks, f'drops[{i}]') for i, g in enumerate(goals['drops'])]

    actions = []
    for name, g, pose in [('山の前', goals['pile_front'], pile)] + \
            [(f'荷下ろし{i + 1}', d, pose) for i, (d, pose) in enumerate(zip(goals['drops'], drops))]:
        a, b = g['landmarks']
        back = f' から {g["standoff"]} m 下がった点' if g.get('standoff') else ''
        actions.append(LogInfo(msg=f'[goals] {name}: {a}-{b} の中点{back} ({pose[0]:.3f}, {pose[1]:.3f}) '
                                   f'yaw {pose[2]:.2f} deg ({g["facing"]} を {g["target_side"]} 側へ)'))
    actions.append(Node(
        package='nedo_master', executable='master_node',
        name='master_node', output='screen',
        parameters=[{
            'pile_front': pile,
            'drop_xs': [d[0] for d in drops],
            'drop_ys': [d[1] for d in drops],
            'drop_yaws_deg': [d[2] for d in drops],
        }],
    ))
    return actions


def _ouster(context):
    """障害物用 LiDAR (Ouster) → depth_clustering → points_projection → /filtered_points_shuttle
    shuttle_system.launch.py と同じ構成。ouster_ros が無い環境でも launch 自体は起動できるよう、使うときだけ探す"""
    if LaunchConfiguration('enable_ouster').perform(context).lower() != 'true':
        return []
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ouster_ros'), 'launch', 'driver.launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('ouster_params'), 'viz': 'false'}.items(),
        ),
        Node(
            package='depth_clustering_cpp', executable='depth_clustering_node', output='screen',
            parameters=[{'use_metadata_file': True, 'metadata_path': LaunchConfiguration('ouster_metadata_path')}],
            sigterm_timeout='2', sigkill_timeout='5',
        ),
        Node(
            package='points_projection', executable='points_projection_node', output='screen',
            sigterm_timeout='2', sigkill_timeout='5',
        ),
        # base_link → os_sensor (shuttle_system.launch.py の既定値)
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='ouster_to_base_link_tf',
            arguments=['--x', '-1.375', '--y', '0.0', '--z', '1.68',
                       '--roll', '0.0', '--pitch', '-0.41', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'os_sensor'],
            output='screen',
        ),
    ]


# ===== ここまで =====


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('nedo_bringup')

    third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')
    livox_share_dir = os.path.join(
        third_party_install_prefix, 'livox_ros2_avia', 'share', 'livox_ros2_avia')

    # master: 目標姿勢の計算に marker_2〜7 が要るので params_yard.yaml (ekf は params.yaml)
    default_landmark_yaml = os.path.join(pkg_share, 'config', 'params_yard.yaml')
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
        'enable_controller', default_value='false',  # master: 林研の経路計画と /cmd_vel を取り合うので既定 false (ekf は true)
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

    # =========================================================
    # ここから master.launch.py で足したもの (ekf にはない)
    # =========================================================
    arm = {name: LaunchConfiguration(name) for name, _, _ in _ARM_ARGS}

    # ---- マスター ----
    ld.add_action(DeclareLaunchArgument(
        'goals_file', default_value=os.path.join(pkg_share, 'config', 'master_goals.yaml'),
        description='目標姿勢の選び方 (ランドマークの組・どちら側か・向ける面)'))
    ld.add_action(DeclareLaunchArgument('use_ui', default_value='true', description='マスターの UI を出すか'))
    ld.add_action(DeclareLaunchArgument(
        'use_mechanism', default_value='true',
        description='機構部の簡易ノードを起動するか (本物の機構部をつなぐときは false)'))
    # ---- 経路計画 (林研) ----
    ld.add_action(DeclareLaunchArgument('robot_radius', default_value='1.0', description='manage_node 最近点除外閾値 [m]'))
    ld.add_action(DeclareLaunchArgument('obstacle_avoid_radius', default_value='1.5', description='manage_node 回避円 r2 [m]'))
    ld.add_action(DeclareLaunchArgument('path_band_half_width', default_value='1.0', description='manage_node 経路バンド片側幅 [m]'))
    ld.add_action(DeclareLaunchArgument('lookahead_distance', default_value='1.5', description='manage_node Pure Pursuit 注視距離 [m]'))
    ld.add_action(DeclareLaunchArgument(
        'voicevox_url', default_value='http://localhost:50021', description='Voicevox サーバ URL (空文字で音声なし)'))
    ld.add_action(DeclareLaunchArgument(
        'enable_ouster', default_value='true',
        description='障害物用 LiDAR (Ouster → depth_clustering → points_projection) を起動するか'))
    ld.add_action(DeclareLaunchArgument(
        'ouster_params', default_value=os.path.join(_test_ws_dir, 'ouster_params.yaml'),
        description='Ouster ドライバ params ファイル'))
    ld.add_action(DeclareLaunchArgument(
        'ouster_metadata_path', default_value=os.path.join(_test_ws_dir, '192.168.1-metadata.json'),
        description='depth_clustering 用 Ouster メタデータ JSON'))
    # ---- スクラップ山 (QT128) ----
    ld.add_action(DeclareLaunchArgument(
        'enable_qt128', default_value='true', description='QT128 ドライバを起動するか (bag 再生時は false)'))
    ld.add_action(DeclareLaunchArgument(
        'qt128_dual_config_file', default_value=os.path.join(pkg_share, 'config', 'config_hesai_qt128_dual.yaml'),
        description='QT128 ドライバ設定 (OT128 と同時に使うので .202 / 2369 の設定)'))
    for n, v, d in _ARM_ARGS:
        ld.add_action(DeclareLaunchArgument(n, default_value=v, description=d))
    # QT128 の取付 (base_link → hesai_qt128)。★取付位置は未定。既定値は仮 (Ranger の 2026-09-24 実測)
    for n, v, d in _QT128_TF_ARGS:
        ld.add_action(DeclareLaunchArgument(n, default_value=v, description=d))

    # ===== 経路計画 (林研): /goal_pose → /cmd_vel, 結果は /navigation/result =====
    ld.add_action(Node(
        package='manage_node', executable='manage_node_master', output='screen',
        parameters=[{
            'robot_radius': LaunchConfiguration('robot_radius'),
            'obstacle_avoid_radius': LaunchConfiguration('obstacle_avoid_radius'),
            'path_band_half_width': LaunchConfiguration('path_band_half_width'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'voicevox_url': LaunchConfiguration('voicevox_url'),
        }],
        sigterm_timeout='2', sigkill_timeout='5',
    ))
    ld.add_action(Node(package='heading_test_node', executable='heading_test_node', output='screen',
                       sigterm_timeout='2', sigkill_timeout='5'))
    ld.add_action(Node(package='drive_node', executable='drive_node', output='screen',
                       sigterm_timeout='2', sigkill_timeout='5'))
    ld.add_action(Node(package='cmd_vel_mux', executable='cmd_vel_mux_node', output='screen',
                       sigterm_timeout='2', sigkill_timeout='5'))
    ld.add_action(OpaqueFunction(function=_ouster))

    # ===== スクラップ山 (QT128): 裾検出 → 把持計画 =====
    ld.add_action(Node(
        package='hesai_ros_driver', executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node', namespace='hesai_qt128', output='screen',
        parameters=[{'config_path': LaunchConfiguration('qt128_dual_config_file')}],
        condition=IfCondition(LaunchConfiguration('enable_qt128')),
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_base_footprint_tf', output='screen',
        arguments=['0', '0', '-0.3', '0', '0', '0', '1', 'base_link', 'base_footprint'],
    ))
    # base_link → hesai_qt128 (取付位置は launch 引数 qt128_*。角度は deg で受けて rad にして渡す)
    ld.add_action(Node(
        package='nedo_localization', executable='lidar_tf_publisher_node',
        name='lidar_tf_publisher_node', output='screen',
        parameters=[{
            'x': LaunchConfiguration('qt128_x'),
            'y': LaunchConfiguration('qt128_y'),
            'z': LaunchConfiguration('qt128_z'),
            **{k: PythonExpression(['radians(', LaunchConfiguration(f'qt128_{k}_deg'), ')'])
               for k in ('roll', 'pitch', 'yaw')},
        }],
    ))
    # ランドマーク地図 (localizer と同じファイル) を渡す: 円柱を家電と間違えないように除き、点を map 系で出す
    ld.add_action(Node(
        package='nedo_localization', executable='scrap_pile_skirt_node',
        name='scrap_pile_skirt_node', output='screen',
        parameters=[landmark_map_file, {
            'input_topic': '/hesai_qt128/lidar_points',
            'ground_frame': 'base_footprint',
            'map_frame': 'map',
            'start_enabled': False,  # マスターが山の前に着いたら動かす
            **arm,
        }],
    ))
    ld.add_action(Node(
        package='nedo_navigation', executable='pickup_planner_node',
        name='pickup_planner_node', output='screen',
        parameters=[{
            'goal_topic': '/pickup_pose',  # /goal_pose だとマスターを通らずに台車が動くため
            'ground_frame': 'base_footprint',
            'map_frame': 'map',
            **arm,
        }],
    ))

    # ===== 機構部・マスター =====
    ld.add_action(Node(
        package='nedo_master', executable='mechanism_node.py', name='mechanism_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_mechanism')),
    ))
    ld.add_action(OpaqueFunction(function=_master))
    ld.add_action(Node(
        package='nedo_master', executable='master_ui.py', name='master_ui', output='screen',
        parameters=[{'nav_node': 'manage_node_master'}],  # 経路計画は林研版
        condition=IfCondition(LaunchConfiguration('use_ui')),
    ))

    return ld
