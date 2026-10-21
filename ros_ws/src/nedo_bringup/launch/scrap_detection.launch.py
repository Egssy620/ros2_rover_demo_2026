import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#このlaunchファイルの位置: ros_ws/src/nedo_bringup/launch/
_this_dir = os.path.dirname(os.path.realpath(__file__))
_ws_dir = os.path.normpath(os.path.join(_this_dir, '..', '..', '..', '..'))
# _ws_dir → リポジトリルート
third_party_install_prefix = os.path.join(_ws_dir, 'third_party', 'install')

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

    # ソースディレクトリ相対で設定ファイルを参照 (再ビルド不要で反映)
    this_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.normpath(os.path.join(this_dir, '..', 'config'))
    params_yaml_path = os.path.join(config_dir, 'params.yaml')
    
    # Hesai ドライバは外部ターミナルで手動起動する方針に変更

    # 引数（円筒領域を定義するマーカーIDを指定可能）
    # 既定は QT128 (bin/ekf launch が ns='hesai_qt128' で起動する場合のトピック名)。
    # 別ターミナルで ros2 run hesai_ros_driver (namespaceなし) する運用なら
    #   hesai_input_topic:=/lidar_points
    # OT128 側を見るなら /hesai_ot128/lidar_points + input_frame を hesai_ot128 に。
    ld.add_action(DeclareLaunchArgument('hesai_input_topic', default_value='/hesai_qt128/lidar_points'))
    ld.add_action(DeclareLaunchArgument('marker_id_1', default_value='marker0', 
                                        description='First marker ID for cylinder region'))
    ld.add_action(DeclareLaunchArgument('marker_id_2', default_value='marker8',
                                        description='Second marker ID for cylinder region'))
    
    # マーカー座標を読み込み (指定されたマーカーIDを直径とする円筒領域)
    # 環境変数からマーカーIDを取得（起動時に上書き可能）
    marker_id_1_str = os.environ.get('SCRAP_MARKER_ID_1', 'marker0')
    marker_id_2_str = os.environ.get('SCRAP_MARKER_ID_2', 'marker8')
    
    marker_coords = load_marker_coordinates(params_yaml_path, marker_id_1_str, marker_id_2_str)

    # third_party のパス追加
    # （必要なら）third_party を AMENT_PREFIX_PATH に追加して依存検出を補助
    ld.add_action(SetEnvironmentVariable(name='AMENT_PREFIX_PATH', value=f"{third_party_install_prefix}:" + os.environ.get('AMENT_PREFIX_PATH', '')))
    ld.add_action(SetEnvironmentVariable(name='CMAKE_PREFIX_PATH', value=f"{third_party_install_prefix}:" + os.environ.get('CMAKE_PREFIX_PATH', '')))

    # ランドマップ配信ノードは削除（固定円筒のため不要）

    static_map_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_tf', output='screen',
        arguments=['0','0','0','0','0','0','1','map','odom']
    )
    
    # TF構造: map → odom → base_footprint → base_link → hesai_qt128
    static_odom_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='odom_to_base_footprint_tf', output='screen',
        arguments=['0','0','0','0','0','0','1','odom','base_footprint']
    )
    
    # base_footprint: ロボットの接地面（地面基準フレーム、Z=0が地面）
    # base_link は base_footprint から上方向にオフセット（例: 0.3m上）
    # TF: base_footprint → base_link (x, y, z, qx, qy, qz, qw, parent, child)
    # ロボットの高さに応じて調整してください
    static_base_footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_footprint_to_base_link_tf', output='screen',
        arguments=['0','0','0.3','0','0','0','1','base_footprint','base_link']
    )

    lidar_tf = Node(
        package='nedo_localization', executable='lidar_tf_publisher_node', name='lidar_tf_publisher_node', output='screen'
    )

    # 円筒領域フィルタのパラメータを設定
    # marker0 と marker8 の座標を直径とする円筒領域を抽出（デフォルト）
    # /robot_pose トピックを使用して動的に円筒領域を計算
    region_filter_params = {
        'input_topic': LaunchConfiguration('hesai_input_topic'),
        # Hesaiドライバの ros_frame_id (config_hesai_qt128.yaml) と一致させること。
        # 以前は 'lidar_link' だったが、ドライバもlidar_tf_publisher_nodeも
        # その名前のフレームを出しておらず、TF lookup が成立していなかった。
        'input_frame': 'hesai_qt128',
        'output_frame': 'base_link',
        'ground_frame': 'base_footprint',  # 地面基準フレーム (Z=0が地面)
        'cylinder_z_min': 0.0,
        'cylinder_z_max': 1.5,
        'debug_output': True,
        'publish_transform_info': True,
        'enable_max_z_output': True,
        'max_z_topic': '/scrap_pile_max_height',
        # Ground-relative height filtering (地面からの高さでフィルタリング)
        # base_footprintフレームでZ=0を地面として使用
        'enable_ground_relative': False,  # 一時的に無効化（処理高速化）
        'ground_relative_min_height': 0.2,  # 地面から0.2m以上
        'ground_relative_max_height': 2.0,  # 地面から2.0m以下
        # Euclidean clustering (クラスタリング)
        'enable_clustering': False,  # 一時的に無効化（処理高速化）
        'cluster_tolerance': 0.1,  # 10cm以内を同一クラスタとする
        'min_cluster_size': 200    # 最小点群数200点
    }
    
    if marker_coords:
        # 動的マーカーモード: /robot_pose を使用してワールド座標からロボット座標に変換
        region_filter_params['use_dynamic_marker'] = True
        region_filter_params['use_marker_diameter'] = False  # 動的モードでは不要
        region_filter_params['robot_pose_topic'] = '/robot_pose'
        region_filter_params['marker0_world_x'] = marker_coords['marker1_x']
        region_filter_params['marker0_world_y'] = marker_coords['marker1_y']
        region_filter_params['marker7_world_x'] = marker_coords['marker2_x']
        region_filter_params['marker7_world_y'] = marker_coords['marker2_y']
        region_filter_params['robot_pose_timeout'] = 2.0  # ロボットポーズのタイムアウト（秒）
        ld.add_action(LogInfo(msg=f"[scrap_detection] Using dynamic marker mode with /robot_pose"))
        ld.add_action(LogInfo(msg=f"[scrap_detection] Marker '{marker_coords['marker1_id']}' at ({marker_coords['marker1_x']:.3f}, {marker_coords['marker1_y']:.3f})"))
        ld.add_action(LogInfo(msg=f"[scrap_detection] Marker '{marker_coords['marker2_id']}' at ({marker_coords['marker2_x']:.3f}, {marker_coords['marker2_y']:.3f})"))
    else:
        # フォールバック: 固定円筒領域
        region_filter_params['use_dynamic_marker'] = False
        region_filter_params['use_marker_diameter'] = False
        region_filter_params['forward_offset'] = 0.0
        region_filter_params['left_offset'] = 1.5
        region_filter_params['fixed_radius'] = 1.0
        ld.add_action(LogInfo(msg='[scrap_detection] Warning: Could not load marker coordinates, using fixed cylinder'))

    region_filter = Node(
        package='nedo_localization', executable='hesai_region_filter_node', name='hesai_region_filter_node', output='screen',
        parameters=[region_filter_params]
    )

    highest_point = Node(
        package='nedo_localization', executable='highest_point_node', name='highest_point_node', output='screen',
        parameters=[{'input_topic': '/scrap_filtered_points'}]
    )

    ld.add_action(LogInfo(msg='[scrap_detection] Launch start (外部で Hesai ドライバを起動してください)'))
    ld.add_action(static_map_tf)
    ld.add_action(static_odom_footprint_tf)
    ld.add_action(static_base_footprint_tf)
    ld.add_action(lidar_tf)
    ld.add_action(region_filter)
    ld.add_action(highest_point)
    # Hesai ドライバは別ターミナルで: ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p config_path:=<path>

    return ld