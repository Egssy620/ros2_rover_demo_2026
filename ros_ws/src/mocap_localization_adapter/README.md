# mocap_localization_adapter

OptiTrack `natnet_ros2` の `geometry_msgs/msg/PoseStamped` を、NEDO統合版が使う自己位置インターフェースへ変換します。

## 入出力

|方向|既定名|型|内容|
|---|---|---|---|
|入力|`/rover_1021/pose`|`geometry_msgs/msg/PoseStamped`|約100 Hz、`frame_id=world`のモーションキャプチャ姿勢|
|出力|`/odometry/filtered`|`nav_msgs/msg/Odometry`|`map`系の車体中心姿勢、車体速度、共分散|
|出力|`/robot_pose`|`geometry_msgs/msg/PoseStamped`|統合版`master_node`が監視する自己位置|
|TF|`map -> odom -> base_link`|TF2|`publish_tf:=true`のときだけ配信|

`/odometry/filtered` と `/robot_pose` は同じ入力姿勢・時刻から生成します。マスターを自己位置の中継ノードにはしません。

## 較正値

`map_from_world_*` はMotiveの`world`を試験場所の`map`へ合わせる変換です。`rigid_body_to_base_*` はOptiTrack剛体原点から車体中心`base_link`への変換です。入力例のzは約1.75 mだったため、未較正のzを車体中心の高さとしてそのまま使わないでください。

設定角度の単位はradです。平面走行用として、出力姿勢はyawだけを使います。

## 速度と共分散

並進速度と角速度は、変換後姿勢の差分を一次ローパスフィルターへ通して生成します。約100 Hzの位置差分は3 mmの観測ノイズを増幅するため、既定時定数は0.10 sです。

位置分散は `(0.003 m)^2`、yaw分散は `(0.3 deg)^2` を既定値にしています。速度分散は実測ログから再調整します。

## ビルドと起動

```bash
cd ~/ros2_rover_demo_2026/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mocap_localization_adapter
source install/setup.bash
ros2 launch mocap_localization_adapter mocap_localization_adapter.launch.py
```

起動直後の`publish_tf`は`false`です。次を確認し、同じTFを配信するノードがない場合だけ有効にします。

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /odometry/filtered --once
ros2 topic echo /robot_pose --once
```

TFを有効にする場合は設定ファイルの`publish_tf: true`を使用します。統合版のEKFと同時には起動しません。
