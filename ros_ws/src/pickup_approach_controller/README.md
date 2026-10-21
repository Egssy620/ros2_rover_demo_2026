# pickup_approach_controller

巡回走行を一周して廃小型家電山へ戻った後、外部ノードが計算した車体中心の目標姿勢へ全方向移動車両を接近・停止させるROS 2 Humble向けC++パッケージです。

## 現在の実装範囲

- `/odometry/filtered` の融合自己位置・姿勢・共分散を一つの入力として使用
- 現在姿勢を経路へ投影して進捗を求める、時間非依存の経路追従
- 車体座標系の `vx`、`vy`、`omega` を生成
- 速度、加速度、ジャーク、スナップと角運動側の各制限
- 自己位置共分散と短時間窓を用いた到着判定
- 山側の `too_close` が真、または監視入力が途絶した場合の停止
- 走行権限サービスと計算専用モード

経路計画器、車体全体と梁の3次元衝突判定、巡回側との最上位速度Muxは次の実装単位です。本ノードは `/cmd_vel` へ直接出力せず、`/pickup_approach/cmd_vel` へ出力します。

## 安全な初期設定

起動時の走行権限はOFFです。さらに `publish_motion_commands` の既定値も `false` なので、受信・計算・ログ確認をしても実速度出力は常にゼロです。計算結果は `/pickup_approach/debug_cmd_vel` で確認できます。

## 入出力

| 種別 | 名前 | 型 | 内容 |
|---|---|---|---|
| Subscribe | `/odometry/filtered` | `nav_msgs/msg/Odometry` | 地図座標の融合自己位置、車体速度、共分散 |
| Subscribe | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 統合版`master_node`が発行する車体中心の目標位置・姿勢 |
| Subscribe | `/pickup_approach/path` | `nav_msgs/msg/Path` | 地図座標の計画経路 |
| Subscribe | `/scrap_pile_skirt_node/too_close` | `std_msgs/msg/Bool` | 山への接近禁止 |
| Publish | `/pickup_approach/cmd_vel` | `geometry_msgs/msg/Twist` | 最上位Muxへ渡す車体速度 |
| Publish | `/pickup_approach/debug_cmd_vel` | `geometry_msgs/msg/Twist` | 出力禁止中も確認できる計算結果 |
| Publish | `/pickup_approach/result` | `std_msgs/msg/String` | `planned`、`arrived`、`cancelled`など |
| Publish | `/pickup_approach/status` | `std_msgs/msg/String` | 制御状態と停止理由 |
| Service | `~/set_enabled` | `std_srvs/srv/SetBool` | 山接近側の走行権限 |
| Service | `~/cancel` | `std_srvs/srv/Trigger` | 現在の目標と経路を破棄 |

## Ubuntu 22.04 / ROS 2 Humbleでのビルド

```bash
cd ~/NEDO_ROS_2026_integrated_ver-main/ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pickup_approach_controller
source install/setup.bash
colcon test --packages-select pickup_approach_controller
colcon test-result --verbose
```

## 起動

```bash
ros2 launch pickup_approach_controller pickup_approach_controller.launch.py
```

まず `publish_motion_commands: false` のまま、自己位置、目標、経路、`too_close` を再生して `/pickup_approach/debug_cmd_vel` と状態を確認します。実機出力を有効にする前に、最上位Mux、オンライン車体外形監視、非常停止系を結線します。

走行要求は権限取得後にだけ受理します。計算専用試験でも、最初に次を実行してから新しい目標と経路を発行します。権限取得時には、それ以前の目標・経路を消去します。

```bash
ros2 service call /pickup_approach_controller/set_enabled std_srvs/srv/SetBool "{data: true}"
```
