# ros2_rover_demo_2026

NEDOローバー統合システムのROS 2ワークスペースです。既存の巡回走行系に加え、巡回完了後の廃小型家電山への接近・停止制御を開発します。

## 対象環境

- Ubuntu 22.04
- ROS 2 Humble
- colcon

## ビルド

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 山接近制御パッケージ

`pickup_approach_controller`は、上流で融合済みの`/odometry/filtered`と外部計算済みのピックアップ目標・経路を受け、全方向移動車両の`vx`、`vy`、`omega`を生成します。

起動時は安全のため走行権限と実速度出力が無効です。最上位の走行権限Muxとオンライン衝突監視が完成するまでは、`publish_motion_commands: false`のままデバッグ出力を確認してください。

```bash
colcon build --symlink-install --packages-select pickup_approach_controller
source install/setup.bash
colcon test --packages-select pickup_approach_controller
colcon test-result --verbose
ros2 launch pickup_approach_controller pickup_approach_controller.launch.py
```

詳細は[`ros_ws/src/pickup_approach_controller/README.md`](ros_ws/src/pickup_approach_controller/README.md)を参照してください。
