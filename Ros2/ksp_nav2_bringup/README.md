# ksp_nav2_bringup

KSPの2D LiDARを使って、LiDAR odometry、SLAM Toolbox、Nav2、RViz2を接続する独立したROS2パッケージです。`ksp_lidar_bridge`やKSP modのコードは変更せず、標準ROS2 Topicだけで接続します。

## データ経路

```text
/ksp_vessel/lidar_2d/<sensor_id>/scan
  -> scan-to-scan ICP
  -> /ksp_nav2/odom + lidar_odom -> nav_base_link
  -> SLAM Toolbox (mapping) または AMCL (保存地図でのlocalization)
  -> Nav2
  -> /cmd_vel
  -> lease-aware planar velocity controller
  -> /ksp_vessel/control/wrench_command
  -> ksp_lidar_bridge
```

`/ksp_vessel/ground_truth/*`と`ground_truth_enu -> base_link`はNav2の入力に使いません。LiDAR推定用のTFは`map -> lidar_odom -> nav_base_link`という別ツリーに分けているため、bridgeが配信する真値TFとも競合しません。

## 依存パッケージ

ROS2 Jazzyで次をインストールします。

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

## `rober A`で上から順に実行する手順

以下は実際のKSP保存データ`ROS2 debug`で確認した手順です。ユーザー向けの機体名は rover A ですが、現在のsave内の名前は **`rober A`** なので、KSP上でこの名前の機体を選びます。LiDARのSensor IDは`lidar_755b97e1`です。

最初にリポジトリ直下で同期します。

```bash
./sync.sh
```

ターミナル1で通常どおりKSPを起動し、保存済みのローバーを開いてください。

Flight画面が開いた後、ターミナル2でbridgeを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge
```

ターミナル3で機体名とLiDARを確認します。scanは約9 Hzです。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /ksp_vessel/lifecycle \
  ksp_ros2_interfaces/msg/VesselLifecycle \
  --qos-durability transient_local --once
ros2 topic hz /ksp_vessel/lidar_2d/lidar_755b97e1/scan --window 20
```

`ros2 topic hz`を`Ctrl-C`で止め、同じターミナル3でMappingを起動します。

```bash
ros2 launch ksp_nav2_bringup mapping.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/lidar_755b97e1/scan \
  use_rviz:=false
```

RViz2も使う場合だけ`use_rviz:=true`へ変更します。ターミナル4で、SLAMとNav2の全ノード、地図、odometryを確認します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 lifecycle get /slam_toolbox
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 topic echo /map_metadata nav_msgs/msg/MapMetaData --once
ros2 topic echo /ksp_nav2/odom nav_msgs/msg/Odometry --once
ros2 action list | grep -E 'navigate_to_pose|navigate_through_poses'
```

低速の走行入力を確認する場合は、同じターミナル4から次を実行します。入力終了後、controllerはゼロWrenchへ移行してleaseを解放します。

```bash
ros2 topic pub --rate 5 --times 10 \
  /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
ros2 topic echo /ksp_vessel/control/authority/state \
  ksp_ros2_interfaces/msg/ControlAuthorityState \
  --qos-durability transient_local --once
```

地図を保存します。roverの広い地図では既定の2秒で間に合わないことがあるため、15秒を明示します。

```bash
mkdir -p "$PWD/.demo-output"
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/.demo-output/rober_a_map" \
  --ros-args -p save_map_timeout:=15.0
ls -lh "$PWD/.demo-output/rober_a_map.yaml" \
  "$PWD/.demo-output/rober_a_map.pgm"
```

ターミナル3のMappingを`Ctrl-C`で終了し、保存地図を使うNavigationへ切り替えます。

```bash
ros2 launch ksp_nav2_bringup navigation.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/lidar_755b97e1/scan \
  map:="$PWD/.demo-output/rober_a_map.yaml" \
  use_rviz:=false
```

ターミナル4で保存時の開始位置をAMCLへ設定します。別の場所から開始した場合は`x`、`y`とorientationを実位置へ合わせてください。

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}'
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 topic echo /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once
```

全ノードが`active [3]`になった後、RViz2の`Nav2 Goal`、または次のAction commandで近距離goalを送れます。目標座標が保存地図上のfree spaceであることを確認してください。

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

## Mappingと同時にNav2を使う

bridgeを起動し、KSPのFlightで2D LiDAR Topicが見える状態にしてから起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ksp_nav2_bringup mapping.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan
```

RViz2の`Nav2 Goal`ツールで地図上の位置と向きを指定すると、`NavigateToPose` Actionを通じて機体を移動できます。地図を保存する場合:

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/ksp_map"
```

大きな地図では`--ros-args -p save_map_timeout:=15.0`も指定してください。

## 保存した地図でNavigationする

```bash
ros2 launch ksp_nav2_bringup navigation.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan \
  map:="$PWD/ksp_map.yaml"
```

RViz2の`2D Pose Estimate`で初期位置を与えてから`Nav2 Goal`を指定します。

## 調整項目

`params/nav2_params.yaml`で少なくとも次を実機体に合わせてください。

- `robot_radius`: 機体の平面半径。既定1.0 m。
- `linear_gain` / `angular_gain`: 速度誤差からN / N·mへの変換係数。
- `max_planar_force` / `max_yaw_torque`: bridgeへ要求する上限。
- `controller_id` / `control_priority`: authorityで識別・調停するcontroller。
- `lease_duration_sec` / `lease_renew_period_sec`: 制御leaseの期限と更新周期。
- `nav_to_body_yaw`: LiDARの+Xと機体`base_link`の+Xがずれる場合のyaw補正[rad]。
- DWBの`max_vel_*`と`acc_lim_*`: 機体が安定して追従できる速度・加速度。
- ICPの`max_correspondence_distance`: 1スキャン間の最大移動量と環境寸法に合わせる値。
- ICPの`max_rmse`と1スキャン当たりの移動・回転上限: 誤対応をodometryへ混ぜないための棄却条件。

既定では`nav_base_link`をLiDAR原点、LiDARの+XをNav2の前方とみなします。LiDARを水平に固定し、十分な静止物が全周に入るようにしてください。

## 安全と制約

- Nav2とこのcontrollerは2D専用です。高度、roll、pitchは制御しません。
- `/cmd_vel`を受け取るまでauthority leaseを取得せず、Wrenchもpublishしません。
- 有効な`cmd_vel`中だけlifecycleの実`vessel_id`へleaseを取得します。command停止後は明示的にreleaseします。
- LiDAR odometryが途切れるとゼロWrenchへ移り、KSP側のtimeoutも最終停止境界になります。
- SAS排他、角速度・変化率・連続噴射limit、emergency stopはKSP内の正式APIが適用します。
- 連続作動limitへ到達した場合、controllerはleaseを保ったまま既定0.75秒のゼロWrenchを送り、安全状態をリセットしてから追従を再開します。
- scan-to-scan推定なので、特徴の乏しい場所、大部分が動く物体、スキャン周期に対して大きすぎる移動ではodometryが失敗します。
- 最初は低い力・トルク上限で、広い場所から調整してください。
