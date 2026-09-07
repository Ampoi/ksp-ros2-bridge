# 2D LiDAR MappingとNav2

`ksp_nav2_bringup`は、2D LiDARだけからscan-to-scan odometryを推定し、SLAM ToolboxとNav2へ渡す独立パッケージです。KSP modと`ksp_lidar_bridge`にはNav2固有コードを入れていません。

## 構成

| 段階 | 入出力 | 役割 |
|---|---|---|
| LiDAR odometry | `LaserScan` → `/ksp_nav2/odom` | 連続スキャンのICPで平面移動量を推定 |
| Mapping | `/ksp_nav2/scan` + LiDAR odometry → `/map` | SLAM Toolboxによる地図生成とloop closure |
| Localization | 保存地図 + `/ksp_nav2/scan` → `map -> lidar_odom` | AMCLによる自己位置推定 |
| Navigation | map / costmap / odometry → `/cmd_vel` | Nav2の経路計画と追従 |
| KSP制御 | `/cmd_vel` → authority lease → `/control/wrench_command` | 平面速度誤差をforce / yaw torqueへ変換 |

Nav2のTFは`map -> lidar_odom -> nav_base_link`です。bridgeのGround Truth TopicとTFは購読しないため、自己位置推定へ真値は混ざりません。

## インストールと起動

保存データ`ROS2 debug`のroverで、KSP起動からMapping、走行入力、地図保存、AMCL、Nav2 Actionまで確認済みです。save内の機体名は`rover A`ではなく`rober A`、2D LiDARのSensor IDは`lidar_755b97e1`です。コピーして上から実行できる複数ターミナルの完全な手順は[`Ros2/ksp_nav2_bringup/README.md`](https://github.com/Ampoi/KSP_ROS2/blob/main/Ros2/ksp_nav2_bringup/README.md#rober-aで上から順に実行する手順)にあります。

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
./dev_sync.sh --skip-ksp-build --skip-ksp-sync
```

bridgeとKSP Flightを起動した後、実際のSensor IDを指定します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ksp_nav2_bringup mapping.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan
```

RViz2の`Nav2 Goal`で目的位置を送れます。作成した地図の保存:

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/ksp_map"
```

広いKSP地図では既定の2秒で購読が間に合わないことがあるため、次のように保存タイムアウトを明示してください。

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/ksp_map" \
  --ros-args -p save_map_timeout:=15.0
```

保存地図を使う場合:

```bash
ros2 launch ksp_nav2_bringup navigation.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan \
  map:="$PWD/ksp_map.yaml"
```

この場合はRViz2の`2D Pose Estimate`で初期位置を与えてから`Nav2 Goal`を使います。

CLIだけで確認する場合も、`/initialpose`をpublishして`map_server`、`amcl`、`controller_server`、`planner_server`、`behavior_server`、`bt_navigator`がすべて`active [3]`になってから`/navigate_to_pose`へgoalを送ります。初期姿勢未設定の間に表示される`Please set the initial pose`は想定内です。

## 機体別の調整

設定は`ksp_nav2_bringup/params/nav2_params.yaml`にまとまっています。機体寸法に合わせて`robot_radius`、推進力に合わせてcontrollerのgainとforce / torque上限、運動性能に合わせてDWBの速度・加速度上限を調整します。`controller_id`と`control_priority`は他controllerとの調停、`lease_duration_sec`と`lease_renew_period_sec`は通信断時の所有権解放を決めます。

LiDARの+Xが機体`base_link`の+Xと異なる場合、`nav_to_body_yaw`へLiDAR座標から機体座標へのyaw回転[rad]を設定します。Nav2は2DなのでLiDARは水平固定が前提で、高度・roll・pitchは別の飛行制御系が担当します。

planar controllerは`cmd_vel`が有効な間だけ`/ksp_vessel/lifecycle`で示された実`vessel_id`へleaseを取得し、停止後にreleaseします。mapping起動だけでは機体を占有しません。SAS排他とKSP側安全limitは同じ正式制御APIが適用されます。

KSP側の連続作動limitへ到達した場合、planar controllerは既定0.75秒のゼロWrenchを挟んでから追従を再開します。これにより長いNav2 goalでも安全limitを無効化せず、連続噴射・連続駆動時間を区切ります。
