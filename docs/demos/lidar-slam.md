# 2D LiDARとSLAM

2D LiDARから平面の自己位置を推定し、SLAM Toolboxで地図を作成します。作成した地図の保存、AMCLによる位置推定、RVizのNav2 Goalによる走行までを試せます。

## 1. デモをインストールする

[Getting Started](../guide/getting-started.md)でMODとbridgeを導入した後、PyLoNリポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  Ros2/pylon_interfaces Ros2/pylon_bridge Ros2/pylon_vehicle_control \
  Demo/pylon_demo_lidar_slam \
  --ignore-src --rosdistro jazzy -y
./sync.sh --skip-ksp-build --skip-ksp-sync --demo lidar_slam
source ~/ros2_ws/install/setup.bash
```

SLAM Toolbox、Nav2、RVizも導入されます。

## 2. 機体と走行場所を準備する

1. 平坦な地面を走行できる機体に2D LiDARを水平に固定します。
2. Sensor IDを`front_lidar`にし、LiDARの前方を機体の前方へ合わせます。
3. 壁、建物、構造物などが複数方向に見える場所でFlightを開始し、静止します。
4. センサーとUDP配信を有効にします。走行も試す場合はモーター・操舵・燃料・電源を確認します。

推定上のベースフレームはLiDARの原点です。取付位置は機体中心付近にし、走行中もLiDARの高さ・roll・pitchをできるだけ一定に保ちます。水平スキャンに地面以外の形状が入る場所を選んでください。

地図作成だけならKSPの手動操作で移動できます。Nav2走行の既定設定は前後移動とyaw回転を使い、横移動の目標は生成しません。旋回動作にはその場で向きを変える能力も必要です。その場旋回ができない前輪操舵ローバーには、[月面Nav2](mun-nav2.md)の走行方式が適しています。

## 3. bridgeとスキャンを確認する

ターミナルAでbridgeを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pylon_bridge udp_bridge \
  --host 127.0.0.1 --port 49010 --disable-ground-truth
```

ターミナルBで確認します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --once /ksp_vessel/lifecycle
ros2 topic hz /ksp_vessel/lidar_2d/front_lidar/scan
```

機体状態がACTIVEでスキャンを受信できたら、`ros2 topic hz`を`Ctrl-C`で終了します。以下の`scan_topic`には、実際のSensor IDを含むTopic名を指定してください。

## 4. SLAMで地図を作る

ターミナルBで実行します。bridgeはターミナルAで動かしたままにします。

```bash
ros2 launch pylon_demo_lidar_slam mapping.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan \
  use_rviz:=true
```

RVizのFixed Frameは`map`です。地図表示には`/map`、LaserScan表示には`/pylon/lidar_slam/scan`を指定します。最初は静止してスキャンと地図が重なることを確認し、KSPの手動操作でゆっくり動かしてください。

別ターミナルで、SLAMとodometryを確認できます。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 lifecycle get /slam_toolbox
ros2 topic echo --once /map_metadata
ros2 topic echo --once /pylon/lidar_slam/odom
```

`slam_toolbox`が`active`で、移動に応じて地図とodometryが更新されれば地図作成が動いています。スキャン間の動きが大きいと対応付けが失敗するため、低速で移動します。

## 5. 地図を保存する

機体を停止させ、mappingを起動したまま別ターミナルで実行します。

```bash
mkdir -p "$HOME/pylon_maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/pylon_maps/site" \
  --ros-args -p save_map_timeout:=15.0
ls "$HOME/pylon_maps/site.yaml" "$HOME/pylon_maps/site.pgm"
```

保存後、mappingのターミナルで`Ctrl-C`を押します。bridgeは動かしたままにします。

## 6. 保存地図でNav2を起動する

```bash
ros2 launch pylon_demo_lidar_slam navigation.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan \
  map:="$HOME/pylon_maps/site.yaml" \
  use_rviz:=true
```

1. RVizの`2D Pose Estimate`で、保存地図上の現在位置と向きを指定します。
2. スキャンが地図上の壁・構造物と重なるまで位置と向きを確認します。
3. `Nav2 Goal`で近い空き領域と到着時の向きを指定します。
4. 経路、実際の走行、制御権の状態を確認します。

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /amcl_pose
ros2 topic echo /ksp_vessel/control/authority/state
```

各ノードが`active`になってからゴールを指定してください。Nav2から速度指令が届くと、制御器が操作機体のleaseを取得し、Body Wrenchを送信します。

mapping中にもNav2 Goalを使えます。地図作成と保存地図でのnavigationは、どちらか一方のlaunchだけを起動してください。

## 停止と再開

RVizのNavigationパネルからゴールを取り消します。制御器は速度指令が途切れると制動し、leaseを解放します。機体の停止を確認してlaunchを`Ctrl-C`で終了し、最後にbridgeを止めます。

機体切替、セッション変更、制御権喪失、odometryの欠測後は制御を停止したまま保持します。Nav2のゴールを取り消し、launch全体を起動し直してください。保存地図を使う場合は`2D Pose Estimate`で現在位置を設定し直します。mappingを再起動すると新しい地図になるため、残したい地図は終了前に保存します。

## 起動引数と調整

| 引数 | 既定値 | 用途 |
|---|---|---|
| `scan_topic` | `/ksp_vessel/lidar_2d/front_lidar/scan` | KSPの2Dスキャン入力 |
| `use_rviz` | `true` | RVizを起動 |
| `use_sim_time` | `false` | ROSの時計設定。この手順では既定値を使用 |
| `nav2_params` | 同梱`params/nav2_params.yaml` | 機体寸法、走行、ICP、制御器の設定 |
| `slam_params` | 同梱`params/slam_toolbox.yaml` | mappingの設定。mapping起動時のみ |
| `map` | 指定必須 | 保存地図YAML。navigation起動時のみ |

機体に合わせる場合は同梱設定をコピーし、`nav2_params:=/absolute/path/to/nav2_params.yaml`で指定します。

```bash
cp "$(ros2 pkg prefix --share pylon_demo_lidar_slam)/params/nav2_params.yaml" \
  "$HOME/pylon_maps/nav2_params.yaml"
```

| 設定 | 調整する内容 |
|---|---|
| 両costmapの`robot_radius` | LiDAR原点から見た機体の平面半径。既定1.0 m |
| `FollowPath.max_vel_x` / `max_speed_xy` | 前進・平面速度上限。既定0.5 m/s |
| `max_planar_force` / `max_yaw_torque` | 機体へ要求する力・トルクの上限 |
| `linear_gain` / `angular_gain` | 速度誤差に対する制御ゲイン |
| `nav_to_body_yaw` | LiDAR前方から機体前方へのyaw補正［rad］ |
| `max_correspondence_distance` / `max_rmse` | ICPの対応距離と棄却条件 |

## 主な出力と座標系

| 出力 | 内容 |
|---|---|
| `/pylon/lidar_slam/scan` | 推定フレームへ接続したLaserScan |
| `/pylon/lidar_slam/odom` | 2D ICPを積算したOdometry |
| `/map` | SLAM Toolboxの地図、または読み込んだ保存地図 |
| `/amcl_pose` | 保存地図内の推定姿勢（navigation時） |
| `/cmd_vel` | Nav2の`geometry_msgs/msg/Twist`指令 |

TFは`map → pylon_slam_odom → pylon_slam_base_link`です。地図作成と保存地図での位置推定には、それぞれSLAM ToolboxとAMCLを使います。Jazzyの速度指令は`Twist`に合わせています。[Nav2の速度メッセージ設定](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/)も参照してください。

特徴の乏しい場所、動く物体が視野の大部分を占める場所、大きな傾斜では推定が不安定になります。地図とスキャンがずれる場合は走行を止め、取付方向、周囲の形状、移動速度、センサー周期を確認してください。
