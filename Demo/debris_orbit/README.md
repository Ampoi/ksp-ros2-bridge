# debris_orbit デモ

3D LiDARの点群からデブリ候補をクラスタリングし、その中心を基準とする円を追従しながら、LiDARの実際の光軸を対象へ向け、30度おきにRGB画像をPNG保存するROS 2 Jazzyパッケージです。

現行ROS2 for KSPのactive-vessel API（`/ksp_vessel`名前空間、種類別センサーTopic）に対応しています。誘導ノードはRCSやWrenchを直接扱わず、目標姿勢・位置・速度を`ControlSetpoint`として出力します。独立した6DoF制御ノードだけが、その目標とGround TruthからBody Wrenchを作ります。

```text
LiDAR認識・周回/探索誘導 → ControlSetpoint → 6DoF制御 → Body Wrench → KSP bridge/RCS
```

> このデモはKSPの物理を使う実験用コントローラです。最初はクイックセーブを取り、RCSを十分搭載し、低い速度・大きい周回半径から調整してください。LiDARからは物体固有IDが得られないため、最大の点群クラスタを対象として追跡します。

## 機体の準備

1. 操作機体に3D LiDAR、`Kerbal ROS2 RGB Camera`、全6軸を制御できるRCSを搭載します。
2. VAB/SPHのPart Action WindowでLiDARのSensor IDを`front_lidar`、カメラを`orbit_camera`にします。別のIDを使う場合は起動引数で変更できます。
3. LiDARとカメラの前方をデブリへ向けます。LiDARは機体に対して回転して搭載しても、TFの取付姿勢から光軸を自動補正します。撮影も正対させたい場合はカメラの光軸をLiDARと揃えてください。
4. 対象デブリ以外がLiDAR視野へ大きく入らない場所でFlightを開始します。

デブリのIDは指定しません。ノードは3D LiDAR点群を距離でクラスタリングし、初回は点数の多いクラスタを選び、その後は推定位置と速度に最も近いクラスタを同じデブリとして追跡します。

`platform_id`は状態Topicと画像ファイル名に使う機体の論理IDです。`lidar_sensor_id`と`camera_sensor_id`がKSP上の実パーツのSensor IDとの紐付けです。これらは後からYAMLまたはlaunch引数で変更できます。

## ビルド

リポジトリ直下で次を実行すると、bridge、interfaces、Nav2と一緒にこのデモも`~/ros2_ws`へ同期・ビルドされます。

```bash
./dev_sync.sh
source ~/ros2_ws/install/setup.bash
```

デモだけを手動で配置する場合:

```bash
mkdir -p ~/ros2_ws/src
cp -r Demo/debris_orbit ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-select ksp_ros2_interfaces debris_orbit
source install/setup.bash
```

## 起動

ターミナル1でbridgeを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

KSPでFlightに入り、次のTopicが存在することを確認します。

```bash
ros2 topic list | grep -E 'front_lidar|orbit_camera|ground_truth|body_wrench'
```

ターミナル2でデモを起動します。`enabled:=true`を明示するまで指令を出しません。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch debris_orbit debris_orbit.launch.py \
  enabled:=true \
  platform_id:=inspection_vehicle \
  lidar_sensor_id:=front_lidar \
  camera_sensor_id:=orbit_camera
```

KSPへ推力を送らず認識・誘導だけを確認する場合は、出力先を別Topicへ変更できます。

```bash
ros2 launch debris_orbit debris_orbit.launch.py \
  enabled:=true \
  platform_id:=inspection_vehicle \
  body_wrench_topic:=/ksp_vessel/demos/debris_orbit/dry_run_wrench
```

launchは`debris_orbit`（認識・誘導）と`debris_orbit_controller`（目標追従）の2ノードを起動します。既定ではまず半径15 mの円周上へ移動して対象へ正対し、位置・速度・姿勢が許容範囲へ収束した地点を0度として、3 deg/sで1周します。`0, 30, ..., 330`度の12枚を`$PWD/debris_orbit_captures`へ保存し、完了後はIDLE目標を送り続けて停止します。

状態はJSONで確認できます。

```bash
ros2 topic echo /ksp_vessel/demos/debris_orbit/inspection_vehicle/status
```

主な状態は`disabled`、`waiting_for_sensor_data`、`detumbling`、`acquiring_target`、`searching`、`approaching_orbit`、`orbiting`、`complete`です。対象が見えている間は、周回開始前も含めLiDARが対象を向く姿勢目標を維持します。見失うと並進を止め、最後の視線を中心に小角度で往復探索します。目標を記憶していない場合は現在のLiDAR方向を中心に探索するため、一方向へ回転し続けません。

高速回転中は目標指向を止めて角速度減衰を優先します。誘導目標、TF、姿勢、速度のいずれかが途切れると制御ノードは0.5秒以内にゼロWrenchへ移る独立failsafeを持ちます。`Ctrl+C`で終了した場合はbridge側のfailsafeも指令を解除します。

点群はLiDAR→`base_link`の接続済み最新TFが届くまでノード内キューで待機します。搭載TFはモデル更新5 Hz、点群は通常10 Hz以上なので、準固定の搭載姿勢へ点群と完全一致するtimestampは要求しません。一方、機体のワールド位置・姿勢はGround Truth pose/twistから点群の`header.stamp`へ補間して算出します。起動直後でも点群コールバックをブロックしません。

`TF base_link <- ksp_*_lidar`待ちで点群が破棄され続ける場合は、古いKSPプラグインまたはbridgeが動作しています。`./dev_sync.sh`後にKSPを完全に再起動し、bridgeも再起動してください。正常時は`base_link -> ksp_*_link_0000 -> ... -> ksp_*_lidar`が接続されます。

## 設定

既定値は[`config/debris_orbit.yaml`](config/debris_orbit.yaml)にあります。コピーして変更し、次のように読み込めます。

```bash
ros2 launch debris_orbit debris_orbit.launch.py \
  config_file:=/absolute/path/to/my_debris_orbit.yaml \
  enabled:=true
```

よく調整する値:

| パラメータ | 内容 | 既定値 |
|---|---|---:|
| `orbit_radius` | 対象中心からの周回半径[m] | `15.0` |
| `vessel_topic_prefix` | ROS2 for KSPの機体系Topicルート | `/ksp_vessel` |
| `angular_speed_deg_s` | 周回角速度[deg/s] | `3.0` |
| `orbit_direction` | 周回方向。`1`または`-1` | `1` |
| `orbit_plane_normal` | `ground_truth_enu`で表した周回面法線 | `[0,0,1]` |
| `target_cluster_index` | 初回検出時に点数順で何番目のクラスタを使うか | `0` |
| `cluster_tolerance` | 同一物体とみなす点間距離[m] | `1.5` |
| `target_center_offset` | 観測表面から視線奥へ寄せる中心補正[m] | `0.0` |
| `target_acquisition_samples` | 制御開始前に必要な連続対象観測数 | `3` |
| `search_yaw_amplitude_deg` | 喪失時の左右探索振幅 | `8.0` |
| `search_pitch_amplitude_deg` | 喪失時の上下探索振幅 | `4.0` |
| `search_period_sec` | 探索が中心へ戻る周期[s] | `8.0` |
| `search_memory_timeout_sec` | 最終対象位置を探索中心に使う時間[s] | `10.0` |
| `detumble_enter_rate_deg_s` | この角速度を超えたらデタンブル開始[deg/s] | `10.0` |
| `detumble_exit_rate_deg_s` | この角速度未満で対象探索へ移行[deg/s] | `3.0` |
| `arrival_*_tolerance` | 0度撮影・周回開始の収束判定 | YAML参照 |
| `transform_wait_timeout_sec` | 点群と同時刻の搭載TFを待つ上限[s] | `1.0` |
| `cloud_queue_size` | TF待ち点群の最大保持数 | `20` |
| `output_directory` | PNG出力先。相対指定は起動時の`$PWD`基準 | `debris_orbit_captures` |

`debris_orbit_controller`側の主な調整値は`position_kp`、`velocity_kd`、`attitude_kp`、`angular_kd`、`max_force`、`max_torque`です。`command_ramp_sec`は制御モード切替時の立上げ、`setpoint_timeout_sec`は誘導ノード停止時のfailsafe時間です。探索ロジックを変えても推力配分の実装へ触れず、機体を変えた場合はこの制御ノード側だけを調整できます。

LiDARの最大クラスタが別物体になる場合は視野を整理するか、`target_cluster_index`を変更してください。点群は対象の見えている表面だけなので、形状中心との差が大きい場合は`target_center_offset`を正値にします。

## 入出力

| 方向 | Topic | 型 |
|---|---|---|
| Subscribe | `/ksp_vessel/lidar_3d/<lidar_sensor_id>/points` | `sensor_msgs/msg/PointCloud2` |
| Subscribe | `/ksp_vessel/camera/<camera_sensor_id>/image_raw` | `sensor_msgs/msg/Image` |
| Subscribe | `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` |
| Subscribe | `/ksp_vessel/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` |
| Publish（誘導） | `/ksp_vessel/demos/debris_orbit/<platform_id>/setpoint` | `ksp_ros2_interfaces/msg/ControlSetpoint` |
| Subscribe（制御） | `/ksp_vessel/demos/debris_orbit/<platform_id>/setpoint` | `ksp_ros2_interfaces/msg/ControlSetpoint` |
| Publish（制御） | `/ksp_vessel/body_wrench` | `geometry_msgs/msg/WrenchStamped` |
| Publish | `/ksp_vessel/demos/debris_orbit/<platform_id>/status` | `std_msgs/msg/String` (JSON) |

各センサーIDはbridgeと同じ規則で小文字のROS名へ正規化します。任意の既存Topicへ直接接続したい場合は、YAMLの各Topicパラメータを設定してください。`setpoint_topic`は両ノードで同じ値にする必要があります。

## 制約

- デブリの選択にKSPの固有IDや名前は使用しません。複数物体が視野にある場合は、初回の`target_cluster_index`で点数順の対象を選択します。
- Ground Truthは制御に使用します。LiDARのみの自己位置推定デモではありません。
- 対象中心の速度は連続するLiDAR観測から推定し、KSP低軌道の大きな公転速度を打ち消して相対運動を制御します。点群を継続取得できない対象や急加速する対象には対応しません。
- RCS配置、質量、SASとの競合に応じてゲインと上限を調整してください。SASは原則OFFで試してください。
