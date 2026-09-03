# debris_orbit デモ

3D LiDARの点群からデブリ候補をクラスタリングし、その中心を基準とする円を追従しながら、機体の`+X`軸を対象へ向け、30度おきにRGB画像をPNG保存するROS 2 Jazzyパッケージです。

> このデモはKSPの物理を使う実験用コントローラです。最初はクイックセーブを取り、RCSを十分搭載し、低い速度・大きい周回半径から調整してください。LiDARからは物体固有IDが得られないため、最大の点群クラスタを対象として追跡します。

## 機体の準備

1. 操作機体に3D LiDAR、`Kerbal ROS2 RGB Camera`、全6軸を制御できるRCSを搭載します。
2. VAB/SPHのPart Action WindowでLiDARのSensor IDを`front_lidar`、カメラを`orbit_camera`にします。別のIDを使う場合は起動引数で変更できます。
3. LiDARとカメラの前方、および機体の`+X`前方をデブリへ向けます。カメラを別方向へ取り付ける場合、このノードはカメラ光軸そのものではなく機体`+X`を対象へ向ける点に注意してください。
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
colcon build --packages-select debris_orbit
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

既定ではまず半径15 mの円周上へ移動して対象へ正対し、位置・速度・姿勢が許容範囲へ収束した地点を0度として、3 deg/sで1周します。`0, 30, ..., 330`度の12枚を`$PWD/debris_orbit_captures`へ保存し、完了後はゼロWrenchを送り続けて停止します。

状態はJSONで確認できます。

```bash
ros2 topic echo /debris_orbit/inspection_vehicle/status
```

主な状態は`disabled`、`waiting_for_sensor_data`、`target_lost`、`approaching_orbit`、`orbiting`、`complete`です。`Ctrl+C`で終了するとbridge側の0.5秒failsafeにより指令が解除されます。

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
| `angular_speed_deg_s` | 周回角速度[deg/s] | `3.0` |
| `orbit_direction` | 周回方向。`1`または`-1` | `1` |
| `orbit_plane_normal` | `ground_truth_enu`で表した周回面法線 | `[0,0,1]` |
| `target_cluster_index` | 初回検出時に点数順で何番目のクラスタを使うか | `0` |
| `cluster_tolerance` | 同一物体とみなす点間距離[m] | `1.5` |
| `target_center_offset` | 観測表面から視線奥へ寄せる中心補正[m] | `0.0` |
| `position_kp`, `velocity_kd` | 位置・速度PDゲイン | `120`, `350` |
| `attitude_kp`, `angular_kd` | 姿勢・角速度PDゲイン | `800`, `300` |
| `max_force`, `max_torque` | 指令上限[N, N·m] | `5000`, `3000` |
| `arrival_*_tolerance` | 0度撮影・周回開始の収束判定 | YAML参照 |
| `output_directory` | PNG出力先。相対指定は起動時の`$PWD`基準 | `debris_orbit_captures` |

LiDARの最大クラスタが別物体になる場合は視野を整理するか、`target_cluster_index`を変更してください。点群は対象の見えている表面だけなので、形状中心との差が大きい場合は`target_center_offset`を正値にします。

## 入出力

| 方向 | Topic | 型 |
|---|---|---|
| Subscribe | `/ksp_vessel/lidar_3d/<lidar_sensor_id>/points` | `sensor_msgs/msg/PointCloud2` |
| Subscribe | `/ksp_vessel/camera/<camera_sensor_id>/image_raw` | `sensor_msgs/msg/Image` |
| Subscribe | `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` |
| Subscribe | `/ksp_vessel/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` |
| Publish | `/ksp_vessel/body_wrench` | `geometry_msgs/msg/WrenchStamped` |
| Publish | `/debris_orbit/<platform_id>/status` | `std_msgs/msg/String` (JSON) |

任意の既存Topicへ直接接続したい場合はYAMLの`lidar_topic`または`camera_topic`を設定してください。

## 制約

- デブリの選択にKSPの固有IDや名前は使用しません。複数物体が視野にある場合は、初回の`target_cluster_index`で点数順の対象を選択します。
- Ground Truthは制御に使用します。LiDARのみの自己位置推定デモではありません。
- 対象中心の速度は連続するLiDAR観測から推定し、KSP低軌道の大きな公転速度を打ち消して相対運動を制御します。点群を継続取得できない対象や急加速する対象には対応しません。
- RCS配置、質量、SASとの競合に応じてゲインと上限を調整してください。SASは原則OFFで試してください。
