# position_estimator デモ

3D LiDARの点群だけから6DoFの自己位置を推定するROS 2 Jazzyパッケージです。連続する2スキャン間のpoint-to-point ICPで相対姿勢を求め、積算して`nav_msgs/msg/Odometry`とTFを配信します。推定そのものにGround Truthは使いませんが、採点専用に購読し、開始時点合わせのドリフト誤差をコンソールと専用Topicへ出します。

```text
/ksp_vessel/lidar_3d/<sensor_id>/points
  -> voxel downsample + range filter
  -> scan-to-scan ICP (6DoF)
  -> /ksp_position_estimator/odom + lidar_odom_3d -> estimator_base_link
  -> console echo + /ksp_position_estimator/error (vs ground truth, 評価のみ)
```

> scan-to-scanの積算なので、特徴の乏しい場所、動く物体が多い場所、スキャン周期に対して大きすぎる移動ではドリフト・失敗します。最初は静止状態で収束を確認し、ゆっくり動かして調整してください。

## 機体の準備

1. 操作機体に3D LiDARを搭載します。
2. VAB/SPHのPart Action WindowでLiDARのSensor IDを`front_lidar`にします。別のIDを使う場合は起動引数で変更できます。
3. LiDARの周囲に十分な静止物（地形・建物・他の機体など）が入る場所でFlightを開始します。

## ビルド

リポジトリ直下で次を実行すると、bridge、interfaces、Nav2、debris_orbitと一緒にこのデモも`~/ros2_ws`へ同期・ビルドされます。

```bash
./dev_sync.sh
source ~/ros2_ws/install/setup.bash
```

デモだけを手動で配置する場合:

```bash
mkdir -p ~/ros2_ws/src
cp -r Demo/position_estimator ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-select position_estimator
source install/setup.bash
```

## 起動

ターミナル1でbridgeを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

KSPでFlightに入り、点群Topicが存在することを確認します。

```bash
ros2 topic list | grep -E 'lidar_3d|position_estimator'
ros2 topic hz /ksp_vessel/lidar_3d/front_lidar/points
```

ターミナル2でデモを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch position_estimator position_estimator.launch.py \
  lidar_sensor_id:=front_lidar
```

RViz2で確認する場合はFixed Frameを`lidar_odom_3d`にし、Odometry表示のTopicを`/ksp_position_estimator/odom`にします。推定のTFは`lidar_odom_3d -> estimator_base_link`という独立ツリーのため、bridgeが配信する真値TF（`ground_truth_enu -> base_link`）とは競合しません。

起動中のターミナルには約2秒おきに推定位置と誤差が出ます。

```text
[position_estimator_node-1] est=[12.34 0.56 -0.12] drift=0.423m att_err=1.35deg rmse=0.112 matches=642
```

誤差だけを別ターミナルで見る場合:

```bash
ros2 topic echo /ksp_position_estimator/error
ros2 topic echo /ksp_position_estimator/status
```

```bash
ros2 topic echo /ksp_position_estimator/odom
```

## 誤差の見方

- Odometryは起動時の位置・姿勢を原点`(0,0,0)`として積算します。一方Ground Truthは`ground_truth_enu`原点の絶対座標なので、そのまま引き算しても意味がありません。このため誤差は**開始時点合わせ**で測ります: 初回採点時の推定値と真値をそれぞれの原点として記録し、そこからの変位ベクトルの差を`position_error` [m]、相対姿勢の差を`attitude_error_deg` [deg]として出します。
- `error` JSONには`estimated_position`、`estimated_displacement`、`truth_displacement`、`position_error_vector`、`position_error`、`attitude_error_deg`に加え、そのスキャンの`rmse`と`matches`が入ります。`status` JSONにも`position_error`と`attitude_error_deg`が入ります（真値未受信時は`null`）。
- Ground Truthは採点にしか使いません。ICP更新への入力は点群だけなので、この誤差はLiDARのみ推定の純粋な性能評価になります。
- Kerbin周回中に別の物体と編隊飛行し、その物体が視野を支配すると、推定はその物体に対する相対運動を追います。その場合、真値（慣性空間での自船運動）との差は編隊相対のドリフトとして現れます。これは故障ではなくscan-to-scan方式の性質です。地形などの静止物が十分に入る状況で評価してください。

## 設定

既定値は[`config/position_estimator.yaml`](config/position_estimator.yaml)にあります。コピーして変更し、次のように読み込めます。

```bash
ros2 launch position_estimator position_estimator.launch.py \
  config_file:=/absolute/path/to/my_position_estimator.yaml
```

よく調整する値:

| パラメータ | 内容 | 既定値 |
|---|---|---:|
| `lidar_sensor_id` | KSP上の3D LiDARのSensor ID | `front_lidar` |
| `vessel_topic_prefix` | ROS2 for KSPの機体系Topicルート | `/ksp_vessel` |
| `lidar_topic` | 空以外で自動解決より優先する点群Topic | `""` |
| `odom_topic` | 推定Odometryの出力先 | `/ksp_position_estimator/odom` |
| `error_topic` | 真値対比の誤差JSONの出力先 | `/ksp_position_estimator/error` |
| `pose_topic` | 空以外で自動解決より優先する真値pose Topic（評価のみ） | `""` |
| `evaluate` | 真値採点とコンソール誤差表示の有効化 | `true` |
| `ground_truth_timeout_sec` | 点群時刻と真値の許容ずれ [s] | `0.5` |
| `console_echo_period_sec` | コンソール表示の周期 [s]。`0`以下で毎スキャン | `2.0` |
| `odom_frame` / `base_frame` | 推定TFの親/子フレーム | `lidar_odom_3d` / `estimator_base_link` |
| `voxel_size` | ダウンサンプルの voxel 一辺 [m] | `0.4` |
| `max_points` | ICPに使う最大点数 | `800` |
| `min_range` / `max_range` | 使用する距離範囲 [m] | `1.0` / `120.0` |
| `max_correspondence_distance` | 対応付けの最大距離 [m] | `1.5` |
| `trim_fraction` | 距離順に残す対応の割合 | `0.8` |
| `max_rmse` | これを超えるRMSEは棄却 [m] | `0.6` |
| `max_translation_per_scan` | 1スキャン間の最大移動量 [m] | `3.0` |
| `max_rotation_per_scan_deg` | 1スキャン間の最大回転量 [deg] | `30.0` |

## 入出力

| 方向 | Topic | 型 |
|---|---|---|
| Subscribe | `/ksp_vessel/lidar_3d/<lidar_sensor_id>/points` | `sensor_msgs/msg/PointCloud2` |
| Subscribe（評価のみ） | `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` |
| Publish | `/ksp_position_estimator/odom` | `nav_msgs/msg/Odometry` |
| Publish | `/ksp_position_estimator/status` | `std_msgs/msg/String` (JSON) |
| Publish | `/ksp_position_estimator/error` | `std_msgs/msg/String` (JSON) |
| Publish | TF `lidar_odom_3d -> estimator_base_link` | `tf2_msgs/msg/TFMessage` |

`status` JSONには`sequence`、`points`、`rmse`、`matches`、`variance`、`position`、`quaternion`、`position_error`、`attitude_error_deg`が入ります。センサーIDはbridgeと同じ規則で小文字のROS名へ正規化します。

## 制約

- Ground Truthは採点にのみ使い、推定には使いません。制御指令も出しません。
- scan-to-scanの相対推定を積算するため、長時間ではドリフトします。
- 視野を支配する物体と一緒に動くと、その物体に対する相対運動を推定します（上記の「誤差の見方」を参照）。
- 点群の`x/y/z`フィールドだけを使います。強度やリング情報は使いません。
- 2秒を超える点群の途切れでは速度をゼロ扱いにし、次のスキャンから再初期化します。
