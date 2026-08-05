# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDAR、ロボティクスモーター、Flight中のactive vesselランタイムプロキシをROS2へpublishする`ament_python`パッケージです。

- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- 2D Topic: `/ros2_ksp/<part_name>/lidar/scan`
- 3D Topic: `/ros2_ksp/<part_name>/lidar/points`
- Active vessel URDF: `/ros2_ksp/active_vessel/robot_description`
- Active vessel root frame: `/ros2_ksp/active_vessel/root_frame`
- Active vessel fixed-joint transforms: `/tf`
- Motor command: `/ros2_ksp/motors/command` (`trajectory_msgs/msg/JointTrajectory`)
- Motor state: `/joint_states` (`sensor_msgs/msg/JointState`)
- Motor diagnostics/current estimate: `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

TopicはKSPのFlight中にスキャンを受信したときだけ作成されます。Flightを終了するとKSPからの停止通知で削除され、通知を受け取れなかった場合もスキャン停止から3秒後に自動削除されます。再びFlightへ入ると、最初のスキャン受信時に自動で再作成されます。

3D点群は、現在のKSP modが送信するフラット配列（`[x,y,z,...]`）と、旧形式のネスト配列（`[[x,y,z],...]`）の両方を受信できます。

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select ksp_lidar_bridge
source install/setup.bash
```

## Run

```bash
ROS_LOCALHOST_ONLY=1 ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

停止通知が欠落した場合のTopic削除時間は`--topic-timeout-sec`で変更できます（デフォルト3秒）。LiDARの最低スキャン周期より長い正の値を指定してください。

KSP以外のホストへ指令を返す場合:

```bash
ros2 run ksp_lidar_bridge udp_bridge --command-host 192.168.1.50 --command-port 49011
```

確認例:

```bash
ros2 topic list | grep /ros2_ksp
ros2 topic echo /ros2_ksp/front_lidar/lidar/scan
ros2 topic echo --once /ros2_ksp/active_vessel/root_frame
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

`<part_name>`はVAB/SPHのパーツ右クリックメニューにある`Edit ROS2 Part Name`で設定します。機体に搭載されたすべてのKerbalLiDARパーツについて、名前別のpublisherが動的に作られます。Topicルートを変更する場合は`--topic-prefix`を指定してください。

## Active vessel runtime proxy

URDFは`std_msgs/msg/String`をtransient-local QoSでpublishします。bridgeはURDFの固定ジョイントを`/tf`へ既定5Hzでpublishするため、RViz2のRobotModelで直接表示できます。操作機体の構成・相対姿勢が変わるとモデルは更新され、KSPからの再送が途絶えると自動的に期限切れになります。LiDARパケットの`partFlightId`がモデル内にある場合、センサの`frame_id`も対応linkへ揃います。

主なオプション:

- `--robot-description-topic`: URDF Topic名
- `--root-frame-topic`: RViz Fixed Frame確認用Topic名
- `--model-tf-rate`: 固定ジョイントTFの更新Hz
- `--allow-remote-models`: 非loopback送信元のモデルパケットを許可
- `--allow-network-model-topics`: `ROS_LOCALHOST_ONLY=1`なしでモデルpublishを許可

モデル経路は既定でfail-closedです。KSP側の`udpHost`はloopback、`allowRemoteUrdf`は`false`にし、bridgeは`ROS_LOCALHOST_ONLY=1`で起動してください。明示的な許可オプションは、SROS2/DDS Securityなどで保護したネットワークでのみ使用してください。

ランタイムプロキシはKSPのmesh、texture、part名、asset pathを一切含まず、匿名linkと量子化されたbox形状だけで構成されます。受信時はgzip展開量、チャンク数、SHA-256、XMLタグ・属性を検証し、`mesh`や外部参照を拒否します。bridge自身はURDFをディスクへ保存しません。ただし、ROS2 Topicを購読できる同一ホストのプロセスによるコピーを技術的に完全禁止するものではありません。

## Motor control

サーボを90度へ動かす例（`servo_12345`は`/joint_states.name`で確認）:

```bash
ros2 topic pub --once /ros2_ksp/motors/command trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [servo_12345], points: [{positions: [1.5708], velocities: [0.5], effort: [100.0]}]}"
```

単位はROS規約に合わせ、回転位置/速度/effortがrad・rad/s・N·m、直動位置/速度/effortがm・m/s・Nです。`JointTrajectoryPoint.time_from_start`付きの複数pointにも対応します。新しいtrajectoryは未送信の古いtrajectoryを置き換えます。

`/diagnostics`の`estimated_current_a`はKSP側の推定モーター出力とパーツ設定のトルク定数/推力定数から計算した値です。KSPには実電流センサーがないため、実測値ではありません。

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
