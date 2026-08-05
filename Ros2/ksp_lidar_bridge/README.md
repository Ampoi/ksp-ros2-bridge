# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDARとFlight中のactive vesselランタイムプロキシをROS2へpublishする`ament_python`パッケージです。

- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- Topic: `/ksp_ros2/lidar/<name>`
- Active vessel URDF: `/ksp_ros2/active_vessel/robot_description`
- Active vessel root frame: `/ksp_ros2/active_vessel/root_frame`
- Active vessel fixed-joint transforms: `/tf`

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

確認例:

```bash
ros2 topic list | grep /ksp_ros2/lidar
ros2 topic echo /ksp_ros2/lidar/front_lidar
ros2 topic echo --once /ksp_ros2/active_vessel/root_frame
```

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

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
