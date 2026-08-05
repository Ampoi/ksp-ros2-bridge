# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDAR名ごとにROS2の標準センサTopicへpublishする`ament_python`パッケージです。

- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- 2D Topic: `/ros2_ksp/<part_name>/lidar/scan`
- 3D Topic: `/ros2_ksp/<part_name>/lidar/points`

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
ros2 run ksp_lidar_bridge udp_bridge --host 0.0.0.0 --port 49010
```

確認例:

```bash
ros2 topic list | grep /ros2_ksp
ros2 topic echo /ros2_ksp/front_lidar/lidar/scan
```

`<part_name>`はVAB/SPHのパーツ右クリックメニューにある`Edit ROS2 Part Name`で設定します。機体に搭載されたすべてのKerbalLiDARパーツについて、名前別のpublisherが動的に作られます。Topicルートを変更する場合は`--topic-prefix`を指定してください。

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
