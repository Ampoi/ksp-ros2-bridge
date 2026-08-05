# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDAR名ごとにROS2の標準センサTopicへpublishする`ament_python`パッケージです。

- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- Topic: `/ksp_ros2/lidar/<name>`

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
ros2 run ksp_lidar_bridge udp_bridge --host 0.0.0.0 --port 49010
```

停止通知が欠落した場合のTopic削除時間は`--topic-timeout-sec`で変更できます（デフォルト3秒）。LiDARの最低スキャン周期より長い正の値を指定してください。

確認例:

```bash
ros2 topic list | grep /ksp_ros2/lidar
ros2 topic echo /ksp_ros2/lidar/front_lidar
```

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
