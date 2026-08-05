# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDARとロボティクスモーターをROS2標準メッセージへ変換する`ament_python`パッケージです。

- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- Topic: `/ksp_ros2/lidar/<name>`
- Motor command: `/ksp_ros2/motors/command` (`trajectory_msgs/msg/JointTrajectory`)
- Motor state: `/joint_states` (`sensor_msgs/msg/JointState`)
- Motor diagnostics/current estimate: `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

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

KSP以外のホストへ指令を返す場合:

```bash
ros2 run ksp_lidar_bridge udp_bridge --command-host 192.168.1.50 --command-port 49011
```

確認例:

```bash
ros2 topic list | grep /ksp_ros2/lidar
ros2 topic echo /ksp_ros2/lidar/front_lidar
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

サーボを90度へ動かす例（`servo_12345`は`/joint_states.name`で確認）:

```bash
ros2 topic pub --once /ksp_ros2/motors/command trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [servo_12345], points: [{positions: [1.5708], velocities: [0.5], effort: [100.0]}]}"
```

単位はROS規約に合わせ、回転位置/速度/effortがrad・rad/s・N·m、直動位置/速度/effortがm・m/s・Nです。`JointTrajectoryPoint.time_from_start`付きの複数pointにも対応します。新しいtrajectoryは未送信の古いtrajectoryを置き換えます。

`/diagnostics`の`estimated_current_a`はKSP側の推定モーター出力とパーツ設定のトルク定数/推力定数から計算した値です。KSPには実電流センサーがないため、実測値ではありません。

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
