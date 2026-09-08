# pylon_demo_lidar_slam

2D LiDARのスキャンから平面odometryを求め、SLAM Toolboxで地図を作成するデモです。Nav2の速度指令をPyLoNのlease付きBody Wrenchへ変換し、保存地図を使うAMCLナビゲーションにも対応します。

導入、機体準備、地図作成、保存、Nav2走行、停止の手順は[2D LiDARとSLAM](../../docs/demos/lidar-slam.md)にまとめています。

```bash
./sync.sh --demo lidar_slam
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_lidar_slam mapping.launch.py \
  scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan
```

bridgeは別ターミナルで起動します。機体切替や通信断の後は、ゴールを取り消し、launch全体を再起動してください。

## 主な入出力

| Topic | 型 | 用途 |
|---|---|---|
| `/ksp_vessel/lidar_2d/<id>/scan` | `sensor_msgs/msg/LaserScan` | センサー入力 |
| `/pylon/lidar_slam/scan` | `sensor_msgs/msg/LaserScan` | 推定フレームでのスキャン |
| `/pylon/lidar_slam/odom` | `nav_msgs/msg/Odometry` | 平面の自己位置・速度 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM地図 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2の速度指令 |
| `/ksp_vessel/control/authority/{command,state}` | `pylon_interfaces/msg/ControlAuthorityCommand` / `ControlAuthorityState` | 制御権 |
| `/ksp_vessel/control/wrench_command` | `pylon_interfaces/msg/BodyWrenchCommand` | 力・トルク指令 |

TFは`map → pylon_slam_odom → pylon_slam_base_link`です。ベースフレームはLiDARの原点・向きに一致します。

## PyLoN本体への貢献

リポジトリのルートでテストを実行します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH=Demo/pylon_demo_lidar_slam:Ros2/pylon_vehicle_control \
  python3 -m unittest discover -s Demo/pylon_demo_lidar_slam/test -v
```

数値処理・セッション管理・設定のテストはNumPyとPyYAMLがあれば実行できます。ROSアダプターのテストはROS2とビルド済み`pylon_interfaces`を必要とし、独立したDDS domainで実行します。

実装変更の手順は[本体開発ガイド](../../docs/contributing/index.md)を参照してください。
