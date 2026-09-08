# ROS2アプリケーションを作る

[Getting Started](getting-started.md)で機体情報と点群を受信できたら、自分のROS2ノードをPyLoNのTopicへ接続します。センサー処理にはROS2の標準メッセージ、機体・パーツ制御には`pylon_interfaces`を使います。

## 1. 利用する入出力を選ぶ

[Topic一覧](../api/topics.md)から必要なデータを選び、パーツ別APIでメッセージ型、QoS、座標系、更新周期を確認します。

| 作りたい機能 | 主な入力 | 出力・接続先 |
|---|---|---|
| 点群処理・地図作成 | LiDARの`LaserScan` / `PointCloud2`、TF | 自分の推定・地図Topic |
| 画像処理 | カメラの`Image` / `CameraInfo` | 自分の検出結果Topic |
| 姿勢推定 | IMU、スタートラッカー | 自分の推定姿勢Topic |
| 機体の誘導・姿勢制御 | 機体のPose / Twist、lifecycle | `ControlSetpoint`またはlease付き`BodyWrenchCommand` |
| 車輪や関節の制御 | `WheelState` / `MotorState` | lease付き`WheelCommand` / `MotorCommand` |

bridgeと同じROS2 Jazzy環境を使い、ノードを起動するターミナルで環境を読み込みます。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 2. センサーTopicを購読する

KSPでセンサーを載せた機体をFlightへ出し、実際のTopic名を確認します。以下の`front_lidar`は、VAB/SPHで設定したSensor IDへ置き換えてください。

```bash
ros2 topic list
ros2 topic info --verbose /ksp_vessel/lidar_3d/front_lidar/points
ros2 topic echo --once /ksp_vessel/lidar_3d/front_lidar/points
ros2 interface show sensor_msgs/msg/PointCloud2
```

自分のノードでは、確認した型とpublisherに対応するQoSでsubscriptionを作ります。センサーTopicは最初のデータ受信時に作られ、Flight終了や受信タイムアウトで消えるため、ノード側でもデータの最終受信時刻を管理してください。

位置や方向を別の座標系へ変換するときは、メッセージの`header.frame_id`と`header.stamp`を使ってTFを参照します。[機体モデルとTF](../api/vessel-model.md)で、センサーフレームと機体の接続条件を確認できます。

## 3. 機体・パーツを制御する

機体の目標姿勢や位置を与える場合は、lease更新とsequence採番を行う`pylon_vehicle_control`を利用できます。

```bash
ros2 run pylon_vehicle_control setpoint_controller --ros-args \
  -p controller_id:=my_controller \
  -p setpoint_topic:=/my_controller/setpoint
```

自分の誘導ノードから`/my_controller/setpoint`へ`pylon_interfaces/msg/ControlSetpoint`をpublishします。フィールドとmode定数は次で確認できます。既定の制御器はGround TruthのPose / Twistを参照します。

```bash
ros2 interface show pylon_interfaces/msg/ControlSetpoint
```

力・トルクや個別パーツを直接指令するノードは、[機体制御API](../api/vehicle-control.md)に従って制御権を取得します。

1. lifecycleから操作機体の`vessel_id`を取得します。
2. leaseを取得し、authority stateで所有権の確定を確認します。
3. 指令へ`vessel_id`、`controller_id`、`lease_id`を設定し、同じlease内の全指令を通じて`sequence`を増やします。
4. timeoutより短い周期で指令を送り、leaseも更新します。終了時にはleaseを解放します。

各パーツのstateとWrench feedbackで実現量を確認してください。機体切替、制御権喪失、入力欠測時には目標を破棄し、新しい状態に基づいて制御を開始します。共通制御器の再開には、新たな`MODE_IDLE`と後続のsetpointが必要です。

## 4. サンプルを参照する

[デモ一覧](../demos/index.md)から、機体準備と起動手順を確認できます。

- [軌道上のデブリ周回・撮影](../demos/debris-orbit.md)：相対運動の推定、RCS制御、画像保存
- [2D LiDARとSLAM](../demos/lidar-slam.md)：地図作成・保存・Nav2走行
- [月面Nav2](../demos/mun-nav2.md)：点群・IMU・車輪を使う自律走行

通信先やTopic名を変える場合は[Bridge起動オプション](../reference/bridge-options.md)、接続できない場合は[トラブルシュート](../reference/troubleshooting.md)を参照してください。
