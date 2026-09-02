# Topic一覧

表の方向はROS2ノードから見た方向です。`Publish`はbridgeがpublishし、`Subscribe`はbridgeがsubscribeします。

## センサー・モデル

| 方向 | Topic | 型 | 生成 | QoS / 主な内容 |
|---|---|---|---|---|
| Publish | `/ros2_ksp/bridge/status` | `std_msgs/msg/String` | 常設 | Reliable / Transient Local / depth 1。`listening` |
| Publish | `/ros2_ksp/<sensor_id>/lidar/scan` | `sensor_msgs/msg/LaserScan` | 動的 | Reliable / Volatile / depth 10。2D距離 |
| Publish | `/ros2_ksp/<sensor_id>/lidar/points` | `sensor_msgs/msg/PointCloud2` | 動的 | Reliable / Volatile / depth 10。3D点群 |
| Publish | `/ros2_ksp/<sensor_id>/camera/image_raw` | `sensor_msgs/msg/Image` | 動的 | Reliable / Volatile / depth 10。`rgb8`画像 |
| Publish | `/ros2_ksp/<sensor_id>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 動的 | Reliable / Volatile / depth 10。pinhole内部パラメーター |
| Publish | `/ros2_ksp/docking_ports/<name>/camera/image_raw` | `sensor_msgs/msg/Image` | 動的 | Reliable / Volatile / depth 10。選択中ポートの`rgb8`画像 |
| Publish | `/ros2_ksp/docking_ports/<name>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 動的 | Reliable / Volatile / depth 10。ポートカメラ内部パラメーター |
| Publish | `/ros2_ksp/active_vessel/robot_description` | `std_msgs/msg/String` | 常設 | Reliable / Transient Local / depth 1。プロキシURDF |
| Publish | `/ros2_ksp/active_vessel/root_frame` | `std_msgs/msg/String` | 常設 | Reliable / Transient Local / depth 1。RViz Fixed Frame名 |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | 常設 | 機体、センサー、Ground TruthのTF |

センサーTopicは最初の完全なデータを受信すると作成され、inactive通知または`--topic-timeout-sec`の無通信で削除されます。`<sensor_id>`はVAB/SPHで設定するセンサーIDです。

## ドッキングポート

| 方向 | Topic | 型 | QoS |
|---|---|---|---|
| Publish | `/ros2_ksp/docking_ports/<name>/state` | `ksp_ros2_interfaces/msg/DockingPortState` | Best Effort / Volatile / depth 10 |
| Subscribe | `/ros2_ksp/docking_ports/<name>/command` | `ksp_ros2_interfaces/msg/DockingPortCommand` | Reliable / Volatile / depth 10 |

`<name>`は`docking_port_<persistentId>_<moduleIndex>`です。commandは`SELECT_CAMERA`、`STOP_CAMERA`、`RELEASE`を提供します。詳細は[ドッキングポート](/parts/docking)を参照してください。

## モーター・従来推進API

| 方向 | Topic | 型 | 生成 | 主な内容 |
|---|---|---|---|---|
| Subscribe | `/ros2_ksp/motors/command` | `trajectory_msgs/msg/JointTrajectory` | 常設 | 回転・直動モーター指令 |
| Publish | `/joint_states` | `sensor_msgs/msg/JointState` | 常設 | 全ROSモーターの状態 |
| Publish | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 常設 | モーター診断とWrench配分残差 |
| Publish | `/ros2_ksp/propulsion/state` | `std_msgs/msg/String` | 常設 | Engine / RCS状態JSON |
| Subscribe | `/ros2_ksp/propulsion/command` | `std_msgs/msg/String` | 常設 | Engine / RCS個別指令JSON |
| Subscribe | `/ros2_ksp/propulsion/main_throttle` | `std_msgs/msg/Float64` | 常設 | メインスロットル`0.0..1.0` |
| Subscribe | `/ros2_ksp/propulsion/rcs_command` | `geometry_msgs/msg/Twist` | 常設 | RCS並進・姿勢6軸、各`-1.0..1.0` |

これらは既存クライアント向けの集約APIです。パーツ単位の制御には次の型付きTopicも利用できます。

## 機体制御・Ground Truth

| 方向 | Topic | 型 | 生成 | QoS / 主な内容 |
|---|---|---|---|---|
| Subscribe | `/body_wrench` | `geometry_msgs/msg/WrenchStamped` | 常設 | Reliable / depth 10。`base_link`基準のN / N·m要求 |
| Publish | `/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | 常設 | Best Effort / depth 10。ENU位置・姿勢 |
| Publish | `/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | 常設 | Best Effort / depth 10。ENU速度 |
| Publish | `/ground_truth/acceleration` | `geometry_msgs/msg/AccelStamped` | 常設 | Best Effort / depth 10。ENU加速度 |

Ground TruthはFlight中に30 Hzで更新され、3つのメッセージと`ground_truth_enu -> base_link` TFは同じbridge受信時刻を共有します。詳細は[機体制御とGround Truth](/api/vehicle-control)を参照してください。

## 型付きアクチュエータ

Flight中に検出したアクチュエータごとに、次の2 Topicを動的に作成します。

| 方向 | Topic | QoS |
|---|---|---|
| Subscribe | `/actuators/<name>/command` | Reliable / Volatile / depth 10 |
| Publish | `/actuators/<name>/state` | 通常はBest Effort / Volatile / depth 10。分離機構はReliable / Transient Local / depth 1 |

| 対象 | command型 | state型 | 名前の例 |
|---|---|---|---|
| KSP標準ホイール | `ksp_ros2_interfaces/msg/WheelCommand` | `ksp_ros2_interfaces/msg/WheelState` | `wheel_<persistentId>_<moduleIndex>` |
| Engine | `ksp_ros2_interfaces/msg/EngineCommand` | `ksp_ros2_interfaces/msg/EngineState` | `engine_<persistentId>_<moduleIndex>` |
| RCS | `ksp_ros2_interfaces/msg/RcsCommand` | `ksp_ros2_interfaces/msg/RcsState` | `rcs_<persistentId>_<moduleIndex>` |
| ROSモーター | `ksp_ros2_interfaces/msg/MotorCommand` | `ksp_ros2_interfaces/msg/MotorState` | `servo_<flightId>` / `linear_<flightId>` |
| デカプラー／フェアリング | `ksp_ros2_interfaces/msg/SeparationCommand` | `ksp_ros2_interfaces/msg/SeparationState` | `decoupler_<persistentId>_<moduleIndex>` / `fairing_<persistentId>_<moduleIndex>` |

通常のアクチュエータはmanifestから外れた場合、または状態が`--topic-timeout-sec`の間届かなかった場合にcommand subscriptionとstate publisherを削除します。切断済みの分離機構だけはReliable / Transient Local / depth 1で最終状態を保持し、active vesselが変わると削除します。全フィールドは[ホイール](/parts/wheels)、[モーター](/parts/motors)、[推進系](/parts/propulsion)、[デカプラー／フェアリング](/parts/separation)に掲載しています。

## Timestamp

ROSメッセージの`header.stamp`にはbridge受信時のROSクロックを使います。KSP側の`universalTime`はUDPパケットの検証や状態JSONには使われますが、標準ROSメッセージのstampへ直接変換しません。ImageとCameraInfo、Ground Truthの各メッセージとTFは、それぞれ同一受信内で同じstampを共有します。

## 名前を変更できる起動引数

| 対象 | 引数 | 既定値 |
|---|---|---|
| センサーTopicとbridge status | `--topic-prefix` | `/ros2_ksp` |
| モーター指令 | `--motor-command-topic` | `/ros2_ksp/motors/command` |
| モーター状態 | `--joint-states-topic` | `/joint_states` |
| 診断 | `--diagnostics-topic` | `/diagnostics` |
| 推進系状態 | `--propulsion-state-topic` | `/ros2_ksp/propulsion/state` |
| 推進系JSON指令 | `--propulsion-command-topic` | `/ros2_ksp/propulsion/command` |
| メインスロットル | `--main-throttle-topic` | `/ros2_ksp/propulsion/main_throttle` |
| RCS指令 | `--rcs-command-topic` | `/ros2_ksp/propulsion/rcs_command` |
| Body Wrench | `--body-wrench-topic` | `/body_wrench` |
| Ground Truth | `--ground-truth-prefix` | `/ground_truth` |
| 型付きアクチュエータ | `--actuators-prefix` | `/actuators` |
| ドッキングポート | `--docking-ports-prefix` | `<topic-prefix>/docking_ports` |
| URDF | `--robot-description-topic` | `/ros2_ksp/active_vessel/robot_description` |
| root frame | `--root-frame-topic` | `/ros2_ksp/active_vessel/root_frame` |

::: warning prefixは独立しています
`--topic-prefix`を変更しても、モーター、推進系、機体制御、モデルの既定Topicは自動追従しません。すべてを別namespaceへ移す場合は専用引数も指定してください。
:::
