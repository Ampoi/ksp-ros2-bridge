# Topic一覧

Topicは、操作中の機体に属するものを`/ksp_vessel`、bridgeプロセス自身に属するものを`/ros2_ksp`へ分けています。表の方向はROS2ノードから見た方向です。

## Bridge

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Publish | `/ros2_ksp/status` | `std_msgs/msg/String` | Reliable / Transient Local。bridgeの稼働状態 |
| Publish | `/ros2_ksp/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | モーター診断とWrench配分残差 |

## 機体・モデル

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Publish | `/ksp_vessel/lifecycle` | `VesselLifecycle` | Reliable / Transient Local。実機体IDとframe lifecycle |
| Subscribe | `/ksp_vessel/control/authority/command` | `ControlAuthorityCommand` | Reliable。lease・SAS排他・e-stop |
| Publish | `/ksp_vessel/control/authority/state` | `ControlAuthorityState` | Reliable / Transient Local。確定したowner |
| Subscribe | `/ksp_vessel/control/wrench_command` | `BodyWrenchCommand` | Reliable。lease-bound `base_link` Wrench |
| Publish | `/ksp_vessel/control/wrench_feedback` | `WrenchFeedback` | requested / allocated / achieved / residual |
| Publish | `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | Best Effort。ENU位置・姿勢 |
| Publish | `/ksp_vessel/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | Best Effort。ENU速度 |
| Publish | `/ksp_vessel/ground_truth/twist_body` | `geometry_msgs/msg/TwistStamped` | Best Effort。body速度 |
| Publish | `/ksp_vessel/ground_truth/acceleration` | `geometry_msgs/msg/AccelStamped` | Best Effort。ENU加速度 |
| Publish | `/ksp_vessel/joint_states` | `sensor_msgs/msg/JointState` | 全ROSサーボの状態 |
| Publish | `/ksp_vessel/robot_description` | `std_msgs/msg/String` | Reliable / Transient Local。プロキシURDF |
| Publish | `/ksp_vessel/root_frame` | `std_msgs/msg/String` | Reliable / Transient Local。RViz Fixed Frame名 |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | Ground TruthとCoM基準proxy rootのdynamic TF |
| Publish | `/tf_static` | `tf2_msgs/msg/TFMessage` | proxy固定jointとsensor mount |

## センサー

| 方向 | Topic | 型 |
|---|---|---|
| Publish | `/ksp_vessel/lidar_2d/<lidar_2d_id>/scan` | `sensor_msgs/msg/LaserScan` |
| Publish | `/ksp_vessel/lidar_3d/<lidar_3d_id>/points` | `sensor_msgs/msg/PointCloud2` |
| Publish | `/ksp_vessel/camera/<camera_id>/image_raw` | `sensor_msgs/msg/Image` |
| Publish | `/ksp_vessel/camera/<camera_id>/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Publish | `/ksp_vessel/docking_ports/<id>/camera/image_raw` | `sensor_msgs/msg/Image` |
| Publish | `/ksp_vessel/docking_ports/<id>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

LiDARとRGBカメラのIDはVAB/SPHの`Edit ROS2 Sensor ID`で設定します。新規パーツには`lidar_2d_...`、`lidar_3d_...`、`camera_...`のIDが自動生成されます。センサーTopicは最初の完全なデータで作成され、inactive通知または`--topic-timeout-sec`の無通信で削除されます。

## 型付きアクチュエータ

個体ごとにTopicを作らず、種類ごとに1組の`command` / `state`を共有します。すべてのcommandは`id`フィールドで対象を指定し、stateにも送信元の`id`が入ります。stateの`name`は既存コード向けの同値エイリアスです。

| 対象 | Command Topic | State Topic | 型 |
|---|---|---|---|
| ROSサーボ／リニアモーター | `/ksp_vessel/actuators/servo/command` | `/ksp_vessel/actuators/servo/state` | `MotorCommand` / `MotorState` |
| KSP標準ホイール | `/ksp_vessel/actuators/wheel/command` | `/ksp_vessel/actuators/wheel/state` | `WheelCommand` / `WheelState` |
| Engine | `/ksp_vessel/actuators/propulsion/command` | `/ksp_vessel/actuators/propulsion/state` | `EngineCommand` / `EngineState` |
| RCSモジュール | `/ksp_vessel/actuators/rcs/command` | `/ksp_vessel/actuators/rcs/state` | `RcsCommand` / `RcsState` |
| デカプラー／フェアリング | `/ksp_vessel/actuators/separation/command` | `/ksp_vessel/actuators/separation/state` | `SeparationCommand` / `SeparationState` |

正式commandはauthority leaseと`vessel_id / controller_id / lease_id / sequence`が必要です。commandはReliable / Volatile / depth 10です。通常のstateはBest Effort / Volatile / depth 10、分離stateはReliable / Transient Local / depth 10です。

### 集約操作（legacy互換、既定無効）

次の所有権を持たない入力は、bridgeへ`--enable-legacy-control`を付けた場合だけ購読されます。新規実装ではlease付きの型付きcommandを使用してください。

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| Subscribe | `/ksp_vessel/actuators/servo/trajectory` | `trajectory_msgs/msg/JointTrajectory` | 複数サーボの軌道指令 |
| Subscribe | `/ksp_vessel/actuators/propulsion/main_throttle` | `std_msgs/msg/Float64` | メインスロットル |
| Subscribe | `/ksp_vessel/actuators/rcs/twist_command` | `geometry_msgs/msg/Twist` | RCS 6軸指令 |
| Subscribe | `/ksp_vessel/actuators/propulsion/json_command` | `std_msgs/msg/String` | 旧JSON個別指令 |
| Publish | `/ksp_vessel/actuators/propulsion/json_state` | `std_msgs/msg/String` | 旧JSON状態 |

## ドッキングポート

| 方向 | Topic | 型 |
|---|---|---|
| Publish | `/ksp_vessel/docking_ports/<id>/state` | `ksp_ros2_interfaces/msg/DockingPortState` |
| Subscribe | `/ksp_vessel/docking_ports/<id>/command` | `ksp_ros2_interfaces/msg/DockingPortCommand` |

`<id>`は`docking_port_<persistentId>_<moduleIndex>`です。commandは`SELECT_CAMERA`、`STOP_CAMERA`、`RELEASE`を提供します。

## 主な起動引数

| 対象 | 引数 | 既定値 |
|---|---|---|
| 機体センサー | `--topic-prefix` | `/ksp_vessel` |
| bridge状態 | `--bridge-prefix` | `/ros2_ksp` |
| 診断 | `--diagnostics-topic` | `/ros2_ksp/diagnostics` |
| Wrench command | `--body-wrench-command-topic` | `/ksp_vessel/control/wrench_command` |
| Authority command | `--control-authority-command-topic` | `/ksp_vessel/control/authority/command` |
| Authority state | `--control-authority-state-topic` | `/ksp_vessel/control/authority/state` |
| Wrench feedback | `--wrench-feedback-topic` | `/ksp_vessel/control/wrench_feedback` |
| Vessel lifecycle | `--vessel-lifecycle-topic` | `/ksp_vessel/lifecycle` |
| 旧Body Wrench | `--body-wrench-topic` | 空（無効） |
| legacy集約制御 | `--enable-legacy-control` | false（無効） |
| Ground Truth | `--ground-truth-prefix` | `/ksp_vessel/ground_truth` |
| アクチュエータ | `--actuators-prefix` | `/ksp_vessel/actuators` |
| ドッキングポート | `--docking-ports-prefix` | `/ksp_vessel/docking_ports` |
| joint state | `--joint-states-topic` | `/ksp_vessel/joint_states` |
| URDF | `--robot-description-topic` | `/ksp_vessel/robot_description` |
| root frame | `--root-frame-topic` | `/ksp_vessel/root_frame` |

全オプションは`ros2 run ksp_lidar_bridge udp_bridge --help`で確認できます。
