# Topic一覧

Topicは、操作中の機体に属するものを`/ksp_vessel`、bridgeプロセス自身に属するものを`/pylon`へ分けています。表の方向はROS2ノードから見た方向です。

## Bridge

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Publish | `/pylon/status` | `std_msgs/msg/String` | Reliable / Transient Local。bridgeの稼働状態 |
| Publish | `/pylon/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | モーター診断とWrench配分残差 |

## IMU

センサーtimestampの間隔はKSPの物理時間を保持します。通信遅延やゲームの描画速度に合わせて飛行中にoffsetを飛ばしません。lifecycleはセンサーと独立したセッションheartbeatから生成します。`udp_bridge --disable-ground-truth`では真値Topicとworld TFの配信を完全に無効化できます。このモードの`VesselLifecycle.world_frame`は空です。`runtime_instance`・`runtime_epoch`・`runtime_generation`がKSP側のセッションを表し、`generation`と`origin_sequence`は機体切替・時刻巻き戻り・セッションtimeoutからの復帰時に更新されるROS側の世代番号です。

全機体で追加パーツ・設定なしに、操作中の機体の3軸ジャイロと3軸加速度計を出力します。機体切替時も同じTopicを使用します。

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Publish | `/ksp_vessel/imu/data_raw` | `sensor_msgs/msg/Imu` | Best Effort / Volatile / depth 10。物理サンプル更新時、最大30 Hz |

`header.frame_id=base_link`、軸はX前方・Y左・Z上。`angular_velocity` は慣性系に対するrad/s（KSPの回転物理座標系では惑星の自転分を加算）、`linear_acceleration` はm/s²の比力（慣性加速度−重力）です。Z上向きで静止していれば約+g、自由落下中は約0となります。KSPの `perturbation_immediate` を機体軸へ変換し、ノイズやバイアスは追加しません。絶対姿勢は測定しないため `orientation_covariance[0]=-1`、角速度・加速度の共分散は未知を表す全要素0です。[ROS IMU規約（REP-145）](https://www.ros.org/reps/rep-0145.html) の `imu/data_raw` に対応します。

タイムスタンプはKSPのシミュレーション時刻を既存bridge時計に写像します。Flight以外、物理演算停止（packed）、ポーズ中は新規測定を出力しません。ロード・機体切替・unpack直後は1サンプル待ちます。仮想センサーは機体重心の比力を機体軸で表すもので、個別パーツ位置の回転による加速度はモデル化しません。非アクティブ機体の個別Topicは作成しません。

## スタートラッカー

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| Publish | `/ksp_vessel/star_tracker/<id>/state` | `pylon_interfaces/msg/StarTrackerState` | 姿勢・共分散・valid・測定不能理由を同時配信 |
| Publish | `/ksp_vessel/star_tracker/<id>/attitude` | `geometry_msgs/msg/QuaternionStamped` | 有効な慣性姿勢のみ |

専用パーツを取り付けると既定5 Hz、Reliable / Volatile / depth 10で配信します。位置は測定しません。`state.valid=false`時はquaternionが全0、共分散先頭が−1です。通信断でも`stale`状態を通知します。座標・制約・再捕捉・Topicの寿命は[スタートラッカー](../parts/star-tracker.md)を参照してください。

## 機体・モデル

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Publish | `/ksp_vessel/lifecycle` | `VesselLifecycle` | Reliable / Transient Local。実機体IDとframe lifecycle |
| Subscribe | `/ksp_vessel/control/authority/command` | `ControlAuthorityCommand` | Reliable。lease・SAS排他・e-stop |
| Publish | `/ksp_vessel/control/authority/state` | `ControlAuthorityState` | Reliable / Transient Local。確定したowner |
| Subscribe | `/ksp_vessel/control/wrench_command` | `BodyWrenchCommand` | Reliable。lease-bound `base_link` Wrench |
| Publish | `/ksp_vessel/control/wrench_feedback` | `WrenchFeedback` | requested / allocated / achieved / residual |
| Publish | `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | Best Effort。ENU位置・姿勢 |
| Publish | `/ksp_vessel/ground_truth/nearby_vessels` | `pylon_interfaces/msg/NearbyVessels` | 自機と近隣機体の同時刻・同原点の絶対位置と速度。最大32機、2500 m以内、同天体・loaded/unpacked |
| Publish | `/ksp_vessel/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | Best Effort。ENU速度 |
| Publish | `/ksp_vessel/ground_truth/twist_body` | `geometry_msgs/msg/TwistStamped` | Best Effort。body速度 |
| Publish | `/ksp_vessel/ground_truth/frame_angular_velocity` | `geometry_msgs/msg/Vector3Stamped` | 惑星固定ENU軸の慣性系に対する回転角速度。評価器が慣性推定と比較するための情報 |
| Publish | `/ksp_vessel/ground_truth/acceleration` | `geometry_msgs/msg/AccelStamped` | Best Effort。ENU加速度 |
| Publish | `/ksp_vessel/joint_states` | `sensor_msgs/msg/JointState` | 全ROSサーボの状態 |
| Publish | `/ksp_vessel/robot_description` | `std_msgs/msg/String` | Reliable / Transient Local。プロキシURDF |
| Publish | `/ksp_vessel/root_frame` | `std_msgs/msg/String` | Reliable / Transient Local。RViz Fixed Frame名 |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | Ground TruthとCoM基準proxy rootのdynamic TF |
| Publish | `/tf_static` | `tf2_msgs/msg/TFMessage` | proxy固定jointとsensor mount |

Ground TruthのENU軸は惑星固定です。真値の角速度もこの軸に対する値に揃え、KSP物理座標系の高度による切替に影響されません。IMUの慣性姿勢と比較する場合は`frame_angular_velocity`による座標回転と速度の輸送項を評価側で加えます。デモの推定器・制御器はこのTopicを購読しません。

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

## ドッキングポート

| 方向 | Topic | 型 |
|---|---|---|
| Publish | `/ksp_vessel/docking_ports/<id>/state` | `pylon_interfaces/msg/DockingPortState` |
| Subscribe | `/ksp_vessel/docking_ports/<id>/command` | `pylon_interfaces/msg/DockingPortCommand` |

`<id>`の既定値は`docking_port_<persistentId>_<moduleIndex>`です。commandは`SELECT_CAMERA`、`STOP_CAMERA`、`RELEASE`を提供します。

## 主な起動引数

| 対象 | 引数 | 既定値 |
|---|---|---|
| 機体センサー | `--topic-prefix` | `/ksp_vessel` |
| bridge状態 | `--bridge-prefix` | `/pylon` |
| 診断 | `--diagnostics-topic` | `/pylon/diagnostics` |
| Wrench command | `--body-wrench-command-topic` | `/ksp_vessel/control/wrench_command` |
| Authority command | `--control-authority-command-topic` | `/ksp_vessel/control/authority/command` |
| Authority state | `--control-authority-state-topic` | `/ksp_vessel/control/authority/state` |
| Wrench feedback | `--wrench-feedback-topic` | `/ksp_vessel/control/wrench_feedback` |
| Vessel lifecycle | `--vessel-lifecycle-topic` | `/ksp_vessel/lifecycle` |
| Ground Truth | `--ground-truth-prefix` | `/ksp_vessel/ground_truth` |
| アクチュエータ | `--actuators-prefix` | `/ksp_vessel/actuators` |
| ドッキングポート | `--docking-ports-prefix` | `/ksp_vessel/docking_ports` |
| joint state | `--joint-states-topic` | `/ksp_vessel/joint_states` |
| URDF | `--robot-description-topic` | `/ksp_vessel/robot_description` |
| root frame | `--root-frame-topic` | `/ksp_vessel/root_frame` |

全オプションは`ros2 run pylon_bridge udp_bridge --help`で確認できます。
