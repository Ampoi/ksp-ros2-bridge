# サーボ / リニアモーター

回転サーボとリニアモーターは共通のROS2 Topicを使います。`JointTrajectory.joint_names`に対象joint名を指定し、状態は全モーターをまとめた`JointState`で受け取ります。

## パーツ

| パーツ | joint type | 可動範囲 | 既定速度 | 定格effort |
|---|---|---:|---:|---:|
| ROS2 Size-0 Axial Servo | `revolute` | -π〜π rad | 45°/s | 250 N·m |
| ROS2 Telescoping I-Beam Actuator | `prismatic` | 0〜1.6 m | 0.25 m/s | 4000 N |

両方とも`bottom`側を親構造へ、動かすパーツ群を`top`側へ取り付けます。I-Beam Actuatorは両端にstack nodeがあり、縮長時はM-Beam 200 Pocket 1本分の約1.6 m、最大1.6 m伸長して全長約3.2 mになります。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 入力 | `/ksp_vessel/actuators/servo/trajectory` | `trajectory_msgs/msg/JointTrajectory` | legacy互換の複数サーボ指令（既定無効） |
| 出力 | `/ksp_vessel/joint_states` | `sensor_msgs/msg/JointState` | position、velocity、effort |
| 出力 | `/ros2_ksp/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 電源・engage・lock・推定電流 |
| 入力 | `/ksp_vessel/actuators/servo/command` | `ksp_ros2_interfaces/msg/MotorCommand` | `id`で指定する型付き指令 |
| 出力 | `/ksp_vessel/actuators/servo/state` | `ksp_ros2_interfaces/msg/MotorState` | `id`付きの型付き状態 |

既定ではKSPが状態を20 HzでUDP 49010へ送り、bridgeは受信するたびにROS2へpublishします。ROS2指令はbridgeからUDP 49011へ送られます。

## Joint名

`motorName`が空の場合はKSPの`partFlightId`から自動生成します。

```text
servo_<partFlightId>
linear_<partFlightId>
```

実際の名前は`/ksp_vessel/joint_states.name`またはKSPのPart Action Windowにある`ROS Joint`で確認してください。

型付きcommand TopicはReliable、state TopicはBest Effortで、どちらもdepth 10です。Topicは常設され、複数モーターのメッセージが同じTopicを流れます。

## JointTrajectory互換入力

`JointTrajectory`は所有権fieldを持てないためlegacy互換扱いで、既定ではbridgeが購読しません。移行時だけbridgeへ`--enable-legacy-control`を付けてください。新規コードでは後述のlease付き`MotorCommand`を使用します。

### 位置指令

サーボを90度へ、速度上限0.5 rad/s、effort上限100 N·mで動かします。

```bash
ros2 topic pub --once /ksp_vessel/actuators/servo/trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [servo_12345], points: [{positions: [1.5708], velocities: [0.5], effort: [100.0]}]}"
```

`positions`があるpointはposition modeです。`velocities`も指定した場合、サーボでは移動速度上限、リニアモーターでは`traverseVelocity`として使われます。`effort`は絶対値を上限として扱います。

### 速度指令とフェイルセーフ

`positions`を省略し、`velocities`を指定するとvelocity modeになります。

```bash
ros2 topic pub -r 10 /ksp_vessel/actuators/servo/trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [linear_12345], points: [{velocities: [0.1]}]}"
```

velocity modeはKSP側で既定0.5秒の通信タイムアウトを持ちます。継続運転中はタイムアウトより短い周期で再送してください。0速度を受信した場合は現在位置をholdします。position modeは通信断で目標値を破棄しません。

### 複数jointと複数point

- `joint_names`は空にできません。
- `positions`、`velocities`、`effort`は空配列、または`joint_names`と同じ要素数が必要です。
- `time_from_start`付きの複数pointはbridgeのROSクロックを基準に順番にUDP送信します。
- 新しい`JointTrajectory`を受けると、まだ送っていない以前のtrajectory pointをすべて置き換えます。
- pointにpositionがあればposition、なければvelocity、effortだけならeffort modeとしてencodeします。

## 単位

| joint | position | velocity | effort |
|---|---|---|---|
| revolute | rad | rad/s | N·m |
| prismatic | m | m/s | N |

`/ros2_ksp/diagnostics`の`estimated_current_a`は、推定effortを`torquePerAmpNm`または`forcePerAmpN`で割った値です。実測電流ではありません。

## MotorCommand / MotorState

`JointTrajectory`を使わず、単一モーターを型付きTopicで指令することもできます。

| MotorCommand field | 内容 |
|---|---|
| `header` | 現行bridgeでは指令変換に使用しない |
| `vessel_id` / `controller_id` / `lease_id` | 取得済みauthority identity |
| `sequence` | 同じlease内で単調増加する番号 |
| `id` | 対象のROS joint ID |
| `mode` | `MODE_POSITION=0`、`MODE_VELOCITY=1`、`MODE_EFFORT=2` |
| `enabled` | `false`ならモーターをdisengage |
| `position` | position modeの目標。回転rad、直動m |
| `velocity` | velocity modeの目標。回転rad/s、直動m/s |
| `effort` | effort modeの目標。回転N·m、直動N |
| `timeout_sec` | override時間。0ならbridge既定値 |

```bash
ros2 topic pub --once /ksp_vessel/actuators/servo/command \
  ksp_ros2_interfaces/msg/MotorCommand \
  "{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 2, id: servo_12345, mode: 0, enabled: true, position: 1.5708, timeout_sec: 0.5}"
```

| MotorState field | 内容 |
|---|---|
| `header` | bridge受信時刻、`frame_id = base_link` |
| `id` | ROS joint ID |
| `name` | ROS joint名 |
| `motor_type` | `revolute` / `prismatic` |
| `enabled` | engage状態 |
| `mode` | 現在のcommand mode |
| `position` / `velocity` / `effort` | 現在値。単位は上表 |
| `target` | 現在の目標値 |
| `current` | 推定電流A |
| `powered` | ElectricCharge利用可否 |
| `locked` | lock状態 |
| `command_active` | ROS override保持中か |

例のidentityは、先に[機体制御API](/api/vehicle-control)で取得したleaseへ置き換えてください。`MotorCommand`はownerだけが使用できます。ROSモーターはBody Wrenchの配分対象ではありません。timeout後は`command_active`がfalseになり、velocity modeは現在位置のholdへ移ります。position modeの目標位置は保持されます。

## DiagnosticStatus

| 条件 | level | message |
|---|---|---|
| 電力なし | ERROR | `ElectricCharge unavailable` |
| disengaged | WARN | `Motor disengaged` |
| locked | WARN | `Servo locked` |
| 通常 | OK | `Motor operational` |

## 機体内の衝突判定

モーター本体と`top`ノードに接続された駆動側パーツは、飛行中も互いのコライダーが有効です。既存の機体ファイルでKSPの`sameVesselCollision`が無効として保存されていても、ジョイント初期化時に両パーツへ適用されます。

## 実装確認先

- `GameData/KerbalLiDAR/Parts/RosServo/part.cfg`
- `GameData/KerbalLiDAR/Parts/RosLinearMotor/part.cfg`
- `Source/KerbalLiDAR/Api/Ksp/KerbalRosMotorSupport.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/motor_packets.py`
- `Ros2/ksp_ros2_interfaces/msg/MotorCommand.msg`
- `Ros2/ksp_ros2_interfaces/msg/MotorState.msg`
