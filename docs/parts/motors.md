# サーボ / リニアモーター

回転サーボとリニアモーターは共通のROS2 Topicを使います。`MotorCommand.id`に対象joint名を指定し、状態は全モーターをまとめた`JointState`で受け取ります。

## パーツ

| パーツ | joint type | 可動範囲 | 既定速度 | 定格effort |
|---|---|---:|---:|---:|
| ROS2 Size-0 Axial Servo | `revolute` | -π〜π rad | 45°/s | 250 N·m |
| ROS2 Slim Telescoping Actuator | `prismatic` | 0〜1.6 m | 0.5 m/s | 4000 N |

両方とも`bottom`側を親構造へ、動かすパーツ群を`top`側へ取り付けます。リニアアクチュエーターは直径0.3125 mの取付円盤を持ち、縮長1.6049312 m、ストローク1.6 m、最大長3.2049312 mです。白い固定筒`LinearBase`に対し、中間筒`LinearSleeve`は伸長量の半分、先端ロッド`LinearRod`は伸長量の全量だけY軸方向へ動きます。先端のstack nodeも伸縮に追従します。`Travel Speed`で0.02〜1.0 m/sの速度をエディター・飛行中に調整できます（既定0.5 m/s）。可動部はネイティブモデルに含まれるため、後からstockモデルを生成する必要はありません。

先端に`PhysicsSignificance = 1`のパーツ（小型花火ランチャーなど）を接続した場合、飛行中にKSPの`PromoteToPhysicalPart()`で独立した物理ボディへ昇格させ、駆動用の接続を生成します。初期化は先端パーツの起動とunpackを待って再試行します。セーブには目標値に加えて実際の伸長量と可動側の機体座標を保存します。読込後の伸長量は接続面同士の実際の距離から測定します。

リニア駆動はKSPの接続を構成する全`PartJoint.joints`を設定し、unpackなどでKSPが固定用springを再設定した後も駆動を復元します。定格4000 NはKSPの物理単位で4 kNに換算し、並列ジョイント全体で分担します。

サーボは直径0.625 m（size 0）、全高0.140 mの専用2円盤モデルです。下側の`ServoBase`は固定、上側の`ServoRotor`はY軸まわりに回転し、オレンジの指標と下側の目盛りで相対角度を確認できます。接続面は底面Y=0、上面Y=0.140 mです。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 出力 | `/ksp_vessel/joint_states` | `sensor_msgs/msg/JointState` | position、velocity、effort |
| 出力 | `/pylon/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 電源・engage・lock・推定電流 |
| 入力 | `/ksp_vessel/actuators/servo/command` | `pylon_interfaces/msg/MotorCommand` | `id`で指定する型付き指令 |
| 出力 | `/ksp_vessel/actuators/servo/state` | `pylon_interfaces/msg/MotorState` | `id`付きの型付き状態 |

既定ではKSPが状態を20 HzでUDP 49010へ送り、bridgeは受信するたびにROS2へpublishします。ROS2指令はbridgeからUDP 49011へ送られます。

## Joint名

`motorName`が空の場合はKSPの`partFlightId`から自動生成します。

```text
servo_<partFlightId>
linear_<partFlightId>
```

実際の名前は`/ksp_vessel/joint_states.name`またはKSPのPart Action Windowにある`ROS Joint`で確認してください。

型付きcommand TopicはReliable、state TopicはBest Effortで、どちらもdepth 10です。Topicは常設され、複数モーターのメッセージが同じTopicを流れます。

## 単位

| joint | position | velocity | effort |
|---|---|---|---|
| revolute | rad | rad/s | N·m |
| prismatic | m | m/s | N |

`/pylon/diagnostics`の`estimated_current_a`は、推定effortを`torquePerAmpNm`または`forcePerAmpN`で割った値です。実測電流ではありません。

## MotorCommand / MotorState

単一モーターをlease付きの型付きTopicで指令します。

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
  pylon_interfaces/msg/MotorCommand \
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

モーター本体と`top`ノードに接続された駆動側パーツは、飛行中も互いに衝突します。

リニアアクチュエーターでは、動く筒・ロッドと駆動側パーツ群の間だけ接触を除外します。固定筒・周囲との接触と、LiDAR等のレイキャスト用の形状は維持します。

## 可動フィンの個別角度制御

アクティブ機体の標準`ModuleControlSurface`（可動フィン、エレボン、ラダー等）を自動検出し、`fin_<persistentId>_<moduleIndex>`という回転jointとして公開します。既存機体の組み直しやModuleManagerパッチは不要です。固定翼・固定フィンと、`displaceVelocity`を使うプロペラ／ローターブレードは対象外です。

実際のIDは次の状態Topicの`id`、または`/ksp_vessel/joint_states`の`name`から確認できます。

```bash
ros2 topic echo /ksp_vessel/actuators/servo/state
```

[機体制御API](/api/vehicle-control)でleaseを取得した後、`MotorCommand`で角度を指定します。例は+10°（0.174533 rad）です。identity、ID、sequenceは実際の値へ置き換えてください。

```bash
ros2 topic pub --once /ksp_vessel/actuators/servo/command \
  pylon_interfaces/msg/MotorCommand \
  "{vessel_id: '<vessel-id>', controller_id: manual, lease_id: '<lease-id>', sequence: 2, id: fin_12345_0, mode: 0, enabled: true, position: 0.174533, timeout_sec: 0.5}"
```

- 対応modeはpositionのみです。符号は可動面モデルのローカルX軸回りで、左右対称パーツにも個別に指定します。機体全体のpitch/yaw/rollの符号とは異なります。
- 角度を各パーツの`±ctrlSurfaceRange`に制限し、KSP標準のアクチュエーター速度と空力計算で動かします。
- 指令中はその面のpitch/yaw/roll入力と展開設定を一時的に上書きします。フィン同士の連動は行いません。
- フィンはposition指令にもタイムアウトがあります。既定0.5秒、指定可能範囲0.05～5秒です。保持するにはタイムアウトより短い周期で、lease内の`sequence`を毎回増やして再送してください。lease自体の更新も必要です。
- `enabled: false`、通信タイムアウト、制御権解放／変更／緊急停止、機体切替、機体のpack時に元のKSP設定へ戻ります。元々展開していた面はその展開設定に戻ります。
- 状態の`position`は標準モジュールの現在舵角、`target`は制限後の目標角で、単位はradです。速度・トルク・電流は取得できないため0を返します。`powered`は制御可能状態、`enabled`と`command_active`はROSによる上書き中を表します。
