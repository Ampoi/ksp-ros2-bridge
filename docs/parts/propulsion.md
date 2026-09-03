# エンジン / RCS

Flight中のactive vesselにあるKSP標準`ModuleEngines` / `ModuleEnginesFX`と`ModuleRCS` / `ModuleRCSFX`を自動検出します。専用パーツへの置き換えは不要です。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 入力 | `/ksp_vessel/actuators/propulsion/command` | `ksp_ros2_interfaces/msg/EngineCommand` | `id`で指定するEngine指令 |
| 出力 | `/ksp_vessel/actuators/propulsion/state` | `ksp_ros2_interfaces/msg/EngineState` | `id`付きEngine状態 |
| 入力 | `/ksp_vessel/actuators/rcs/command` | `ksp_ros2_interfaces/msg/RcsCommand` | `id`で指定するRCS指令 |
| 出力 | `/ksp_vessel/actuators/rcs/state` | `ksp_ros2_interfaces/msg/RcsState` | `id`付きRCS状態 |
| 入力 | `/ksp_vessel/actuators/propulsion/main_throttle` | `std_msgs/msg/Float64` | メインスロットル`0.0..1.0` |
| 入力 | `/ksp_vessel/actuators/rcs/twist_command` | `geometry_msgs/msg/Twist` | RCS 6軸入力、各`-1.0..1.0` |
| 入力 | `/ksp_vessel/actuators/propulsion/json_command` | `std_msgs/msg/String` | 互換用JSON指令 |
| 出力 | `/ksp_vessel/actuators/propulsion/json_state` | `std_msgs/msg/String` | 互換用JSON状態、10 Hz |

## State JSON

すべてのstateに共通するfieldです。

| Field | 型 | 内容 |
|---|---|---|
| `type` | string | `ksp_propulsion_state` |
| `version` | number | `1` |
| `name` | string | `engine_<partFlightId>_<moduleIndex>`または`rcs_...` |
| `kind` | string | `engine` / `rcs` |
| `vessel` | string | active vessel名 |
| `partFlightId` | number | KSP part ID |
| `moduleIndex` | number | Part.Modules内のindex |
| `universalTime` | number | KSP universal time |
| `enabled` | boolean | Engine ignitedまたはRCS enabled |
| `flameout` | boolean | flameout状態 |
| `throttleLimit` | number | module固有の推力上限`0.0..1.0` |
| `thrust` | number | 現在推力 |
| `maxThrust` | number | 定格最大推力 |
| `commandActive` | boolean | ROS overrideを保持中 |
| `timedOut` | boolean | overrideがタイムアウト済み |

Engineには`engineId`、`operational`、`throttleable`、`throttle`が追加されます。RCSには`active`と`thrusterCount`が追加されます。

```bash
ros2 topic echo /ksp_vessel/actuators/propulsion/json_state
```

## 個別モジュール指令

`String.data`にはJSON objectを入れます。複数moduleは`commands`配列へまとめられます。

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/json_command std_msgs/msg/String \
  '{data: "{\"commands\":[{\"name\":\"engine_12345_0\",\"kind\":\"engine\",\"enabled\":true,\"throttle\":0.65}],\"timeout\":0.5}"}'
```

| Field | 必須 | 内容 |
|---|---|---|
| `commands[].name` | 条件付き | stateの`name`。全対象は`*` |
| `commands[].partFlightId` + `moduleIndex` | 条件付き | `name`の代わりにID指定 |
| `commands[].kind` | 任意 | `engine` / `rcs`。省略時は両方を探索 |
| `commands[].enabled` | 条件付き | Engine起動・停止、RCS module有効・無効 |
| `commands[].throttle` | 条件付き | module固有推力`0.0..1.0` |
| `commands[].release` | 条件付き | ROS overrideを解除し元の設定へ復帰 |
| `timeout` | 任意 | 0.05〜10秒。既定0.5秒 |

各commandには`enabled`、`throttle`、`release`のいずれかが必要です。

全moduleのoverrideを解除します。

```bash
ros2 topic pub --once /ksp_vessel/actuators/propulsion/json_command std_msgs/msg/String \
  '{data: "{\"commands\":[{\"name\":\"*\",\"release\":true}]}"}'
```

## メインスロットル

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/main_throttle \
  std_msgs/msg/Float64 '{data: 0.8}'
```

0.0〜1.0以外や非有限値はbridgeで拒否します。既定では最後の指令から0.5秒で0へ落ちます。

## RCS 6軸

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/rcs/twist_command \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.2, z: 0.0}}'
```

| Twist | KSP入力 |
|---|---|
| `linear.x` / `.y` / `.z` | X / Y / Z並進 |
| `angular.x` | pitch |
| `angular.y` | yaw |
| `angular.z` | roll |

受信中はRCSアクショングループを自動的に有効にします。タイムアウトまたはactive vessel切替時は6軸入力を0へ戻し、元のRCSアクショングループ状態を復元します。pitch / yaw / rollはKSP共通操舵軸のため、リアクションホイールや舵面も反応します。

## フェイルセーフとKSP制約

メインスロットル、RCS 6軸、module固有throttleは既定0.5秒で0になります。連続噴射中は`-r 5`など、タイムアウトより短い周期でpublishしてください。`enabled`だけの起動・停止にはthrottle timeoutを設定しません。

再点火不可、停止不可、燃料切れ、stage条件、`throttleLocked`などKSP標準moduleの制約が優先されます。

## 型付きEngine Topic

Engine IDは`engine_<persistentId>_<moduleIndex>`です。commandはReliable、stateはBest Effortで、どちらもdepth 10です。

| EngineCommand field | 単位 | 内容 |
|---|---|---|
| `header` | — | 現行bridgeでは指令変換に使用しない |
| `id` | — | 対象Engine ID |
| `enabled` | — | Engineの起動状態 |
| `target_thrust` | N | 目標推力 |
| `timeout_sec` | s | override時間。0ならbridge既定値 |

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/command \
  ksp_ros2_interfaces/msg/EngineCommand \
  "{id: engine_12345_0, enabled: true, target_thrust: 50.0, timeout_sec: 0.5}"
```

| EngineState field | 単位 | 内容 |
|---|---|---|
| `header` | — | bridge受信時刻、`frame_id = base_link` |
| `id` | — | Engine ID |
| `name` | — | アクチュエータ名 |
| `enabled` / `operational` / `flameout` | — | KSP module状態 |
| `throttle` | 0.0..1.0 | 現在スロットル |
| `thrust` / `max_thrust` | N | 現在推力 / 最大推力 |
| `command_active` | — | 型付きoverride保持中か |

## 型付きRCS Topic

RCS IDは`rcs_<persistentId>_<moduleIndex>`です。QoSはEngineと同じです。

| RcsCommand field | 単位 | 内容 |
|---|---|---|
| `header` | — | 現行bridgeでは指令変換に使用しない |
| `id` | — | 対象RCS ID |
| `enabled` | — | RCS moduleの有効状態 |
| `thrust_limit` | N | moduleの推力上限 |
| `timeout_sec` | s | override時間。0ならbridge既定値 |

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/rcs/command \
  ksp_ros2_interfaces/msg/RcsCommand \
  "{id: rcs_12345_1, enabled: true, thrust_limit: 2.0, timeout_sec: 0.5}"
```

| RcsState field | 単位 | 内容 |
|---|---|---|
| `header` | — | bridge受信時刻、`frame_id = base_link` |
| `id` | — | RCS ID |
| `name` | — | アクチュエータ名 |
| `enabled` / `active` / `flameout` | — | KSP module状態 |
| `thrust` / `max_thrust` / `thrust_limit` | N | 現在推力、最大推力、現在上限 |
| `command_active` | — | 型付きoverride保持中か |

型付きcommandは対象moduleについてBody Wrench配分より優先されます。timeoutまたはactive vessel切替時はoverrideを解除し、元のEngine independent throttleまたはRCS設定へ戻します。有効なtimeout範囲はKSP側で0.05〜10秒です。

## 実装確認先

- `Source/KerbalLiDAR/Api/Ksp/KerbalRosPropulsionSupport.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/propulsion_packets.py`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/udp_bridge.py`
- `Ros2/ksp_ros2_interfaces/msg/EngineCommand.msg`
- `Ros2/ksp_ros2_interfaces/msg/EngineState.msg`
- `Ros2/ksp_ros2_interfaces/msg/RcsCommand.msg`
- `Ros2/ksp_ros2_interfaces/msg/RcsState.msg`
