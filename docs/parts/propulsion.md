# エンジン / RCS

Flight中のactive vesselにあるKSP標準`ModuleEngines` / `ModuleEnginesFX`と`ModuleRCS` / `ModuleRCSFX`を自動検出します。専用パーツへの置き換えは不要です。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 入力 | `/ksp_vessel/actuators/propulsion/command` | `pylon_interfaces/msg/EngineCommand` | `id`で指定するEngine指令 |
| 出力 | `/ksp_vessel/actuators/propulsion/state` | `pylon_interfaces/msg/EngineState` | `id`付きEngine状態 |
| 入力 | `/ksp_vessel/actuators/rcs/command` | `pylon_interfaces/msg/RcsCommand` | `id`で指定するRCS指令 |
| 出力 | `/ksp_vessel/actuators/rcs/state` | `pylon_interfaces/msg/RcsState` | `id`付きRCS状態 |

## フェイルセーフとKSP制約

メインスロットル、RCS 6軸、module固有throttleは既定0.5秒で0になります。連続噴射中はタイムアウトより短い周期で、lease内の`sequence`を毎回増やしてpublishしてください。`enabled`だけの起動・停止にはthrottle timeoutを設定しません。

再点火不可、停止不可、燃料切れ、stage条件、`throttleLocked`などKSP標準moduleの制約が優先されます。

## 型付きEngine Topic

Engine IDは`engine_<persistentId>_<moduleIndex>`です。commandはReliable、stateはBest Effortで、どちらもdepth 10です。

| EngineCommand field | 単位 | 内容 |
|---|---|---|
| `header` | — | 現行bridgeでは指令変換に使用しない |
| `vessel_id` / `controller_id` / `lease_id` | — | 取得済みauthority identity |
| `sequence` | — | 同じlease内で単調増加する番号 |
| `id` | — | 対象Engine ID |
| `enabled` | — | Engineの起動状態 |
| `target_thrust` | N | 目標推力 |
| `timeout_sec` | s | override時間。0ならbridge既定値 |

```bash
ros2 topic pub --once /ksp_vessel/actuators/propulsion/command \
  pylon_interfaces/msg/EngineCommand \
  "{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 2, id: engine_12345_0, enabled: true, target_thrust: 50.0, timeout_sec: 0.5}"
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

## エンジンTVC（推力偏向）

型付き `EngineCommand` で、対象エンジンの標準 `ModuleGimbal` に個別入力を渡せます。`ModuleEnginesFX` も対象です。追加パーツは不要です。

| TVC Command field | 内容 |
|---|---|
| `has_gimbal_command` | `true` でTVCを制御。`false`（既定値）で以前のTVC overrideを解除 |
| `gimbal_pitch` | KSP pitch入力、−1〜1 |
| `gimbal_yaw` | KSP yaw入力、−1〜1 |
| `gimbal_roll` | KSP roll入力、−1〜1 |

これらは偏向角（rad）やROSの回転ベクトルではなく、KSPの操舵入力です。対象ジンバルだけが反応し、リアクションホイール・RCS・舵面へこの入力を送ることはありません。標準ジンバルの可動範囲、limiter、応答速度、lock、pitch/yaw/rollの有効設定を尊重します。ロールの効き方はノズル配置に依存します。

取得済みleaseを指定した例です。`target_thrust` はNで、TVCと同じメッセージで送ります。

```bash
ros2 topic pub --once /ksp_vessel/actuators/propulsion/command \
  pylon_interfaces/msg/EngineCommand \
  "{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 3, id: engine_12345_0, enabled: true, target_thrust: 50000.0, has_gimbal_command: true, gimbal_pitch: 0.2, gimbal_yaw: -0.1, gimbal_roll: 0.0, timeout_sec: 0.5}"
```

継続制御はtimeoutより短い周期で送信し、送るたびに `sequence` を増やしてください。同じsequenceの繰り返しは受理されません。非有限値や−1〜1の範囲外は拒否します。timeout、lease喪失、緊急停止、機体切替時は偏向を中立へ戻し、標準ジンバル制御へ復帰します。

| TVC State field | 内容 |
|---|---|
| `gimbal_available` | 対象エンジンのノズルに対応する標準ジンバルがある |
| `gimbal_command_active` | 対応するジンバルにTVC overrideがある |
| `gimbal_pitch` / `gimbal_yaw` / `gimbal_roll` | ジンバルへ渡した指令入力。実測偏向角ではない。overrideなしは0 |

TVC非対応エンジンは推力制御のみ動作し、`gimbal_available` はfalseです。複数のエンジンモードが同じジンバルを共有する場合、現在有効な指令のうち最大sequenceのTVC入力が優先され、stateは共有ハードウェアの指令を返します。Body Wrenchの自動配分はTVC角を最適化しません。TVCはこの個別Engine APIで指定してください。

## 型付きRCS Topic

RCS IDは`rcs_<persistentId>_<moduleIndex>`です。QoSはEngineと同じです。

| RcsCommand field | 単位 | 内容 |
|---|---|---|
| `header` | — | 現行bridgeでは指令変換に使用しない |
| `vessel_id` / `controller_id` / `lease_id` | — | 取得済みauthority identity |
| `sequence` | — | 同じlease内で単調増加する番号 |
| `id` | — | 対象RCS ID |
| `enabled` | — | RCS moduleの有効状態 |
| `thrust_limit` | N | moduleの推力上限 |
| `timeout_sec` | s | override時間。0ならbridge既定値 |

```bash
ros2 topic pub --once /ksp_vessel/actuators/rcs/command \
  pylon_interfaces/msg/RcsCommand \
  "{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 2, id: rcs_12345_1, enabled: true, thrust_limit: 2.0, timeout_sec: 0.5}"
```

| RcsState field | 単位 | 内容 |
|---|---|---|
| `header` | — | bridge受信時刻、`frame_id = base_link` |
| `id` | — | RCS ID |
| `name` | — | アクチュエータ名 |
| `enabled` / `active` / `flameout` | — | KSP module状態 |
| `thrust` / `max_thrust` / `thrust_limit` | N | 現在推力、最大推力、現在上限 |
| `command_active` | — | 型付きoverride保持中か |

例のidentityは、先に[機体制御API](/api/vehicle-control)で取得したleaseへ置き換えてください。型付きcommandはownerだけが使用でき、対象moduleについてBody Wrench配分より優先されます。timeoutまたはactive vessel切替時はoverrideを解除し、元のEngine independent throttleまたはRCS設定へ戻します。
