# KSP標準ホイール

active vessel内の`ModuleWheelBase`を自動検出し、共有の型付きTopicで制御します。専用パーツへの置き換えは不要です。

## 入出力Topic

`<name>`は`wheel_<persistentId>_<moduleIndex>`です。`persistentId`が0の場合だけ`flightID`を使います。

| 方向 | Topic | 型 | QoS |
|---|---|---|---|
| 入力 | `/ksp_vessel/actuators/wheel/command` | `ksp_ros2_interfaces/msg/WheelCommand` | Reliable / depth 10 |
| 出力 | `/ksp_vessel/actuators/wheel/state` | `ksp_ros2_interfaces/msg/WheelState` | Best Effort / depth 10 |

状態はFlight中に30 Hzで送信されます。

## WheelCommand

| Field | 単位 | 内容 |
|---|---|---|
| `header` | — | 現行bridgeでは指令変換に使用しない |
| `vessel_id` / `controller_id` / `lease_id` | — | 取得済みauthority identity |
| `sequence` | — | 同じlease内で単調増加する番号 |
| `id` | — | 対象の`wheel_<persistentId>_<moduleIndex>` |
| `enabled` | — | ホイールモーターを有効化 |
| `target_angular_velocity` | rad/s | 目標回転速度 |
| `steering_angle` | rad | 目標操舵角 |
| `max_drive_torque` | N·m | 最大駆動トルク |
| `timeout_sec` | s | override時間。0ならbridge既定値 |

```bash
ros2 topic pub --once /ksp_vessel/actuators/wheel/command \
  ksp_ros2_interfaces/msg/WheelCommand \
  "{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 2, id: wheel_12345_2, enabled: true, target_angular_velocity: 12.0, steering_angle: 0.2, max_drive_torque: 20.0, timeout_sec: 0.5}"
```

## WheelState

| Field | 単位 | 内容 |
|---|---|---|
| `header` | — | bridge受信時刻、`frame_id = base_link` |
| `id` | — | アクチュエータID |
| `name` | — | アクチュエータ名 |
| `enabled` | — | モーター有効状態 |
| `grounded` | — | 接地状態 |
| `angular_position` | rad | ホイール回転位置 |
| `angular_velocity` | rad/s | ホイール回転速度 |
| `steering_angle` | rad | 現在操舵角 |
| `drive_torque` | N·m | 駆動トルク |
| `brake_torque` | N·m | ブレーキトルク |
| `slip` | — | KSPのcombined tire slip |
| `max_drive_torque` | N·m | 現在の最大駆動トルク |

```bash
ros2 topic echo /ksp_vessel/actuators/wheel/state
```

## 制御の優先順位と解除

例のidentityは、先に[機体制御API](/api/vehicle-control)で取得したleaseへ置き換えてください。型付きcommandを受けたホイールはBody Wrench allocatorより優先されます。timeoutまたはactive vessel切替時にはdrive / steer入力を0へ戻し、元のmotor有効状態と最大トルクを復元します。

## 実装確認先

- `Source/KerbalLiDAR/Api/Ksp/KerbalRosVehicleSupport.cs`
- `Ros2/ksp_ros2_interfaces/msg/WheelCommand.msg`
- `Ros2/ksp_ros2_interfaces/msg/WheelState.msg`
