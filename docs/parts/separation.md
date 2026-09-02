# デカプラー／フェアリング

Flight中のactive vesselにある通常・ラジアルデカプラーと手動展開式フェアリングを自動検出します。ドッキングポートと`ModuleJettison`はこのAPIの対象外です。

## 入出力Topic

| 方向 | Topic | 型 | QoS |
|---|---|---|---|
| 入力 | `/actuators/<name>/command` | `ksp_ros2_interfaces/msg/SeparationCommand` | Reliable / Volatile / depth 10 |
| 出力 | `/actuators/<name>/state` | `ksp_ros2_interfaces/msg/SeparationState` | Reliable / Transient Local / depth 1 |

名前は`decoupler_<persistentId>_<moduleIndex>`または`fairing_<persistentId>_<moduleIndex>`です。実際の名前は`ros2 topic list | grep '^/actuators/'`で確認できます。

## 状態

| フィールド | 内容 |
|---|---|
| `name` | 対象の安定名 |
| `mechanism` | `decoupler`または`fairing` |
| `available` | 現在、KSP標準操作で切断・展開可能か |
| `separated` | 切断済み、またはフェアリング展開済みなら`true` |

切断済み状態は部品がactive vesselから外れてもbridgeに保持され、後から購読したsubscriberにも届きます。active vesselを切り替えると前の機体のTopicは削除されます。

```bash
ros2 topic echo /actuators/decoupler_12345_0/state
```

## 切断・展開指令

`separate: true`を1回publishすると、KSP標準の切断またはフェアリング展開処理を実行します。操作は不可逆です。`false`、既に作動済みの対象、利用不能な対象、存在しない名前への指令は無視されます。

```bash
ros2 topic pub --once /actuators/decoupler_12345_0/command \
  ksp_ros2_interfaces/msg/SeparationCommand '{separate: true}'
```

```bash
ros2 topic pub --once /actuators/fairing_67890_1/command \
  ksp_ros2_interfaces/msg/SeparationCommand '{separate: true}'
```

対象機構は常にROS2指令を受け付けます。KSP内に許可スイッチはありません。
