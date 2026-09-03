# デカプラー／フェアリング

Flight中のactive vesselにある通常・ラジアルデカプラーと手動展開式フェアリングを自動検出します。ドッキングポートと`ModuleJettison`はこのAPIの対象外です。

## 入出力Topic

| 方向 | Topic | 型 | QoS |
|---|---|---|---|
| 入力 | `/ksp_vessel/actuators/separation/command` | `ksp_ros2_interfaces/msg/SeparationCommand` | Reliable / Volatile / depth 10 |
| 出力 | `/ksp_vessel/actuators/separation/state` | `ksp_ros2_interfaces/msg/SeparationState` | Reliable / Transient Local / depth 10 |

IDは`decoupler_<persistentId>_<moduleIndex>`または`fairing_<persistentId>_<moduleIndex>`です。stateの`id`で確認できます。

## 状態

| フィールド | 内容 |
|---|---|
| `id` | 対象の安定ID |
| `name` | 対象の安定名 |
| `mechanism` | `decoupler`または`fairing` |
| `available` | 現在、KSP標準操作で切断・展開可能か |
| `separated` | 切断済み、またはフェアリング展開済みなら`true` |

切断済み状態は部品がactive vesselから外れてもbridgeに保持され、後から購読したsubscriberにも届きます。active vesselを切り替えると前の機体の保持状態を破棄します。

```bash
ros2 topic echo /ksp_vessel/actuators/separation/state
```

## 切断・展開指令

`separate: true`を1回publishすると、KSP標準の切断またはフェアリング展開処理を実行します。操作は不可逆なので、実`vessel_id`に対する取得済みauthority leaseと単調増加`sequence`が必須です。owner以外、`false`、作動済み、利用不能、存在しない名前への指令は拒否または無視されます。

```bash
ros2 topic pub --once /ksp_vessel/actuators/separation/command \
  ksp_ros2_interfaces/msg/SeparationCommand \
  '{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 2, id: decoupler_12345_0, separate: true}'
```

```bash
ros2 topic pub --once /ksp_vessel/actuators/separation/command \
  ksp_ros2_interfaces/msg/SeparationCommand \
  '{vessel_id: <vessel-id>, controller_id: manual, lease_id: <lease-id>, sequence: 3, id: fairing_67890_1, separate: true}'
```

identityは先に[機体制御API](/api/vehicle-control)で取得したleaseへ置き換えてください。lease外から不可逆操作を行う互換経路はありません。
