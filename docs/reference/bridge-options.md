# Bridge起動オプション

## 基本形

```bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

## UDPとノード

| 引数 | 既定値 | 内容 |
|---|---|---|
| `--host` | `0.0.0.0` | KSP→bridge UDPのbind host |
| `--port` | `49010` | KSP→bridge UDPのbind port |
| `--command-host` | `127.0.0.1` | bridge→KSP指令の送信先host |
| `--command-port` | `49011` | bridge→KSP指令の送信先port |
| `--max-datagram-bytes` | `65535` | 1回のUDP受信上限 |
| `--node-name` | `ksp_lidar_udp_bridge` | ROS2 node名 |

同一PC内だけで使う場合は、既定の`0.0.0.0`ではなく`--host 127.0.0.1`を明示するとUDP受信範囲をloopbackへ限定できます。

## センサーとframe

| 引数 | 既定値 | 内容 |
|---|---|---|
| `--topic-prefix` | `/ksp_vessel` | 機体センサーTopicのprefix |
| `--bridge-prefix` | `/ros2_ksp` | bridge statusのprefix |
| `--frame-prefix` | `ros2_ksp` | model未接続時のセンサーframe prefix |
| `--topic-timeout-sec` | `3.0` | 最終データ受信から動的publisherを削除する秒数 |
| `--docking-ports-prefix` | `<topic-prefix>/docking_ports` | ドッキングポートstate/command/cameraのprefix |

`--topic-timeout-sec`は有限の正数のみ受け付けます。利用する最低センサー周期より長くしてください。

## Active vesselモデル

| 引数 | 既定値 | 内容 |
|---|---|---|
| `--robot-description-topic` | `/ksp_vessel/robot_description` | URDF Topic |
| `--root-frame-topic` | `/ksp_vessel/root_frame` | root frame Topic |
| `--model-tf-rate` | `5.0` | 固定joint TFのpublish Hz。内部で0.5〜60 Hzへclamp |
| `--allow-remote-models` | 無効 | 非loopbackからのモデルパケットを許可 |

## モーターTopic

| 引数 | 既定値 |
|---|---|
| `--motor-command-topic` | `/ksp_vessel/actuators/servo/trajectory` |
| `--joint-states-topic` | `/ksp_vessel/joint_states` |
| `--diagnostics-topic` | `/ros2_ksp/diagnostics` |

## 推進系Topicとtimeout

| 引数 | 既定値 |
|---|---|
| `--propulsion-command-topic` | `/ksp_vessel/actuators/propulsion/json_command` |
| `--propulsion-state-topic` | `/ksp_vessel/actuators/propulsion/json_state` |
| `--main-throttle-topic` | `/ksp_vessel/actuators/propulsion/main_throttle` |
| `--rcs-command-topic` | `/ksp_vessel/actuators/rcs/twist_command` |
| `--propulsion-timeout-sec` | `0.5` |

`--propulsion-timeout-sec`はFloat64メインスロットルとTwist RCS指令に付与するKSP側フェイルセーフ時間です。有限の正数のみ受け付けます。String JSON指令はpayloadの`timeout`で0.05〜10秒を指定します。

## 機体制御・型付きアクチュエータ

| 引数 | 既定値 | 内容 |
|---|---|---|
| `--body-wrench-topic` | `/ksp_vessel/body_wrench` | 機体Wrench入力Topic |
| `--ground-truth-prefix` | `/ksp_vessel/ground_truth` | pose / twist / accelerationのprefix |
| `--actuators-prefix` | `/ksp_vessel/actuators` | 種類別の型付きアクチュエータTopicのprefix |
| `--vehicle-command-timeout-sec` | `0.5` | Body Wrenchと型付きcommandの既定timeout |

`--vehicle-command-timeout-sec`は有限の正数のみ受け付けます。ホイール、Engine、RCS、モーターの型付きcommandで`timeout_sec`に0以外を指定すると、その値を優先します。KSP側では0.05〜10秒へclampされます。不可逆な`SeparationCommand`にはtimeoutはありません。

## すべてを独自namespaceへ移す例

```bash
ros2 run ksp_lidar_bridge udp_bridge \
  --host 127.0.0.1 \
  --topic-prefix /my_rover \
  --bridge-prefix /my_bridge \
  --motor-command-topic /my_rover/actuators/servo/trajectory \
  --joint-states-topic /my_rover/joint_states \
  --diagnostics-topic /my_bridge/diagnostics \
  --propulsion-command-topic /my_rover/actuators/propulsion/json_command \
  --propulsion-state-topic /my_rover/actuators/propulsion/json_state \
  --main-throttle-topic /my_rover/actuators/propulsion/main_throttle \
  --rcs-command-topic /my_rover/actuators/rcs/twist_command \
  --body-wrench-topic /my_rover/body_wrench \
  --ground-truth-prefix /my_rover/ground_truth \
  --actuators-prefix /my_rover/actuators \
  --docking-ports-prefix /my_rover/docking_ports \
  --robot-description-topic /my_rover/robot_description \
  --root-frame-topic /my_rover/root_frame
```

実装上の全オプションは次でも確認できます。

```bash
ros2 run ksp_lidar_bridge udp_bridge --help
```
