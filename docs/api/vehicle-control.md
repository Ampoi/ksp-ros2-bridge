# 機体制御とGround Truth

正式な機体制御APIは、KSPの実`vessel_id`へ期限付きleaseを取得してから、body frameのWrenchまたは型付きアクチュエータ指令を送ります。KSP側が所有権、SAS排他、sequence、timeout、安全上限を最終判定します。

## 制御フロー

1. `/ksp_vessel/lifecycle`で`STATE_ACTIVE`または`STATE_CHANGED`と`vessel_id`を受け取る。
2. `/ksp_vessel/control/authority/command`へ`ACTION_ACQUIRE`を送る。
3. `/ksp_vessel/control/authority/state`で同じ`controller_id`と`lease_id`の`STATE_OWNED`を確認する。
4. leaseと同じidentityを持つ指令を、timeoutより短い周期で送る。
5. 制御終了時に`ACTION_RELEASE`を送る。停止時に送れなくてもleaseは期限切れになる。

`sequence`は同じ`controller_id + lease_id`の全正式指令を通じて単調増加させます。同じ値や古い値はKSPで拒否されます。`priority`が高い新規leaseだけが現在のownerをpreemptできます。同じpriorityでは先に取得したownerを維持します。

## Topic

| 方向 | Topic | 型 | QoS / 内容 |
|---|---|---|---|
| Subscribe | `/ksp_vessel/control/authority/command` | `ControlAuthorityCommand` | Reliable。取得、更新、解放、e-stop |
| Publish | `/ksp_vessel/control/authority/state` | `ControlAuthorityState` | Reliable / Transient Local。KSPが確定したowner |
| Subscribe | `/ksp_vessel/control/wrench_command` | `BodyWrenchCommand` | Reliable。lease-bound body Wrench |
| Publish | `/ksp_vessel/control/wrench_feedback` | `WrenchFeedback` | requested / allocated / achieved / residual |
| Publish | `/ksp_vessel/lifecycle` | `VesselLifecycle` | Reliable / Transient Local。機体identityとframe状態 |

`BodyWrenchCommand.header.frame_id`は空または`base_link`だけを受け付けます。座標は+X前、+Y左、+Z上、forceはN、torqueはN·mです。`timeout_sec`は0.05〜10秒です。

角速度・トルクもROSの右手系です。Unityからの変換では、位置・力の軸入替に加え、軸性ベクトルの左右反転符号を補正します。角速度はGround Truth姿勢quaternionの時間差分と同じ回転方向になります。

手動確認では、まずlifecycleから実際のIDを確認します。

```bash
ros2 topic echo --once /ksp_vessel/lifecycle
ros2 topic echo /ksp_vessel/control/authority/state
ros2 topic echo /ksp_vessel/control/wrench_feedback
```

通常の連続制御には、lease更新とsequence採番を行う`pylon_vehicle_control`を使ってください。

```bash
ros2 run pylon_vehicle_control setpoint_controller --ros-args \
  -p controller_id:=my_controller \
  -p setpoint_topic:=/my_controller/setpoint
```

共通controllerの`ATTITUDE_HOLD`と`SIX_DOF`は、姿勢誤差を`attitude_hold_rate_limit_deg_s`以下の目標body角速度へ変換して内側の角速度loopを閉じます。上限を超える回転をさらに加速するtorque成分も除き、制動を優先します。`DETUMBLE`はbody角速度と反対向きのtorqueだけを生成します。

## 「要求」と「実現」の違い

このAPIはN/N·mを受け取ります。KSP内部の推進系はt・kN・kN·m系なので、KSP adapter境界で推力・トルクをSIへ変換してから配分し、観測値もSIへ戻して公開します。したがって`20 N`がKSP内部の`20 kN`として扱われることはありません。一方、KSPのnormalized flight-control inputを使うため、指定Wrenchを誤差なく生成する理想force sourceではありません。

RCS配分器は、現在有効な各ノズルについて次をKSPと同じ軸規則で評価し、12個の正負操作channelを解きます。

- `useZaxis`を含む実際のノズル噴射軸
- ノズル位置と現在のcenter of massから得るモーメントアーム
- pitch / yaw / rollとX / Y / Zのenable設定
- moduleの作動状態と現在の最大推力

`WrenchFeedback`の値は次の意味です。

| field | 意味 |
|---|---|
| `requested` | controllerが送った値 |
| `allocated` | 安全filterとアクチュエータ配分後の要求値 |
| `achieved` | 直前のKSP physics tickで観測したengine/RCS推力から再構成した値 |
| `allocation_residual` | `requested - allocated` |
| `tracking_residual` | `requested - achieved` |
| `saturation_ratio` | 配分できなかった割合 |
| `tracking_error_ratio` | 観測値との差の割合 |

`achieved`にはreaction wheel、タイヤ接触力、空力は含みません。`achieved_quality`に測定遅延と除外対象を明記します。並進による回転や燃料・KSP制御則による差を隠さず、controller側で飽和を判断できます。

型付き`EngineCommand.target_thrust`、`RcsCommand.thrust_limit`、`WheelCommand.max_drive_torque`と対応するstateも同じくN/N·mです。KSP内部単位をROS messageへ直接露出しません。

## SAS・emergency stop・安全上限

`suppress_sas: true`のleaseを取得すると、ownerが存在する期間を通じてKSP側がSASを停止し、解放または期限切れ時に元の状態へ戻します。個々のトルク指令の有無では切り替えません。

`ACTION_EMERGENCY_STOP`は通常のWrenchと個別overrideをゼロ化し、SASも停止したままlatchします。解除できるのはe-stopを発行した同じ`controller_id + lease_id`だけです。e-stopの状態は機体切替では自動解除されません。

e-stop自体は現在のownerでなくても、activeな`vessel_id`と一意なlease identity、正のsequenceを指定して発行できます。解除は同じidentityでsequenceを増やします。

```bash
ros2 topic pub --once /ksp_vessel/control/authority/command \
  pylon_interfaces/msg/ControlAuthorityCommand \
  "{action: 4, vessel_id: '<vessel-id>', controller_id: safety_operator, lease_id: '<unique-lease-id>', sequence: 1}"

ros2 topic pub --once /ksp_vessel/control/authority/command \
  pylon_interfaces/msg/ControlAuthorityCommand \
  "{action: 5, vessel_id: '<vessel-id>', controller_id: safety_operator, lease_id: '<unique-lease-id>', sequence: 2}"
```

KSP側の最終制限は`GameData/PyLoN/Config/ControlSafety.cfg`で設定します。

| 設定 | 既定値 | 内容 |
|---|---:|---|
| `maxForceN` | 250000 | force magnitude上限 |
| `maxTorqueNm` | 100000 | torque magnitude上限 |
| `maxAngularSpeedRadSec` | 0.35 | これを超える回転を加速する成分を除去 |
| `maxForceSlewNPerSec` | 50000 | force変化率上限 |
| `maxTorqueSlewNmPerSec` | 10000 | torque変化率上限 |
| `maxContinuousActuationSec` | 30 | 非ゼロ指令の連続時間上限 |
| `continuousResetIdleSec` | 0.5 | 連続時間limitを解除するゼロ指令時間 |

ROS側の上限よりKSP側を大きく設定し、KSP側は故障時の最終境界として使うのが基本です。

## Ground Truthとframe

| Topic | frame_id | 内容 |
|---|---|---|
| `/ksp_vessel/ground_truth/pose` | `pylon_ground_truth_enu` | 位置m、姿勢quaternion |
| `/ksp_vessel/ground_truth/nearby_vessels` | `pylon_ground_truth_enu` | 自機と近隣機体の同時刻・同原点の絶対位置・速度。差分から相対状態を算出 |
| `/ksp_vessel/ground_truth/twist` | `pylon_ground_truth_enu` | world-frame速度m/s、角速度rad/s |
| `/ksp_vessel/ground_truth/twist_body` | `base_link` | body-frame速度m/s、角速度rad/s |
| `/ksp_vessel/ground_truth/acceleration` | `pylon_ground_truth_enu` | world-frame運動学的加速度 |
| `/tf` | `pylon_ground_truth_enu -> base_link` | KSP universal time基準のdynamic TF |
| `/tf_static` | proxy fixed joint / sensor mount | 取付姿勢 |

センサーデータのtimestampはKSP universal timeからROS clockへ対応付けます。Ground Truthの最新poseを同じセンサー時刻へ最大0.1秒外挿してTFを補うため、点群より遅い姿勢周期によるfuture extrapolationを避けます。

`VesselLifecycle`は`UNAVAILABLE / ACTIVE / CHANGED / STALE`、generation、origin sequence、model readinessを公開します。display nameではなく`vessel_id`が制御identityです。機体切替時、既存leaseと不一致なproxy modelは即座に無効になります。

## 型付きアクチュエータ

`EngineCommand`、`RcsCommand`、`WheelCommand`、`MotorCommand`、`SeparationCommand`も同じ`vessel_id`、`controller_id`、`lease_id`、`sequence`を必須とします。ownerでない指令はKSPが拒否します。不可逆な分離操作もlease外では実行されません。


## Bridge起動引数

| 引数 | 既定値 |
|---|---|
| `--body-wrench-command-topic` | `/ksp_vessel/control/wrench_command` |
| `--control-authority-command-topic` | `/ksp_vessel/control/authority/command` |
| `--control-authority-state-topic` | `/ksp_vessel/control/authority/state` |
| `--wrench-feedback-topic` | `/ksp_vessel/control/wrench_feedback` |
| `--vessel-lifecycle-topic` | `/ksp_vessel/lifecycle` |
| `--ground-truth-prefix` | `/ksp_vessel/ground_truth` |
| `--actuators-prefix` | `/ksp_vessel/actuators` |
| `--vehicle-command-timeout-sec` | `0.5` |

共通setpoint controllerは制御権喪失・入力欠測・実行中の世代変更で目標を破棄します。復旧時は新たな`MODE_IDLE`を受けてから、後続の新しいsetpointを受理します。
