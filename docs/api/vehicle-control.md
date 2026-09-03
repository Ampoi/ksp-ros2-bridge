# 機体制御とGround Truth

機体全体へ要求するWrench、シミュレーションのGround Truth、パーツ単位の型付きアクチュエータを提供します。`cmd_vel`や`nav_msgs/msg/Odometry`は使用しません。

## Body Wrench入力

| Topic | 型 | QoS | timeout |
|---|---|---|---|
| `/ksp_vessel/body_wrench` | `geometry_msgs/msg/WrenchStamped` | Reliable / depth 10 | 既定0.5秒 |

`header.frame_id`は空文字列または`base_link`だけを受け付けます。座標は`+X`前、`+Y`左、`+Z`上、forceはN、torqueはN·mです。全成分は有限値である必要があります。

```bash
ros2 topic pub -r 10 /ksp_vessel/body_wrench geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: base_link}, wrench: {force: {x: 1000.0}, torque: {z: 100.0}}}"
```

bridgeは要求をUDPでKSPへ送り、接地ホイール、作動中の主エンジン、RCSへ飽和付きで配分します。機体Rigidbodyへ直接forceを加えません。接地状態、推力方向、重心からのモーメント、推力上限、燃料切れを考慮し、実現できなかった残差比が10%を超えると`/ros2_ksp/diagnostics`へWARNをpublishします。

最後の指令から`--vehicle-command-timeout-sec`が経過すると要求は解除されます。既定値は0.5秒なので、継続制御ではそれより短い周期でpublishしてください。

## Ground Truth出力

| Topic | 型 | frame_id | 内容 |
|---|---|---|---|
| `/ksp_vessel/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | `ground_truth_enu` | 位置m、姿勢quaternion |
| `/ksp_vessel/ground_truth/twist` | `geometry_msgs/msg/TwistStamped` | `ground_truth_enu` | 線速度m/s、角速度rad/s |
| `/ksp_vessel/ground_truth/acceleration` | `geometry_msgs/msg/AccelStamped` | `ground_truth_enu` | 線加速度m/s²、角加速度rad/s² |
| `/tf` | `tf2_msgs/msg/TFMessage` | `ground_truth_enu -> base_link` | poseと同じ位置・姿勢 |

stateとTFは30 Hzで配信し、3つのstate TopicはBest Effortです。`ground_truth_enu`は操作機体を選択した地点を原点とする東・北・上座標で、KSPの浮動原点には依存しません。機体または天体が切り替わると原点、`originSequence`、加速度の微分履歴をリセットします。加速度は速度差分から求める運動学的な値です。

```bash
ros2 topic echo /ksp_vessel/ground_truth/pose
ros2 topic hz /ksp_vessel/ground_truth/twist
ros2 run tf2_ros tf2_echo ground_truth_enu base_link
```

## 型付きアクチュエータとの優先順位

ホイール、Engine、RCSの`/ksp_vessel/actuators/<type>/command`は、`id`で指定した対象についてBody Wrench配分より優先されます。commandの`timeout_sec`が0の場合は`--vehicle-command-timeout-sec`を使用し、期限切れ後は個別overrideを解除してBody Wrench配分へ戻ります。有効なtimeout範囲はKSP側で0.05〜10秒です。ROSサーボとリニアモーターはBody Wrenchの配分対象ではなく、専用のモーター制御系で動作します。

型付きTopicは種類ごとに常設されます。commandはReliable、通常のstateはBest Effortです。

## 起動引数

| 引数 | 既定値 | 内容 |
|---|---|---|
| `--body-wrench-topic` | `/ksp_vessel/body_wrench` | Wrench入力Topic |
| `--ground-truth-prefix` | `/ksp_vessel/ground_truth` | 3つのGround Truth Topicのprefix |
| `--actuators-prefix` | `/ksp_vessel/actuators` | 型付きアクチュエータTopicのprefix |
| `--vehicle-command-timeout-sec` | `0.5` | Wrenchと型付きcommandの既定timeout |

## 実装確認先

- `Source/KerbalLiDAR/Api/Ksp/KerbalRosVehicleSupport.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/vehicle_packets.py`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/udp_bridge.py`
