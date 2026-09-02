# システム概要

KerbalLiDARは、KSP 1.xのFlightシーンとROS2ノードの間を2本のUDP経路で接続します。ROS2側では、用途ごとに標準メッセージへ変換されたTopicだけを扱えばよく、UDP JSONを直接処理する必要はありません。

<div class="topic-flow">
  <div><strong>KSP plugin</strong>センサー、機体状態、Ground TruthをUDP 49010へ送信</div>
  <div><strong>ROS2 bridge</strong>UDP JSONと標準ROS2メッセージを相互変換</div>
  <div><strong>ROS2 nodes</strong>Topicを購読し、指令Topicへpublish</div>
</div>

指令は逆向きに、ROS2 bridgeからKSPのUDP 49011へ送られます。

## データフロー

| 方向 | 内容 | 既定経路 |
|---|---|---|
| KSP → bridge | センサー、モーター/アクチュエータ状態、推進系、Ground Truth、URDF | UDP `127.0.0.1:49010` |
| bridge → KSP | モーター、推進系、Body Wrench、型付きアクチュエータ指令 | UDP `127.0.0.1:49011` |
| bridge → ROS2 | センサー、状態、診断、Ground Truth、URDF、TF | ROS2 Topic |
| ROS2 → bridge | モーター、推進系、機体・アクチュエータ指令 | ROS2 Topic |

## パーツと機能

| KSP側 | ROS2側の主なAPI | 役割 |
|---|---|---|
| Kerbal LiDAR 2D | `sensor_msgs/msg/LaserScan` | 平面距離スキャン |
| Kerbal LiDAR 3D | `sensor_msgs/msg/PointCloud2` | 前方半球点群 |
| Kerbal ROS2 RGB Camera | `Image` + `CameraInfo` | RGB画像と内部パラメーター |
| ROS2 Size-0 Axial Servo | `JointTrajectory` / `JointState` | 回転軸制御 |
| ROS2 Telescoping I-Beam Actuator | `JointTrajectory` / `JointState` | 最大約2倍に伸びる直動軸制御 |
| KSP標準Engine / RCS | `String` / `Float64` / `Twist` | 推進・6軸入力 |
| KSP標準Wheel / Engine / RCS | `ksp_ros2_interfaces` | パーツ単位の型付き制御・状態 |
| KSP標準ドッキングポート | `DockingPortCommand/State` + `Image` | 状態、切離し、選択式ポートカメラ |
| active vessel | `WrenchStamped` / `PoseStamped` | 機体要求とGround Truth |

標準Engine / RCSは専用パーツではありません。active vessel内の`ModuleEngines`系と`ModuleRCS`系をFlight開始後に自動検出します。

## Topicの寿命

- bridge状態、集約モーター/推進系、機体制御、Ground Truth、モデル関連のTopicはbridge起動時から存在します。
- LiDARとカメラのTopicは、対象パーツから最初のデータを受け取った時点で動的に作成されます。
- `/actuators/<name>`はactive vesselのmanifestまたは最初のstateで動的に作成され、通常はmanifestから外れるか無通信timeoutになると削除されます。切断済みの分離機構だけは最終stateを保持し、active vessel切替時に削除されます。
- `/ros2_ksp/docking_ports/<name>`はactive vesselのドッキングポートmanifestから動的に作成され、ポート消失または無通信timeoutで削除されます。
- センサーTopicはKSPからinactive通知を受けると削除されます。通知が欠落した場合も、既定では最終受信から3秒後に削除されます。
- 次のFlightでは、最初のデータ受信時に同じTopicが再作成されます。

## 名前と座標系

センサーTopicの`<part_name>`は、VAB/SPHのパーツ右クリックメニューから設定します。英数字とアンダースコアへ正規化され、同一機体内の重複には`_2`、`_3`のような接尾辞が付きます。

LiDARはROSセンサー座標の`+X`前方・`+Z`上方です。カメラはREP-103 optical座標の`+X`右・`+Y`下・`+Z`前方です。active vesselモデルを受信できた場合、各メッセージの`frame_id`は対応する機体linkの子frameへ接続されます。
