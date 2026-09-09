# システム概要

初めて導入する場合は、[Getting Started](getting-started.md)で必要なソフトのインストールから受信確認まで進めてください。

PyLoNは、KSP 1.xのFlightシーンとROS2ノードの間を2本のUDP経路で接続します。ROS2側では、用途ごとに標準メッセージへ変換されたTopicだけを扱えばよく、UDP JSONを直接処理する必要はありません。

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
| bridge → KSP | authority、lease-bound Wrench、型付きアクチュエータ指令 | UDP `127.0.0.1:49011` |
| bridge → ROS2 | センサー、状態、診断、Ground Truth、URDF、TF | ROS2 Topic |
| ROS2 → bridge | モーター、推進系、機体・アクチュエータ指令 | ROS2 Topic |

## パーツと機能

| KSP側 | ROS2側の主なAPI | 役割 |
|---|---|---|
| PyLoN LiDAR 2D | `sensor_msgs/msg/LaserScan` | 平面距離スキャン |
| PyLoN LiDAR 3D | `sensor_msgs/msg/PointCloud2` | 前方半球点群 |
| PyLoN RGB Camera | `Image` + `CameraInfo` | RGB画像と内部パラメーター |
| PyLoN Size-0 Axial Servo | `MotorCommand` / `JointState` | 回転軸制御 |
| PyLoN Slim Telescoping Actuator | `MotorCommand` / `JointState` | 最大約2倍に伸びる直動軸制御 |
| KSP標準Wheel / Engine / RCS | `pylon_interfaces` | パーツ単位の型付き制御・状態 |
| KSP標準ドッキングポート | `DockingPortCommand/State` + `Image` | 状態、切離し、選択式ポートカメラ |
| active vessel | `BodyWrenchCommand` / `WrenchFeedback` / `PoseStamped` | 所有権付き機体要求・実現量・Ground Truth |

標準Engine / RCSは専用パーツではありません。active vessel内の`ModuleEngines`系と`ModuleRCS`系をFlight開始後に自動検出します。

## Topicの寿命

- bridge状態、集約モーター/推進系、機体制御、Ground Truth、モデル関連のTopicはbridge起動時から存在します。
- LiDARとカメラのTopicは、対象パーツから最初のデータを受け取った時点で動的に作成されます。
- `/ksp_vessel/actuators/<type>`は種類ごとの常設Topicで、commandの`id`により個体を選択します。切断済みの分離状態はactive vessel切替まで保持されます。
- `/ksp_vessel/docking_ports/<id>`はactive vesselのドッキングポートmanifestから動的に作成され、ポート消失または無通信timeoutで削除されます。
- センサーTopicはKSPからinactive通知を受けると削除されます。通知が欠落した場合も、既定では最終受信から3秒後に削除されます。
- 次のFlightでは、最初のデータ受信時に同じTopicが再作成されます。

## 名前と座標系

センサーTopicの`<sensor_id>`は、VAB/SPHのパーツ右クリックメニューから設定します。英数字とアンダースコアへ正規化され、同一機体内の重複には`_2`、`_3`のような接尾辞が付きます。

LiDARはROSセンサー座標の`+X`前方・`+Z`上方です。カメラはREP-103 optical座標の`+X`右・`+Y`下・`+Z`前方です。active vesselモデルを受信できた場合、各メッセージの`frame_id`はSensor ID由来の安定名になり、対応する機体linkへ`/tf_static`で接続されます。

## Flight中にパーツIDを確認する

Flight画面の標準ツールバーにある`ID`ボタンへマウスを重ねると、操作機体のROS2対応パーツにIDを表示します。クリックすると表示を固定でき、もう一度クリックしてマウスを外すと非表示になります。

対象はセンサー、モーター、ホイール、エンジン、RCS、分離機構、ドッキングポート、可動フィンです。ラベルにはTopicや指令に使用するIDを表示し、線で対象パーツを示します。同じパーツに複数のIDがあれば併記します。機体切替・分離・ドッキング後は現在の操作機体に追従し、マップ画面とF2によるUI非表示中はラベルを隠します。
