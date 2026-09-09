# pylon_bridge

PyLoNのUDP JSONを受け取り、LiDAR、RGBカメラ、ロボティクスモーター、標準エンジン/RCS、Flight中のactive vesselランタイムプロキシをROS2へpublishする`ament_python`パッケージです。

kRPCは使用しません。このbridgeはPyLoN KSPプラグインと直接UDP通信し、kRPCサーバーやPythonの`krpc`パッケージには依存しません。

- Default vessel IMU: `/ksp_vessel/imu/data_raw` (`sensor_msgs/msg/Imu`, 3軸角速度rad/s・比力m/s²、`base_link`、最大30 Hz)。全機体で追加パーツ不要。操作中の機体に自動追従し、姿勢なし（`orientation_covariance[0]=-1`）。静止時は上向き約+g、自由落下時は約0。角速度は慣性系基準で、KSPの回転物理座標系では惑星の自転を含めます。

`--disable-ground-truth`を付けると、真値パケットを破棄し、真値Topicと`pylon_ground_truth_enu -> base_link`を配信しません。機体IDとlifecycleは独立したセッションheartbeatから生成し、`origin_sequence`はROS側のセッション世代です。LiDAR＋IMUだけで動く[デブリ周回デモ](../../Demo/pylon_demo_debris_orbit/README.md)の検証に使用します。

センサーtimestampは初回のKSP universal timeとROS時計のoffsetを固定して対応付けます。遅延した画像や物理時間の進みの遅さでoffsetを変更するとジャイロ積分に架空の時間差が入るため、飛行中は補正しません。セッションheartbeatで機体切替や時刻の巻き戻りによるepoch変更を検知した際に初期化します。受信timeoutは単調時計で判定します。
- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- 2D Topic: `/ksp_vessel/lidar_2d/<sensor_id>/scan`
- 3D Topic: `/ksp_vessel/lidar_3d/<sensor_id>/points`
- RGB image: `/ksp_vessel/camera/<sensor_id>/image_raw` (`sensor_msgs/msg/Image`, `rgb8`)
- Camera calibration: `/ksp_vessel/camera/<sensor_id>/camera_info` (`sensor_msgs/msg/CameraInfo`)
- Bridge status: `/pylon/status`
- Active vessel URDF: `/ksp_vessel/robot_description`
- Active vessel root frame: `/ksp_vessel/root_frame`
- Active vessel dynamic pose (`pylon_ground_truth_enu` → `base_link` → proxy root): `/tf`
- Proxy fixed joints and sensor mounts: `/tf_static`
- Motor state: `/ksp_vessel/joint_states` (`sensor_msgs/msg/JointState`)
- Motor diagnostics/current estimate: `/pylon/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- Typed actuators: `/ksp_vessel/actuators/<type>/{command,state}`（commandの`id`で対象指定）
- Control authority: `/ksp_vessel/control/authority/{command,state}`
- Body wrench: `/ksp_vessel/control/wrench_command` (`BodyWrenchCommand`、`base_link`)
- Wrench feedback: `/ksp_vessel/control/wrench_feedback` (`WrenchFeedback`)
- Vessel lifecycle: `/ksp_vessel/lifecycle` (`VesselLifecycle`)
- Ground truth: `/ksp_vessel/ground_truth/{pose,twist,twist_body,acceleration}`
- Nearby vessel truth: `/ksp_vessel/ground_truth/nearby_vessels` (`NearbyVessels`), containing the observer and up to 32 loaded, unpacked nearby vessels in one timestamp and world origin. The debris demo can subtract these absolute states before using LiDAR estimates.
- Separation actuators: `SeparationCommand/State` for stock decouplers and procedural fairings
- Docking ports: `/ksp_vessel/docking_ports/<id>/{state,command}` (`DockingPortState/Command`)
- Selected docking camera: `/ksp_vessel/docking_ports/<id>/camera/{image_raw,camera_info}`

TopicはKSPのFlight中にスキャンを受信したときだけ作成されます。Flightを終了するとKSPからの停止通知で削除され、通知を受け取れなかった場合もスキャン停止から3秒後に自動削除されます。再びFlightへ入ると、最初のスキャン受信時に自動で再作成されます。

RGB画像は上端始まりの`rgb8`で、対応する`CameraInfo`とtimestampおよび`frame_id`を共有します。frameの軸はREP-103 camera optical規約（+X右、+Y下、+Z前方）です。

3D点群は、現在のKSP modが使うコンパクトなFibonacci半球配置をrange配列から復元します。明示的な方向配列はフラット形式（`[x,y,z,...]`）だけを受信します。

内部の通信責務と将来のROS 2／Space ROS分離境界は[設計文書](ARCHITECTURE.md)を参照してください。

## Build

ROS2 Jazzy（Ubuntu 24.04、Python 3.12）を対象とします。

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/pylon_bridge ~/ros2_ws/src/
cp -r Ros2/pylon_interfaces ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to pylon_bridge
source install/setup.bash
```

Humbleなど別のROS版でビルドしたworkspaceと共用せず、Jazzy用のworkspaceを用意してください。初回導入は[Getting Started](../../docs/guide/getting-started.md)を参照してください。

## Run

```bash
ros2 run pylon_bridge udp_bridge --host 127.0.0.1 --port 49010
```

通常は`ROS_LOCALHOST_ONLY`の設定やROS2 daemonの再起動は不要です。起動直後から`/pylon/status`が作成されるため、KSPが未起動でも通常の`ros2 topic list`でbridgeを確認できます。LiDAR固有TopicはIDと2D/3D種別を最初のUDPスキャンから決定するため、Flight中にスキャンを受信した後に作成されます。

停止通知が欠落した場合のTopic削除時間は`--topic-timeout-sec`で変更できます（デフォルト3秒）。LiDARの最低スキャン周期より長い正の値を指定してください。

KSP以外のホストへ指令を返す場合:

```bash
ros2 run pylon_bridge udp_bridge --command-host 192.168.1.50 --command-port 49011
```

確認例:

```bash
ros2 topic list
ros2 topic echo --once /pylon/status
ros2 topic echo /ksp_vessel/lidar_2d/front_lidar/scan
ros2 topic echo --once /ksp_vessel/camera/rgb_camera/camera_info
ros2 topic echo --once /ksp_vessel/root_frame
ros2 topic echo /ksp_vessel/joint_states
ros2 topic echo /pylon/diagnostics
ros2 topic echo /ksp_vessel/actuators/propulsion/state
```

`<sensor_id>`はVAB/SPHのセンサーパーツ右クリックメニューにある`Edit ROS2 Sensor ID`で設定します。LiDARとRGBカメラだけが編集可能なIDを持ち、新規パーツには種類付き8桁UIDが自動設定されます。Topicルートを変更する場合は`--topic-prefix`を指定してください。

## Vehicle authority, wrench, ground truth, and typed actuators

正式APIでは`/ksp_vessel/lifecycle`の実`vessel_id`へ期限付きleaseを取得してから、`/ksp_vessel/control/wrench_command`へ`BodyWrenchCommand`を送ります。KSPがowner、sequence、対象機体を検証し、lease期間中のSAS排他とemergency stopを管理します。

Wrenchは`base_link`（+X前、+Y左、+Z上）のN/N·m要求です。各RCSノズルの位置、噴射方向、CoMからのモーメントアーム、axis enableを使ってKSP操作channelへ配分します。KSPの剛体へ直接Forceを加えません。

```bash
ros2 topic echo /ksp_vessel/lifecycle
ros2 topic echo /ksp_vessel/control/authority/state
ros2 topic echo /ksp_vessel/control/wrench_feedback
```

`WrenchFeedback`は`requested / allocated / achieved / residual`を公開します。`achieved`は直前のphysics tickで観測したengineとRCS推力からの再構成値で、reaction wheel・接触力・空力は含みません。KSP内の安全filterはforce/torque、角速度、変化率、連続噴射時間を制限します。

Ground Truthは操作機体を選択した地点を原点とする`pylon_ground_truth_enu`で、pose、world/body frameの速度、運動学的加速度を配信します。`pylon_ground_truth_enu`から`base_link`へのTFもセンサーデータと同じKSP universal timeへ対応付けます。

Flight中に検出された各ホイール、Engine、RCS、ROSモーター、デカプラー、手動展開式フェアリングには、`persistentId`とmodule indexから安定した`<name>`が自動生成されます。正式な型付きcommandにも同じauthority identityとsequenceが必要です。分離機構は`SeparationCommand`の`separate: true`で作動し、`SeparationState`をReliable / Transient Localで保持します。ドッキングポートは専用の`DockingPortCommand/State` APIで状態、カメラ選択、Undock/Decoupleを扱います。`ModuleJettison`は分離APIの対象外です。

## Active vessel runtime proxy

URDFは`std_msgs/msg/String`をtransient-local QoSでpublishします。bridgeはKSPの永続的なvessel IDからproxy link名を作り、CoM基準の`base_link -> proxy root`だけをdynamic TFとして更新します。URDF固定jointとSensor ID由来の安定したmount frameは`/tf_static`です。このためGround Truthの`pylon_ground_truth_enu -> base_link`から搭載センサーまでが1本のTFツリーになります。操作機体の構成が変わった場合だけmodelを更新し、機体切替時は不一致modelを直ちに切断します。

主なオプション:

- `--robot-description-topic`: URDF Topic名
- `--root-frame-topic`: RViz Fixed Frame確認用Topic名
- `--model-tf-rate`: CoMからproxy rootへのdynamic TF更新Hz
- `--allow-remote-models`: 非loopback送信元のモデルパケットを許可

KSPからbridgeへのモデル経路は既定でfail-closedです。KSP側の`PYLON_TRANSPORT.stateHost`はloopback、`allowRemoteUrdf`は`false`のまま利用してください。別ホストからモデルを受け取る場合だけ`--allow-remote-models`を指定します。ROS2 Topicの到達範囲は通常のDDS設定に従うため、同一ホストだけに制限する場合はbridgeとROS2 CLIを起動する全ターミナルで`ROS_LOCALHOST_ONLY=1`を設定してください。

ランタイムプロキシはKSPのmesh、texture、part名、asset pathを一切含まず、匿名linkとコライダー由来のbox/cylinder/sphere近似だけで構成されます。mesh colliderは外形比と表面形状からプリミティブを選び、カプセル形状は円柱と球の組み合わせにします。各プリミティブはパーツローカルの中心・姿勢を保持するため、機体のワールド姿勢で寸法は変わりません。受信時はgzip展開量、チャンク数、SHA-256、XMLタグ・属性を検証し、`mesh`や外部参照を拒否します。bridge自身はURDFをディスクへ保存しません。ただし、ROS2 Topicを購読できる同一ホストのプロセスによるコピーを技術的に完全禁止するものではありません。

## Motor control

Lease付き`pylon_interfaces/msg/MotorCommand`を使用します。[Topicと単位](../../docs/parts/motors.md)。

## Propulsion control

EngineCommand・RcsCommand・BodyWrenchCommandを使用します。[制御契約](../../docs/api/vehicle-control.md)。

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```

真値比較用`/ksp_vessel/ground_truth/frame_angular_velocity`は惑星固定ENU軸の回転率です。IMU周回の評価器はこれを使って真値を慣性系へ移し、推定姿勢と比較します。推定器・制御器では使いません。
