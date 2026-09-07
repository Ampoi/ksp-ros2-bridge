# ksp_lidar_bridge

KerbalLiDARのUDP JSONを受け取り、LiDAR、RGBカメラ、ロボティクスモーター、標準エンジン/RCS、Flight中のactive vesselランタイムプロキシをROS2へpublishする`ament_python`パッケージです。

kRPCは使用しません。このbridgeはKerbalLiDAR KSPプラグインと直接UDP通信し、kRPCサーバーやPythonの`krpc`パッケージには依存しません。

- Default vessel IMU: `/ksp_vessel/imu/data_raw` (`sensor_msgs/msg/Imu`, 3軸角速度rad/s・比力m/s²、`base_link`、最大30 Hz)。全機体で追加パーツ不要。操作中の機体に自動追従し、姿勢なし（`orientation_covariance[0]=-1`）。静止時は上向き約+g、自由落下時は約0。
- 2D LiDAR: `sensor_msgs/msg/LaserScan`
- 3D LiDAR: `sensor_msgs/msg/PointCloud2`
- 2D Topic: `/ksp_vessel/lidar_2d/<sensor_id>/scan`
- 3D Topic: `/ksp_vessel/lidar_3d/<sensor_id>/points`
- RGB image: `/ksp_vessel/camera/<sensor_id>/image_raw` (`sensor_msgs/msg/Image`, `rgb8`)
- Camera calibration: `/ksp_vessel/camera/<sensor_id>/camera_info` (`sensor_msgs/msg/CameraInfo`)
- Bridge status: `/ros2_ksp/status`
- Active vessel URDF: `/ksp_vessel/robot_description`
- Active vessel root frame: `/ksp_vessel/root_frame`
- Active vessel dynamic pose (`ground_truth_enu` → `base_link` → proxy root): `/tf`
- Proxy fixed joints and sensor mounts: `/tf_static`
- Legacy motor trajectory (opt-in): `/ksp_vessel/actuators/servo/trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- Motor state: `/ksp_vessel/joint_states` (`sensor_msgs/msg/JointState`)
- Motor diagnostics/current estimate: `/ros2_ksp/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- Typed actuators: `/ksp_vessel/actuators/<type>/{command,state}`（commandの`id`で対象指定）
- Legacy main throttle (opt-in): `/ksp_vessel/actuators/propulsion/main_throttle` (`std_msgs/msg/Float64`)
- Legacy RCS 6-axis command (opt-in): `/ksp_vessel/actuators/rcs/twist_command` (`geometry_msgs/msg/Twist`)
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

3D点群は、現在のKSP modが使うコンパクトなFibonacci半球配置をrange配列から復元します。明示的な方向配列についても、フラット形式（`[x,y,z,...]`）と旧形式のネスト配列（`[[x,y,z],...]`）の両方を受信できます。

## Build

ROS2 Jazzy（Ubuntu 24.04、Python 3.12）を対象とします。

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cp -r Ros2/ksp_ros2_interfaces ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to ksp_lidar_bridge
source install/setup.bash
```

Humbleでビルド済みのworkspaceを使う場合は、Python 3.10向けの`build`、`install`、`log`を削除してからJazzyで再ビルドしてください。

## Run

```bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

通常は`ROS_LOCALHOST_ONLY`の設定やROS2 daemonの再起動は不要です。起動直後から`/ros2_ksp/status`が作成されるため、KSPが未起動でも通常の`ros2 topic list`でbridgeを確認できます。LiDAR固有TopicはIDと2D/3D種別を最初のUDPスキャンから決定するため、Flight中にスキャンを受信した後に作成されます。

停止通知が欠落した場合のTopic削除時間は`--topic-timeout-sec`で変更できます（デフォルト3秒）。LiDARの最低スキャン周期より長い正の値を指定してください。

KSP以外のホストへ指令を返す場合:

```bash
ros2 run ksp_lidar_bridge udp_bridge --command-host 192.168.1.50 --command-port 49011
```

確認例:

```bash
ros2 topic list
ros2 topic echo --once /ros2_ksp/status
ros2 topic echo /ksp_vessel/lidar_2d/front_lidar/scan
ros2 topic echo --once /ksp_vessel/camera/rgb_camera/camera_info
ros2 topic echo --once /ksp_vessel/root_frame
ros2 topic echo /ksp_vessel/joint_states
ros2 topic echo /ros2_ksp/diagnostics
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

Ground Truthは操作機体を選択した地点を原点とする`ground_truth_enu`で、pose、world/body frameの速度、運動学的加速度を配信します。`ground_truth_enu`から`base_link`へのTFもセンサーデータと同じKSP universal timeへ対応付けます。

Flight中に検出された各ホイール、Engine、RCS、ROSモーター、デカプラー、手動展開式フェアリングには、`persistentId`とmodule indexから安定した`<name>`が自動生成されます。正式な型付きcommandにも同じauthority identityとsequenceが必要です。分離機構は`SeparationCommand`の`separate: true`で作動し、`SeparationState`をReliable / Transient Localで保持します。ドッキングポートは専用の`DockingPortCommand/State` APIで状態、カメラ選択、Undock/Decoupleを扱います。`ModuleJettison`は分離APIの対象外です。

## Active vessel runtime proxy

URDFは`std_msgs/msg/String`をtransient-local QoSでpublishします。bridgeはKSPの永続的なvessel IDからproxy link名を作り、CoM基準の`base_link -> proxy root`だけをdynamic TFとして更新します。URDF固定jointとSensor ID由来の安定したmount frameは`/tf_static`です。このためGround Truthの`ground_truth_enu -> base_link`から搭載センサーまでが1本のTFツリーになります。操作機体の構成が変わった場合だけmodelを更新し、機体切替時は不一致modelを直ちに切断します。

主なオプション:

- `--robot-description-topic`: URDF Topic名
- `--root-frame-topic`: RViz Fixed Frame確認用Topic名
- `--model-tf-rate`: CoMからproxy rootへのdynamic TF更新Hz
- `--allow-remote-models`: 非loopback送信元のモデルパケットを許可

KSPからbridgeへのモデル経路は既定でfail-closedです。KSP側の`udpHost`はloopback、`allowRemoteUrdf`は`false`のまま利用してください。別ホストからモデルを受け取る場合だけ`--allow-remote-models`を指定します。ROS2 Topicの到達範囲は通常のDDS設定に従うため、同一ホストだけに制限する場合はbridgeとROS2 CLIを起動する全ターミナルで`ROS_LOCALHOST_ONLY=1`を設定してください。

ランタイムプロキシはKSPのmesh、texture、part名、asset pathを一切含まず、匿名linkとコライダー由来のbox/cylinder/sphere近似だけで構成されます。mesh colliderは外形比と表面形状からプリミティブを選び、カプセル形状は円柱と球の組み合わせにします。各プリミティブはパーツローカルの中心・姿勢を保持するため、機体のワールド姿勢で寸法は変わりません。受信時はgzip展開量、チャンク数、SHA-256、XMLタグ・属性を検証し、`mesh`や外部参照を拒否します。bridge自身はURDFをディスクへ保存しません。ただし、ROS2 Topicを購読できる同一ホストのプロセスによるコピーを技術的に完全禁止するものではありません。

## Motor control

次は`--enable-legacy-control`を付けた移行用`JointTrajectory`例です（`servo_12345`は`/ksp_vessel/joint_states.name`で確認）。新規コードではauthority付き`MotorCommand`を使います。

```bash
ros2 topic pub --once /ksp_vessel/actuators/servo/trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [servo_12345], points: [{positions: [1.5708], velocities: [0.5], effort: [100.0]}]}"
```

単位はROS規約に合わせ、回転位置/速度/effortがrad・rad/s・N·m、直動位置/速度/effortがm・m/s・Nです。`JointTrajectoryPoint.time_from_start`付きの複数pointにも対応します。新しいtrajectoryは未送信の古いtrajectoryを置き換えます。

`/ros2_ksp/diagnostics`の`estimated_current_a`はKSP側の推定モーター出力とパーツ設定のトルク定数/推力定数から計算した値です。KSPには実電流センサーがないため、実測値ではありません。

## Propulsion control

KSP側はactive vesselの標準`ModuleEngines` / `ModuleEnginesFX`と`ModuleRCS` / `ModuleRCSFX`を自動検出します。型付きcommandは`id`で対象を指定します。以下のJSON / Float64 / Twist例は`--enable-legacy-control`を付けた移行用途だけで、新規コードではauthority付きの型付きcommandまたはBody Wrenchを使います。

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/json_command std_msgs/msg/String \
  '{data: "{\"commands\":[{\"name\":\"engine_12345_0\",\"enabled\":true,\"throttle\":0.65}],\"timeout\":0.5}"}'

ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/main_throttle std_msgs/msg/Float64 \
  '{data: 0.8}'

ros2 topic pub -r 5 /ksp_vessel/actuators/rcs/twist_command geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.2, z: 0.0}}'
```

推力系の指令は既定0.5秒でフェイルセーフ停止するため、噴射中はタイムアウトより短い周期で送信します。JSON commandの`timeout`は0.05〜10秒、bridgeのFloat64/Twist用タイムアウトは`--propulsion-timeout-sec`で変更できます。`{"commands":[{"name":"*","release":true}]}`を送ると全モジュールのROSオーバーライドを解除します。

RCSの`Twist.linear.x/y/z`はKSPのX/Y/Z、`angular.x/y/z`はpitch/yaw/rollです。受信中はRCSアクショングループも自動で有効になります。ノズル選択はKSP標準のRCS制御に任せ、モジュールごとの推力上限と有効状態をString commandで指定します。pitch/yaw/rollはKSP共通軸なのでリアクションホイールや舵面も反応します。固体燃料、再点火不可、停止不可などの制約はKSPのモジュール設定が優先されます。

## Test

ROS2を読み込まずにパケット変換ロジックをテストできます。

```bash
python3 -m unittest discover -s test -v
```
