# Kerbal LiDAR / ROS2 Robotics

Kerbal Space Program 1.x向けのセンサー・ロボティクスmodです。2D/3D LiDARとRGBカメラに加え、ROS2から操作できる回転サーボとリニアモーターを追加します。KSPとROS2の間はUDP JSONで中継します。

## APIドキュメント

起動手順、全Topic、パーツごとの入出力、設定値はVitePressドキュメントにまとめています。

```bash
pnpm install
pnpm run docs:dev
```

静的ビルドは`pnpm run docs:build`です。ドキュメント本体は`docs/`にあります。

kRPCは使用しません。KSPプラグインがセンサー取得と機体制御を行い、ROS2 bridgeと直接UDP通信するため、`GameData/kRPC`やkRPCクライアントライブラリは不要です。

## できること

- 表面取付の2D LiDARパーツ
- 表面取付の3D LiDARパーツ
- 表面取付のRGBカメラパーツ（`sensor_msgs/Image` + `CameraInfo`）
- 標準ドッキングポートの状態・切離しTopicと選択式RGBポートカメラ
- `part.cfg`からレーザー本数、FOV、最大距離、スキャン周波数、UDP送信先を変更
- レイキャスト結果をUDP JSONで外部へ送信
- 別ROS2パッケージでUDP JSONを標準ROS2 Topicへ中継
- Flight中の操作機体を、資産を含まないランタイム用プロキシURDFとしてROS2へ中継
- 自船コライダーを無視する設定
- Clamp-O-Tron Jr.と同じ0.625 m径の両面スタック式回転サーボ
- 両端にパーツを取り付けられる、ストローク1.5 mのリニアモーター
- ROS2 `trajectory_msgs/msg/JointTrajectory`による位置・速度・effort上限制御
- ROS2 `sensor_msgs/msg/JointState`による位置・速度・トルク/推力フィードバック
- `diagnostic_msgs/msg/DiagnosticArray`による電源状態・推定電流フィードバック
- KSP標準エンジンとRCSをROS2から列挙し、モジュールごとに起動・停止・個別推力制御
- ROS2 `Float64`によるメインスロットルと`Twist`によるRCS 6軸制御
- 実`vessel_id`へ結び付くpriority付き制御lease、SAS排他、emergency stop
- 各RCSノズルの位置・方向に基づくWrench配分と実現量feedback
- 共通の`ControlSetpoint`→安全な6DoF機体制御package

コードの責務と依存方向は[モノレポ設計](ARCHITECTURE.md)にまとめています。実行例は`Demo/`にあります。`./sync.sh`で本番MODとROS2をビルド・同期できます。開発用コード・検証記録・モデル編集元はローカル専用の`Development/`へ分離し、Git管理・本番ビルドの対象から除外しています。

## ビルド

KSP本体のManaged DLLを参照してビルドします。KSPのインストール先を`KSPDIR`として渡してください。

```powershell
.\build.ps1 -KspDir "C:\SteamLibrary\steamapps\common\Kerbal Space Program"
```

ビルドに成功するとDLLが`GameData/KerbalLiDAR/Plugins/KerbalLiDAR.dll`へ出力されます。

## インストール

`GameData/KerbalLiDAR`フォルダをKSPの`GameData`へコピーします。

## 設定

レーザー本数などは各パーツのCFGで変更します。

- `GameData/KerbalLiDAR/Parts/Lidar2D/part.cfg`
- `GameData/KerbalLiDAR/Parts/Lidar3D/part.cfg`

主な設定値:

- `horizontalLaserCount`: 水平方向のレーザー数
- `verticalLaserCount`: 垂直方向のレーザー数。2Dでは`1`
- `horizontalFovDegrees`: 水平FOV
- `verticalFovDegrees`: 垂直FOV
- `scanRateHz`: 1秒あたりのスキャン回数
- `streamAt20Fps`: `true`にすると`scanRateHz`ではなく20Hz固定でUDP配信
- `sensorId`: ROS2 Topic名に使う機体内で一意なセンサーID。VAB/SPHで編集可能
- `maxDistance`: 最大測距距離[m]
- `rangeProfile`: 3D LiDARの`near` / `medium` / `long`距離・精度プロファイル
- `nearRangeMeters`: 近距離プロファイルの最大距離（10〜30 m、160 rays/sr）
- `mediumRangeMeters`: 中距離プロファイルの最大距離（50〜150 m、320 rays/sr）
- `longRangeMeters`: 長距離プロファイルの最大距離（150〜250 m、1024 rays/sr）
- `udpHost`: UDP送信先
- `udpPort`: UDP送信先ポート
- `activeVesselUrdfEnabled`: Flight中の操作機体プロキシURDFを送信
- `activeVesselUrdfRefreshSeconds`: URDFを再送・更新する間隔。既定値は2秒
- `activeVesselUrdfChunkBytes`: gzip圧縮後の1チャンクの最大バイト数
- `maxActiveVesselUrdfChunks`: 1モデルに許可する最大チャンク数
- `allowRemoteUrdf`: loopback以外へのURDF送信を明示的に許可。既定値は`false`
- `ignoreOwnVessel`: 自船コライダーを無視
- `forwardAxis` / `upAxis`: レイを飛ばすパーツローカル軸
- `align3DToAttachNormal`: 3D LiDARの半球中心を取付面の外向き法線へ自動的に合わせる。既定値は`true`
- `rayOriginLocalPosition`: レイ原点のパーツローカル座標。標準パーツではOBJの前面に設定
- `originOffsetMeters`: `rayOriginLocalPosition`からスキャン前方へ追加する距離
- `includeHitPoints`: ヒット座標もJSONに含める
- `includeDirections`: レイ方向もJSONに含める

3D LiDARは初期設定で取付ノード法線をモデル正面として使い、その方向側の半球を走査します。手動で軸を指定する場合は`align3DToAttachNormal = false`にし、CFG内の`forwardAxis`と`upAxis`を`+X`、`-X`、`+Y`、`-Y`、`+Z`、`-Z`のいずれかへ変更してください。2D LiDARは従来どおり`forwardAxis`と`upAxis`を使用します。

3D LiDARではPart Action Windowの`3D Range: Near`、`Medium`、`Long`から距離プロファイルを選べます。選択中のプロファイルだけ距離スライダーが表示され、指定範囲内で最大距離を調整できます。遠距離ほど半球密度を上げ、Nearは約1005 ray、Mediumは約2011 ray、Longは既定の`maxLaserCount`上限で4096 rayです。3D方向配列はUDPへ重複送信せず、bridgeが既知のFibonacci配置から復元するため、Longでも60 KBのデータグラム上限内に収まります。

## レーザープレビュー

VAB/SPHまたはFlightでLiDARパーツを右クリックし、Part Action Windowの`Show Laser Preview`を押すと、そのパーツだけ赤いレーザー線を表示します。表示中は同じボタンが`Hide Laser Preview`になり、そのパーツだけ非表示にできます。

アクショングループには`Toggle Laser Preview`として登録できます。

## ROS2センサーID

VAB/SPHでLiDARまたはRGBカメラを右クリックし、`Edit ROS2 Sensor ID`を押すとIDを編集できます。入力値はROS2名として使える英数字とアンダースコアへ正規化され、同じ機体内に同名がある場合は`_2`、`_3`のような接尾辞が自動で付きます。新規パーツには`lidar_<8桁UID>`または`camera_<8桁UID>`が設定されます。

IDはcraftファイルへ保存され、センサーTopicの名前空間になります。以前のcraftに保存された`partName`/`lidarName`は初回ロード時に移行されます。タイヤ、エンジン、RCS、ROSモーターには編集可能なIDを追加せず、KSPの`persistentId`からTopic名を自動生成します。

## ROS2

対応環境はROS2 Jazzy（Ubuntu 24.04、Python 3.12）です。KSP側からROS2プロセスは起動しません。別プロセスとして`Ros2/ksp_lidar_bridge`パッケージを起動し、KSPから飛んでくるUDP JSONをLiDAR名ごとのTopicへ変換します。

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cp -r Ros2/ksp_ros2_interfaces ~/ros2_ws/src/
cp -r Ros2/ksp_vehicle_control ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to ksp_lidar_bridge ksp_vehicle_control
source install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

Humbleから同じworkspaceを移行する場合は、Pythonバージョンと生成済みinterfaceが異なるため、`build`、`install`、`log`を削除してからJazzy環境で再ビルドしてください。

Topicはパーツごとに作られます。2D LiDARは`sensor_msgs/msg/LaserScan`、3D LiDARは`sensor_msgs/msg/PointCloud2`、RGBカメラは`sensor_msgs/msg/Image`と`sensor_msgs/msg/CameraInfo`としてpublishします。

- Bridge状態: `/ros2_ksp/status` (`std_msgs/msg/String`、起動直後から常設)
- 2D LiDAR: `/ksp_vessel/lidar_2d/<sensor_id>/scan`
- 3D LiDAR: `/ksp_vessel/lidar_3d/<sensor_id>/points`
- RGB画像: `/ksp_vessel/camera/<sensor_id>/image_raw` (`rgb8`)
- カメラ情報: `/ksp_vessel/camera/<sensor_id>/camera_info`
- ドッキング状態・指令: `/ksp_vessel/docking_ports/<id>/{state,command}`
- ドッキングカメラ: `/ksp_vessel/docking_ports/<id>/camera/{image_raw,camera_info}`

同じ機体に複数のセンサーを搭載した場合も、各パーツ名のTopicへすべてpublishされます。

TopicはFlight中にスキャンを受信したときだけ作成されます。Flight終了時に自動削除され、停止通知が欠落してもスキャン停止から3秒後に削除されます。次にFlightへ入ると最初のスキャンから自動でpublishを再開します。タイムアウトはbridgeの`--topic-timeout-sec`で変更できます。

```bash
ros2 topic list
ros2 topic echo --once /ros2_ksp/status
ros2 topic echo /ksp_vessel/lidar_2d/front_lidar/scan
```

## RGBカメラ

`Kerbal ROS2 RGB Camera`を機体表面へ取り付けると、Flight中の3D描画（銀河背景、Scaled Space、近距離／遠距離シーン。画面UIを除く）をセンサー視点で合成し、ROS2の標準カメラメッセージとして配信します。初期設定は320 x 240、5 Hz、垂直FOV 60度です。Part Action Windowから160 x 120、320 x 240、640 x 480を選択でき、フレームレートとFOVも変更できます。

UDPでは1フレームをチェックサム付きの複数チャンクへ分割し、bridgeは全チャンクが揃ったフレームだけをpublishします。ROS2側の画像は上端始まりの`rgb8`で、`Image`と`CameraInfo`のtimestampおよび`frame_id`は一致します。カメラframeはREP-103のoptical規約（+X右、+Y下、+Z前方）です。

```bash
ros2 topic hz /ksp_vessel/camera/rgb_camera/image_raw
ros2 topic echo --once /ksp_vessel/camera/rgb_camera/camera_info
ros2 run rqt_image_view rqt_image_view /ksp_vessel/camera/rgb_camera/image_raw
```

通常は`ROS_LOCALHOST_ONLY`やROS2 daemonの操作は不要です。bridgeの起動中は、KSPが未起動でも`/ros2_ksp/status`、アクチュエータ指令、モデル関連のTopicが`ros2 topic list`へ表示されます。LiDAR固有TopicだけはセンサーIDと2D/3D種別を最初のUDPスキャンから決定するため、LiDARを搭載した機体でFlightへ入った後に表示されます。

### Active vesselのランタイムURDF

Flight中は、現在操作している機体に搭載されたLiDARのうち1個だけが送信担当になります。機体のパーツ構成や相対姿勢が変わるとURDFも更新され、操作機体を切り替えると古いモデルはclearまたは有効期限切れになります。

- URDF: `/ksp_vessel/robot_description` (`std_msgs/msg/String`, transient local)
- RVizのFixed Frameに使うルート名: `/ksp_vessel/root_frame` (`std_msgs/msg/String`)
- 固定ジョイント・sensor mount TF: `/tf_static`

確認例:

```bash
ros2 topic echo --once /ksp_vessel/root_frame
ros2 topic echo --once /ksp_vessel/robot_description
```

RViz2ではRobotModel表示のDescription Topicを`/ksp_vessel/robot_description`にし、Fixed Frameには`root_frame` Topicで得た値を指定します。TFはbridgeが配信するため、この用途だけなら別の`robot_state_publisher`は不要です。LiDARの`partFlightId`が現行モデルに含まれる場合、bridgeは対応するURDF linkの子に専用LiDAR frameを配信し、`LaserScan`と`PointCloud2`の`frame_id`をそのframeへ揃えます。点群座標と2D走査角はROSのLiDAR座標（+X前方、+Z上方）です。

### URDF資産保護の境界

送信するものはKSP機体そのものの再配布用URDFではなく、そのセッション中だけ使うプロキシです。

- KSPのmesh、texture、パーツ名、メーカー名、`GameData`パスは含めません。
- visual/collisionは各パーツで現在描画中のmesh外形から作る、ローカル姿勢付きのbox/cylinder/sphere近似です。描画meshを取得できないパーツではcollider外形を使います。タイヤや構造部材を含め、複数の描画要素は要素ごとに近似し、元meshは含めません。カプセル形状は円柱と球を組み合わせます。link名は永続的なKSP vessel IDの短縮prefixを使い、同じ機体の再ロードで安定します。
- gzipチャンクはSHA-256検証され、ROS2側ではサイズ上限とXML許可リストを適用します。`mesh`や外部URIは拒否します。
- bridgeはURDFをファイル保存せず、メモリ上だけで保持します。KSPからbridgeへのUDPは既定でloopback限定です。ROS2 Topicの到達範囲は通常のDDS設定に従います。
- 別ホストのKSPからモデルを受信する場合だけ、KSPの`allowRemoteUrdf = true`とbridgeの`--allow-remote-models`を指定します。ROS2 Topicも同一ホストだけに制限したい場合は、bridgeとROS2 CLIを起動する全ターミナルで`ROS_LOCALHOST_ONLY=1`を設定してください。

ROS2 Topicへ平文をpublishした後、同じホスト上の別プロセスによる購読・rosbag保存まで完全に防ぐことはできません。この実装は再利用可能なゲーム資産を最初から含めず、ネットワーク配布と永続化を既定で避ける設計です。より強いアクセス制御が必要な環境では、OSユーザー分離とSROS2/DDS Securityも併用してください。

## 制御所有権・Body Wrench・Ground Truth

正式な機体I/Oは、active vesselの実IDへ期限付きleaseを取得し、KSP側でSAS排他と安全制限を適用します。

- 所有権: `/ksp_vessel/control/authority/{command,state}`
- 入力: `/ksp_vessel/control/wrench_command` (`BodyWrenchCommand`)
- 出力: `/ksp_vessel/control/wrench_feedback` (`WrenchFeedback`)
- 出力: `/ksp_vessel/lifecycle` (`VesselLifecycle`)
- 出力: `/ksp_vessel/ground_truth/pose` (`geometry_msgs/msg/PoseStamped`)
- 出力: `/ksp_vessel/ground_truth/twist` (`geometry_msgs/msg/TwistStamped`)
- 出力: `/ksp_vessel/ground_truth/twist_body` (`geometry_msgs/msg/TwistStamped`)
- 出力: `/ksp_vessel/ground_truth/acceleration` (`geometry_msgs/msg/AccelStamped`)
- 個別I/O: `/ksp_vessel/actuators/<type>/command`、`/ksp_vessel/actuators/<type>/state`（`id`で対象指定）

Wrenchは`base_link`（+X前、+Y左、+Z上）のN/N·mです。各RCSノズルの実噴射方向とCoMまでのモーメントアームからKSP操作channelを配分します。`requested / allocated / achieved / residual`をfeedbackするため、normalized inputで実現できなかった量をcontrollerから確認できます。

```bash
ros2 topic echo /ksp_vessel/lifecycle
ros2 topic echo /ksp_vessel/control/authority/state
ros2 topic echo /ksp_vessel/control/wrench_feedback
```

Ground Truthは操作機体を選択した地点を原点とする東・北・上の`ground_truth_enu`です。world/body両方のTwistを公開し、センサーと同じKSP universal timeへpose TFを整合させます。固定proxy jointとsensor mountは`/tf_static`です。

ホイール、Engine、RCS、ROSモーター、デカプラー、手動展開式フェアリングの正式commandにも同じlease identityとsequenceが必要です。分離機構はownerだけが作動できます。詳細は[機体制御API](docs/api/vehicle-control.md)を参照してください。

## 2D LiDAR Mapping・Nav2

`Ros2/ksp_nav2_bringup`はbridgeと分離したROS2 integration packageです。2D `LaserScan`だけからscan-to-scan ICP odometryを作り、SLAM Toolbox / AMCL / Nav2へ接続します。planar controllerは`cmd_vel`が有効な間だけauthority leaseを取得し、停止後に解放します。起動方法と制約は[2D LiDAR MappingとNav2](docs/guide/nav2.md)を参照してください。

実機体を使って上から実行できる手順は、[`test A`デブリ周回](Demo/debris_orbit/README.md#実機の準備と起動)と[`rober A` SLAM + Nav2](Ros2/ksp_nav2_bringup/README.md#rober-aで上から順に実行する手順)に分けています。デブリ周回は位置推定とRCS制御を分離し、3D LiDAR＋IMUで真値を使わずに周回し、36度ごとに機体カメラで撮影します。RVizで対象点群・視線・相対軌跡を表示します。KSPの通常の操作で機体を準備して実行します。

## ROS2モーター

追加パーツ:

- `ROS2 Size-0 Axial Servo`: 回転軸。可動範囲は-180〜180度、既定速度は45度/s、定格トルクは250 N·m
- `ROS2 Telescoping I-Beam Actuator`: M-Beam 200 Pocket型の直動軸。可動範囲は0〜1.6 mで、縮長約1.6 mから全長約3.2 mまで伸びる。既定速度は0.25 m/s、定格推力は4000 N

どちらも`bottom`側を親パーツへ、動かしたい構造物を`top`側へ取り付けます。モデルはベースKSPのFL-R20とM-Beam 200 I-Beam Pocket Editionを利用しているため、新規モデルファイルは不要です。

ROS2ブリッジを起動すると、KSPは状態をUDP 49010へ送り、ブリッジは指令をUDP 49011へ返します。モーター名を未設定にした場合は、KSPの`partFlightId`を使って`servo_<id>`または`linear_<id>`になります。

主なTopic:

- legacy subscribe（既定無効） `/ksp_vessel/actuators/servo/trajectory`: `trajectory_msgs/msg/JointTrajectory`
- publish `/ksp_vessel/joint_states`: `sensor_msgs/msg/JointState`
- publish `/ros2_ksp/diagnostics`: `diagnostic_msgs/msg/DiagnosticArray`

回転軸のposition/velocityはrad・rad/s、直動軸はm・m/sです。effortは回転軸がN·m、直動軸がNです。以下は`--enable-legacy-control`を付けた移行用の例です。新規コードではauthority付き`MotorCommand`を使います。

```bash
ros2 topic pub --once /ksp_vessel/actuators/servo/trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [servo_12345], points: [{positions: [1.5708], velocities: [0.5], effort: [100.0]}]}"
```

`JointTrajectory`に複数pointを指定した場合、`time_from_start`の時刻に順番にKSPへ送ります。新しいtrajectoryを受信すると、未送信の古いtrajectoryは置き換えます。positionを省略してvelocityだけを送ると速度モードになり、通信断時は0.5秒でその場停止します。

`/ksp_vessel/joint_states.effort`はKSPロボティクスジョイントのモーター出力から換算した推定トルク/推力です。`/ros2_ksp/diagnostics`の`estimated_current_a`は、パーツ設定の`torquePerAmpNm`または`forcePerAmpN`を使った推定値で、実測電流ではありません。

## ROS2推進系

Flight中のactive vesselにあるすべての`ModuleEngines` / `ModuleEnginesFX`と`ModuleRCS` / `ModuleRCSFX`を自動検出します。専用パーツへの差し替えは不要です。

主なTopic:

- publish `/ksp_vessel/actuators/propulsion/state`: `ksp_ros2_interfaces/msg/EngineState`
- subscribe `/ksp_vessel/actuators/propulsion/command`: `ksp_ros2_interfaces/msg/EngineCommand`
- legacy subscribe（既定無効） `/ksp_vessel/actuators/propulsion/main_throttle`: `std_msgs/msg/Float64`
- legacy subscribe（既定無効） `/ksp_vessel/actuators/rcs/twist_command`: `geometry_msgs/msg/Twist`

`state`には1モジュール1メッセージのJSONが10 Hzで流れます。`name`は`engine_<partFlightId>_<moduleIndex>`または`rcs_<partFlightId>_<moduleIndex>`です。ほかに`enabled`、`throttleLimit`、現在推力`thrust`、定格推力`maxThrust`、`flameout`、ROS制御中かを示す`commandActive`などが含まれます。

```bash
ros2 topic echo /ksp_vessel/actuators/propulsion/state
```

以下はbridgeへ`--enable-legacy-control`を付けた移行用APIの例です。所有権を調停できないため、新規コードではauthority付き`EngineCommand` / `RcsCommand`またはBody Wrenchを使ってください。legacy推力指令には通信断フェイルセーフがあるため、噴射中は`-r 5`などで0.5秒より短い間隔で継続送信します。

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/json_command std_msgs/msg/String \
  '{data: "{\"commands\":[{\"name\":\"engine_12345_0\",\"kind\":\"engine\",\"enabled\":true,\"throttle\":0.65}],\"timeout\":0.5}"}'
```

複数のエンジンやRCSブロックは`commands`配列にまとめて指定できます。`enabled`は起動・停止、`throttle`は0.0〜1.0のモジュール固有推力です。エンジンではKSPのindependent throttle、RCSではthrust limiterを使います。固体燃料エンジンなど絞れないものは状態の`throttleable`が`false`で、KSP側の物理制約が優先されます。

すべてのROS推力オーバーライドを解除し、元のKSP設定へ戻す例:

```bash
ros2 topic pub --once /ksp_vessel/actuators/propulsion/json_command std_msgs/msg/String \
  '{data: "{\"commands\":[{\"name\":\"*\",\"release\":true}]}"}'
```

メインスロットルは0.0〜1.0です。

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/propulsion/main_throttle std_msgs/msg/Float64 '{data: 0.8}'
```

RCSは`Twist.linear.{x,y,z}`をKSPのX/Y/Z並進入力へ、`Twist.angular.{x,y,z}`をpitch/yaw/rollへ対応させます。各値は-1.0〜1.0です。受信中だけRCSアクショングループも自動で有効になります。個々のノズルはKSPが機体姿勢と噴射方向から選択し、各RCSモジュールの最大推力は上記`propulsion/command`で個別設定できます。pitch/yaw/rollはKSP共通の操舵軸なので、同じ軸を使うリアクションホイールや舵面も反応します。

```bash
ros2 topic pub -r 5 /ksp_vessel/actuators/rcs/twist_command geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.2, z: 0.0}}'
```

メインスロットル、RCS 6軸入力、個別推力指令は既定0.5秒でタイムアウトし、推力または入力を0へ落とします。明示的な`release`時と操作機体の切替時には、ROS制御前のindependent throttle / RCS limiter設定を復元します。エンジンの再点火可否、停止可否、燃料切れ、ステージ条件はKSP標準の制約に従います。

## UDP JSON

送信されるJSONは1スキャン1パケットです。距離配列は垂直方向を外側、水平方向を内側にしたフラット配列です。未ヒットはデフォルトで`-1`です。

```json
{
  "type": "ksp_lidar_scan",
  "version": 1,
  "mode": "3D",
  "name": "front_lidar",
  "partName": "front_lidar",
  "lidarName": "front_lidar",
  "vessel": "Rover",
  "partFlightId": 12345,
  "universalTime": 42.0,
  "horizontalCount": 64,
  "verticalCount": 16,
  "horizontalFovDeg": 360.0,
  "verticalFovDeg": 30.0,
  "maxDistance": 2000.0,
  "layout": "vertical-major",
  "hitCount": 512,
  "ranges": [1.23, 1.25, -1.0],
  "hitMask": [1, 1, 0]
}
```
