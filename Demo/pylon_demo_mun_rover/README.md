# Mun rover / RViz Nav2 demo

ROS 2 Jazzy。前輪操舵の4輪・6輪ローバーを、3D LiDAR・IMU・車輪情報から推定して走らせます。RVizの **Nav2 Goal** は到着位置と向きを指定します。内部Nav2 actionは `/pylon/mun_rover/navigate_to_pose`、安全監視を通る公開actionは `/navigate_to_pose` です。直接内部actionへ指令しないでください。

## 起動

KSPの通常操作で下記の条件を満たす機体を月面に配置し、停止・接地を確認します。ビルドは`./sync.sh --demo mun_rover`です。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_mun_rover demo.launch.py lidar_sensor_id:=lidar_3d_189eb405
```

地面の白い領域、点群、車体外形が現れ、状態が `Ready: set Nav2 Goal` になってから矢印を置きます。経路を計画できる地面と旋回スペースが必要です。前進だけで向きを合わせるため、近いゴールでも大きく回り込む場合があります。最高速度は0.5 m/s、加速度は0.2 m/s²です。地図の灰色は未知で、走行させません。

`start_bridge:=false` で既存bridgeへ接続できます。`evaluate:=true` で真値の配信と独立した評価ノードを起動します。既定ではbridgeの真値配信自体を無効にします。推定・制御ノードはどちらのモードでも真値を購読しません。

## 機体の条件

- 左右対称の標準KSPホイール4輪または6輪。前の2輪だけ操舵可能、残りは直進固定。モーターとブレーキが使用可能で、十分な電源があること。
- 制御点は前方を向けること。Mk2 lander canでは **Control Point: Forward**。body frameはX前、Y左、Z上です。
- 3D LiDARは前方と地面が見える位置へ固定。幅の広いMun-Munでは低い取り付けだと地面の左右端が疎になり、未知領域による停止が起きます。初期試験は長距離・高密度プロファイル、約1.5 m高い位置を使用します。
- 起動時はMun上で車輪を接地させ、ブレーキを掛けたまま静止すること。IMUの重力方向と静止バイアスを30サンプル以上取得してから動作します。

## センサー処理と制約

車輪・IMUの予測を、3D点群のpoint-to-plane ICPと局所submapで補正します。平面上で観測できない移動方向はICPで補正せず、車輪から推定します。推定不確かさが0.6 mを超えると停止します。長距離SLAMや地図のループ閉じ込みは行いません。

地図は120 m四方、解像度0.25 m。観測された地面同士の辺長2 m以下の三角形だけを補間します。大きな欠測は未知のままです。レーザーの無反射を空き地と解釈しません。傾斜12度、段差0.25 m、凹凸0.15 mを初期の通行禁止基準とします。センサーの点間隔より小さい障害物の検出を保証するものではありません。

自車体で隠れる初期占有範囲だけは、静止した全車輪の接地を根拠に地面を初期化します。以後の道路はこの初期化で増やしません。車体寸法と0.5 mの余裕を含め、停止距離まで未知・障害物がないか毎回確認します。

地図・TFは真値ツリーと独立した `map → pylon_rover_odom → pylon_rover_base_footprint → pylon_rover_base_link` を使用します。センサーから機体への相対取り付け変換だけはbridgeのTFから読みます。

取消、指令やセンサーの0.5秒以上の欠測、接地喪失、制御権喪失、推定異常で停止し、古いゴールを自動再開しません。KSP側にも車輪指令0.25秒のwatchdogと駐車ブレーキがあります。異常後に再初期化するには、停止・取消後に:

```bash
ros2 service call /pylon/mun_rover/reset std_srvs/srv/Trigger '{}'
```

機体切替やテレポートでは独立heartbeatのruntime epochからlifecycle世代が変わり、地図と推定を破棄します。テレポート通知がUDPで1回落ちても次のパケットで検出します。

## 主なインターフェース

| 入出力 | Topic / action | 型 |
|---|---|---|
| 入力 | `/ksp_vessel/lidar_3d/<id>/points` | PointCloud2 |
| 入力 | `/ksp_vessel/imu/data_raw` | Imu |
| 入出力 | `/ksp_vessel/actuators/wheel/state`, `command` | WheelState / WheelCommand |
| 入力 | `/navigate_to_pose` | NavigateToPose action |
| 出力 | `/pylon/mun_rover/odom`, `odom_3d` | Odometry |
| 出力 | `/pylon/mun_rover/map` | OccupancyGrid |
| 出力 | `/pylon/mun_rover/points`, `ground`, `obstacles` | PointCloud2 |
| 出力 | `/pylon/mun_rover/plan`, `trajectory` | Path |
| 出力 | `/pylon/mun_rover/status`, `geometry`, `evaluation` | String (JSON) |

WheelStateにはSIの半径・機体座標内の位置・車体境界・回転符号・操舵符号と上限を追加しています。WheelCommandの`brake`は0〜1で、モーターenabledとは独立です。interfaces・bridge・Modは同時に更新してください。旧Modではgeometryが欠けるため自律走行を開始しません。

## 検証

```bash
PYTHONPATH=Ros2/pylon_perception:Demo/pylon_demo_mun_rover \
  python3 -m unittest discover -s Demo/pylon_demo_mun_rover/test -v
./sync.sh --demo mun_rover
```
