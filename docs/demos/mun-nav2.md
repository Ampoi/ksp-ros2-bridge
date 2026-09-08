# 月面Nav2

前輪操舵のローバーをMunに配置し、RVizのNav2 Goalで指定した位置と向きへ走行させます。3D LiDAR、IMU、車輪情報から自己位置と通行可能な地面を推定します。

## 1. デモをインストールする

[Getting Started](../guide/getting-started.md)でMODとbridgeを導入した後、PyLoNリポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  Ros2/pylon_interfaces Ros2/pylon_bridge Ros2/pylon_vehicle_control \
  Ros2/pylon_perception Demo/pylon_demo_mun_rover \
  --ignore-src --rosdistro jazzy -y
./sync.sh --skip-ksp-build --skip-ksp-sync --demo mun_rover
source ~/ros2_ws/install/setup.bash
```

Nav2、RViz、点群処理に必要な依存パッケージも導入されます。

## 2. ローバーを準備する

1. 左右対称にKSP標準ホイールを4輪または6輪配置します。前の2輪だけ操舵を有効にし、残りは直進固定にします。
2. 各ホイールのモーターとブレーキを使用できる状態にし、十分な電源を用意します。
3. 制御点を前方へ向けます。Mk2 lander canでは`Control Point: Forward`を選びます。
4. 3D LiDARを前方と地面が見える位置へ固定し、Sensor IDを`front_lidar`にします。幅の広い機体ではLiDARを高めに配置し、左右の地面も視野に入れます。
5. KSPの通常操作でMunへ配置し、全車輪の接地を確認してブレーキを掛け、静止します。

起動時にIMUの重力方向と静止バイアスを30サンプル以上取得します。静止した状態で初期化を待ってください。

## 3. Nav2とRVizを起動する

手動で起動したbridgeがあれば終了してから、次を実行します。このlaunchはbridgeも起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_mun_rover demo.launch.py \
  lidar_sensor_id:=front_lidar
```

既存bridgeを使う場合は`start_bridge:=false`を付けます。bridgeは同時に1つだけ起動してください。

## 4. ゴールを指定する

RVizに点群、車体外形、白い地面領域が表示され、状態が`Ready: set Nav2 Goal`になるまで待ちます。

1. RVizの`Nav2 Goal`を選択します。
2. 白い通行可能領域を押し、到着時に向けたい方向へドラッグして矢印を置きます。
3. 表示された経路と車体の動きを確認します。

灰色は未観測領域で、走行対象になりません。最初は観測済みの近い地点を選び、旋回する空間も確保してください。前進で向きを合わせるため、近いゴールでも回り込む場合があります。最高速度は0.5 m/s、加速度は0.2 m/s²です。

自分のノードからゴールを送る場合は、公開actionの`/navigate_to_pose`を使います。`/pylon/mun_rover/navigate_to_pose`は内部用です。

## 5. 状態と地図を確認する

別ターミナルでROS環境を読み込んでから実行します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /pylon/mun_rover/status
```

| 表示・出力 | Topic |
|---|---|
| 地図 | `/pylon/mun_rover/map` |
| 自己位置 | `/pylon/mun_rover/odom`、`odom_3d` |
| 点群・地面・障害物 | `/pylon/mun_rover/points`、`ground`、`obstacles` |
| 計画経路・走行軌跡 | `/pylon/mun_rover/plan`、`trajectory` |
| 機体形状 | `/pylon/mun_rover/geometry` |

地図の範囲は120 m四方、解像度は0.25 mです。TFは`map → pylon_rover_odom → pylon_rover_base_footprint → pylon_rover_base_link`を使います。同梱RViz設定ではこの推定座標系の地図と軌跡を表示します。

## 停止と再初期化

走行中のゴールを取り消すと停止します。指令・センサーの0.5秒以上の欠測、接地喪失、制御権喪失、推定異常でも停止し、古いゴールは自動再開しません。

異常の原因を解消し、停止・ゴール取消・接地を確認してから初期化します。

```bash
ros2 service call /pylon/mun_rover/reset std_srvs/srv/Trigger '{}'
```

再びReadyになったら新しいゴールを指定します。終了する場合はゴールを取り消して停止を確認し、launchのターミナルで`Ctrl-C`を押します。

## 主な起動引数

| 引数 | 既定値 | 用途 |
|---|---|---|
| `lidar_sensor_id` | `front_lidar` | 3D LiDARのSensor ID |
| `start_bridge` | `true` | bridgeも起動 |
| `use_rviz` | `true` | RVizを起動 |
| `evaluate` | `false` | Ground Truthと比較する評価ノードを起動 |
| `params_file` | 同梱`config/nav2.yaml` | Nav2の設定 |
| `bt_xml` | 同梱`config/navigate.xml` | ナビゲーションの動作設定 |

真値との比較を行う場合は`evaluate:=true`を付け、`/pylon/mun_rover/evaluation`を確認します。評価は独立したノードで行います。既存bridgeを使う場合は、そちらでもGround Truth配信を有効にしてください。

## 走り出さない・途中で止まる場合

| 状態 | 確認すること |
|---|---|
| Readyにならない | Mun上で全車輪が接地し、静止しているか。前輪のみ操舵できるか |
| 地面が灰色のまま | LiDARが前方と左右の地面を観測できているか。取付高さ・角度・距離プロファイルを調整 |
| 経路が作られない | ゴールと経路が観測済みで、車体と旋回の余裕があるか |
| 推定異常で停止 | 地形の特徴、車輪の接地、点群・IMUの受信周期を確認し、停止後にreset |

長距離のループ閉じ込みを行うSLAMではなく、局所地図を使うデモです。通行判定の初期基準は傾斜12度、段差0.25 m、凹凸0.15 mです。車体と0.5 mの余裕を含む停止距離内に未知領域・障害物がある場合も停止します。
