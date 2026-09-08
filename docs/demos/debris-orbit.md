# 軌道上のデブリ周回・撮影

3D LiDARとIMUで近くのデブリとの相対運動を推定し、RCSで周回しながら機体カメラで撮影します。KSPの通常操作で軌道投入・分離を済ませた状態から始めます。

## 1. デモをインストールする

[Getting Started](../guide/getting-started.md)でMODとbridgeを導入した後、PyLoNリポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  Ros2/pylon_interfaces Ros2/pylon_bridge Ros2/pylon_vehicle_control \
  Demo/pylon_demo_debris_orbit \
  --ignore-src --rosdistro jazzy -y
./sync.sh --skip-ksp-build --skip-ksp-sync --demo debris_orbit
source ~/ros2_ws/install/setup.bash
```

## 2. KSPで機体と対象を準備する

- 全6軸の並進・回転を操作できるRCSと、燃料・電源を用意します。
- 3D LiDARを搭載し、Sensor IDを`front_lidar`にします。
- LiDARと同じ方向を向くRGBカメラを搭載し、Sensor IDを`orbit_camera`にします。
- 通常操作で軌道投入とデブリの分離を行い、相対運動が小さい状態にします。対象は推力を出していない物体を使います。
- LiDARの測距範囲に対象を入れ、センサーとUDP配信を有効にします。

このデモは、近距離で共に自由落下する自機と対象を想定しています。周回半径の既定値は15 mです。機体とデブリの大きさに合わせ、衝突しない空間を確保してください。

## 3. bridgeを起動する

ターミナルAで実行します。bridgeはこの1プロセスを使います。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pylon_bridge udp_bridge \
  --host 127.0.0.1 --port 49010 --disable-ground-truth
```

別ターミナルで入力を確認します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --once /ksp_vessel/lifecycle
ros2 topic hz /ksp_vessel/lidar_3d/front_lidar/points
ros2 topic hz /ksp_vessel/imu/data_raw
ros2 topic hz /ksp_vessel/camera/orbit_camera/image_raw
```

`ros2 topic hz`は1つずつ実行し、確認したら`Ctrl-C`で終了します。機体状態がACTIVEで、点群・IMU・画像を受信できてから進みます。

## 4. 周回を開始する

ターミナルBで実行します。`enabled:=true`を指定すると機体の制御を開始します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_debris_orbit pylon_demo_debris_orbit.launch.py \
  enabled:=true demo_instance_id:=orbit_a \
  lidar_sensor_id:=front_lidar camera_sensor_id:=orbit_camera \
  orbit_radius:=15.0 rviz:=true
```

対象を見つけるまでは機体を回して探索します。点群から3回続けて対象を取得すると、対象への指向・接近を始め、指定半径へ到達後に周回します。RVizでは点群、対象クラスタ、対象中心、自機、目標半径、軌跡を確認できます。

推定と表示だけを試す場合は、上のコマンドの`enabled:=true`を`enabled:=false controller_enabled:=false`へ置き換えます。

## 5. 状態と撮影結果を確認する

```bash
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/status
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/estimator_status
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/controller_status
```

`demo_instance_id`を変更した場合は、Topic内の`orbit_a`も置き換えます。

周回開始位置を0度とし、36度ごとにPNGと計測JSONを保存します。保存先はデモを起動したディレクトリからの相対パスで、`pylon_demo_debris_orbit_captures/orbit_a/<起動日時>/`です。既定では2周目以降も撮影を続けます。

画像の時刻と推定角度を照合し、指定角度を通過してから既定2度以内の画像を保存します。`capture_missed`の場合は画像配信の周期と遅延を確認してください。

## 停止と再開

デモのターミナルで`Ctrl-C`を押すと、制御指令をゼロにしてleaseを解放します。終了後にbridgeも`Ctrl-C`で停止できます。

機体切替、制御権喪失、入力欠測後は停止を保持します。IMUが0.5秒を超えて途切れた場合も、機体を安定させてからデモのlaunch全体を起動し直してください。

## 主な起動引数

| 引数 | この手順の値 | 用途 |
|---|---|---|
| `enabled` | `true` | 周回制御を開始 |
| `demo_instance_id` | `orbit_a` | Topicと撮影ディレクトリの識別名 |
| `lidar_sensor_id` / `camera_sensor_id` | `front_lidar` / `orbit_camera` | 搭載センサーのID |
| `orbit_radius` | `15.0` | 周回半径［m］ |
| `rviz` | `true` | RVizを起動 |
| `controller_enabled` | `true`（既定） | 共通制御器を起動 |
| `config_file` | パッケージ同梱YAML | 推定・誘導・撮影設定を変更 |

設定ファイルはリポジトリの`Demo/pylon_demo_debris_orbit/config/pylon_demo_debris_orbit.yaml`です。機体への指令と所有権は[機体制御API](../api/vehicle-control.md)、画像の設定は[RGBカメラ](../parts/camera.md)を参照してください。
