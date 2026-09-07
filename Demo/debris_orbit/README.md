# debris_orbit デモ

デブリに3D LiDARを向けたままRCSで周回するROS 2 Jazzyデモです。位置推定と移動制御を分離しており、既定は**真値による相対位置**で制御します。同じ制御へLiDAR推定を接続できます。

```text
位置推定パート                          移動制御パート
truth: 自機・デブリの同時刻の絶対位置 ─┐
                                     ├─ RelativeTarget ─ 周回誘導 ─ ControlSetpoint
lidar: 3D点群 → SciPyクラスタ → 追跡 ─┘                         ↓
                                                   ksp_vehicle_control
                                                     lease → Body Wrench → RCS
```

`debris_target_estimator`は位置推定だけを担当し、制御指令を出しません。`debris_orbit`は点群を購読せず、共通の相対位置・相対速度から目標位置・姿勢・速度を作ります。`debris_orbit_controller`は既存の`ksp_vehicle_control`を使う共通制御器です。

## 実機の準備と起動

1. 機体に全6軸を制御できるRCS、3D LiDAR、必要ならRGBカメラを搭載します。
2. Sensor IDをLiDARは`front_lidar`、カメラは`orbit_camera`にします。LiDARの取付位置と姿勢はTFから取得します。カメラも正対させるなら光軸をLiDARと揃えます。
3. 分離後の対象がLiDARの250 m範囲内にある状態で実行します。画像保存はカメラなしでも周回制御を妨げません。

以下は`ROS2 debug`の`test A`を使う手順です。`dev_debug.sh`は元のセーブを保持し、隔離デバッグセーブを作ります。

```bash
# リポジトリ直下
./dev_sync.sh
./Development/commands/dev_debug.sh \
  --save "ROS2 debug" --vessel "test A" --launch-craft \
  --lidar-profile long --no-teleport --keep-session
```

Flightが開いたら別ターミナルでbridgeを起動します。Mod変更後はKSPを再起動してください。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

さらに別ターミナルで低軌道へ移し、真値モードを起動します。

```bash
./Development/commands/dev_teleport.sh lko
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch debris_orbit debris_orbit.launch.py \
  target_source:=truth enabled:=true demo_instance_id:=test_a_run rviz:=true
```

対象がない間は`waiting_for_truth_target`で待ちます。別ターミナルから分離します。

```bash
./Development/commands/dev_separate.sh
```

真値モードは、同一時刻・同一原点の絶対位置／速度を引き算し、最も近いデブリを初回に選択してそのvessel IDへ固定します。対象が消えても別の物体へ勝手に乗り換えません。指定したい場合は`target_vessel_id:=<vessel_id>`を追加します。候補一覧は次で確認できます。

```bash
ros2 topic echo /ksp_vessel/ground_truth/nearby_vessels --once
```

`demo_instance_id`は状態・画像の名前空間です。操作するKSP機体はlifecycleのactive vesselで決まります。

## LiDAR推定への切替

真値モードのターミナルで`Ctrl-C`を押して止め、デブリをLiDAR正面に捉えている間に次を起動します。

```bash
ros2 launch debris_orbit debris_orbit.launch.py \
  target_source:=lidar enabled:=true demo_instance_id:=test_a_lidar rviz:=true
```

推定だけを検証する場合は、制御器を起動しません。

```bash
ros2 launch debris_orbit debris_orbit.launch.py \
  target_source:=lidar controller_enabled:=false rviz:=true
```

LiDARモードは`nearby_vessels`を購読しません。NumPyによる間引き、[SciPy cKDTree](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html)による近傍検索、SciPyの疎行列連結成分によるクラスタリングを使います。前方35度以内の点から大きすぎる面を除き、初回は最大クラスタ、その後は予測位置に近いクラスタを選択します。観測表面の包囲箱中心を6状態のKalman filterで追跡し、相対位置と相対速度を配信します。

推定に使う自機情報は姿勢・角速度とLiDAR取付TFです。**デブリの位置・速度の真値は推定に使いません。** 現在の姿勢基準と共通制御器の自機状態にはGround Truthを使用します。LiDARだけによる全6DoF自己位置推定・SLAMではありません。別デモ`position_estimator`は静止環境のscan-to-scan ICP用で、動くデブリの中心推定とは役割が異なります。

見えている表面の中心と実際の重心には差があります。`target_center_offset`で視線奥への補正を設定できますが、対象形状・見える面・点群密度に依存するため、一定補正で重心が正確に求まるわけではありません。ほぼ対称な物体の姿勢や見えない形状も復元しません。

## RViz表示

`rviz:=true`で付属設定を読み込み、Fixed Frameを`debris_view_<demo_instance_id>`へ自動設定します。真値モードでも相対位置と軌跡を表示できます。

- 水色の点: 選択したデブリのクラスタ。灰色の点: LiDARの観測点。
- 水色の球: 推定対象中心。オレンジの球・線: 自機位置と相対軌跡。
- 灰色の円: 目標周回半径。直線: 対象への視線。

表示は推定した対象中心を原点とする独立した座標系です。点群・機体・軌跡を同じ座標系へ移しているため、低軌道の大きな絶対座標に流されず観察できます。`ground_truth_enu -> base_link`とはTFを競合させません。

## 制御動作

既定半径15 m、角速度1 deg/s。まずLiDARを対象へ向けて円周へ接近し、到達後は現在の実測方位から円周方向の速度を与えます。時間だけで先へ進む軌道ではないので、接近や指向の遅れで目標が先走りません。近接・周回ともLiDAR光軸を制御し、視線変化の角速度も先行指令します。

- 指向誤差が8度を超える間は円周運動を保留し、相対速度を減速させます。
- 接近位置ステップは2 mに制限し、遠方への大きな指令による行き過ぎを抑えます。
- 相対位置のみを自機状態の時刻まで予測します。制御器もsetpointを自機poseの時刻へ整合し、公転速度×通信遅延による偽の位置誤差を防ぎます。
- 過大な角速度では`detumbling`へ移ります。入力喪失、原点変更、機体変更は追跡と軌道をリセットします。取付TFがない間はIDLEです。
- 対象を失ったLiDARモードは最後の視線を中心に往復探索します。観測が継続して3回届くまで周回を再開しません。
- 0, 30, …, 330度の画像を12枚保存し、既定では撮影後も周回を続けます。`stop_after_capture: true`なら撮影が揃い、実測角が360度へ達した後に停止します。

`Ctrl-C`で停止すると共通制御器がゼロWrenchを送りleaseを解放します。KSP側の角速度・指令timeout・連続噴射の制限も有効です。

## 設定とインターフェース

[`config/debris_orbit.yaml`](config/debris_orbit.yaml)の`debris_target_estimator`が位置推定、`debris_orbit`が誘導、`debris_orbit_controller`が共通制御の設定です。独自YAMLは`config_file:=/absolute/path/config.yaml`で指定します。

| 設定 | 用途 |
|---|---|
| `target_source:=truth\|lidar` | 位置推定を切替 |
| `target_vessel_id` | 真値モードの対象固定。空なら最寄りデブリ |
| `orbit_radius:=15.0` | 誘導とRViz両方の半径を上書き |
| `lidar_sensor_id`, `camera_sensor_id` | KSP上のSensor ID |
| `lidar_frame` | 誘導用の取付TF名。bridgeのframe-prefix変更時に指定 |
| `target_topic` | 推定・誘導の共通入力Topic |
| `estimator_enabled:=false` | 外部のRelativeTarget publisherに接続 |
| `controller_enabled:=false` | 認識・誘導だけを起動 |
| `max_off_axis_deg`, `max_cluster_extent` | LiDAR候補の前方角度・最大対角長[m] |
| `measurement_sigma`, `acceleration_sigma` | 相対追跡filterの観測・加速度雑音 |
| `arrival_*_tolerance` | 周回開始の位置・相対速度・姿勢許容値 |

共通Topicルートは`/ksp_vessel/demos/debris_orbit/<demo_instance_id>`です。

| Topic末尾 | 型・意味 |
|---|---|
| `target` | `RelativeTarget`: デブリ−自機の位置[m]・速度[m/s]、world軸表現 |
| `setpoint` | `ControlSetpoint`: 共通制御器への目標 |
| `status`, `estimator_status`, `controller_status` | JSON状態 |
| `points`, `target_points` | RViz用PointCloud2（LiDARモード） |
| `markers`, `path` | RViz用MarkerArray、Path |

`RelativeTarget.header.stamp`は観測時刻、`header.frame_id`は`ground_truth_enu`です。位置の原点は自機ですが、軸は回転する`base_link`ではなくworld軸です。`observer_vessel_id`と`origin_sequence`がlifecycleと一致する必要があります。両sourceを同じTopicへ同時に配信しないでください。

真値入力`/ksp_vessel/ground_truth/nearby_vessels`は、同じ天体のロード済み・unpacked・2500 m以内の他機体を最大32件配信します。デモ側は既定で250 m以内に絞ります。

## 実機記録とテスト

実際のKSPで真値モード、続いてLiDARモードを操作・計測した結果は[実機試験記録](../../Development/evidence/debris_orbit/README.md)に保存しています。

評価用ノードは制御・推定と独立し、真値との差、実測周回角、真のデブリへの光軸誤差を保存します。対象候補が複数なら`--target-id`も指定してください。

```bash
python3 Development/tools/record_debris_orbit.py \
  --instance test_a_lidar --output /tmp/debris_trial --stop-after-turns 1
python3 Development/tools/plot_debris_orbit.py /tmp/debris_trial
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH="Demo/debris_orbit:Ros2/ksp_vehicle_control:Ros2/ksp_lidar_bridge:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s Demo/debris_orbit/test -v
```

ROS callbackテストは独立したROS_DOMAIN_ID 173で動き、試験中のKSPへ指令を送りません。
