# pylon_demo_debris_orbit: LiDAR＋IMU周回デモ

機体の準備から停止までの手順は、[デモガイド](../../docs/demos/debris-orbit.md)を参照してください。

3D LiDARと6軸IMUだけでデブリへの相対位置・相対速度・姿勢変化を推定し、LiDARを向けながらRCSで周回します。**推定器・誘導器・制御器のすべてでGround Truthを購読しません。** 機体カメラで36度ごとに撮影し、次の周回も撮影を続けます。

```text
3D点群 ─ SciPyクラスタリング ─┐
                              ├─ 相対運動Kalman filter ─ RelativeTarget ─ 周回誘導
IMU ─ ジャイロ積分・比力予測 ─┘               │                         │
                                         推定Pose/Twist ─ 共通制御器 ─ RCS
                                                                    │
                                   機体カメラ ─ 36度ごとにPNG＋計測JSON
```

`pylon_debris_target_estimator`が推定、`pylon_demo_debris_orbit`が周回誘導・撮影、`pylon_vehicle_control`がlease付きのBody Wrenchを担当します。制御器の入力には、このデモの推定Pose/Twistを明示的に接続します。旧`target_source:=truth`/`lidar`は廃止し、`lidar_imu`が唯一の入力方式です。

## 実機の準備と起動

全6軸を操作できるRCS、3D LiDAR（Sensor ID `front_lidar`）、LiDARと同方向を向くカメラ（`orbit_camera`）を搭載します。センサー取付位置・姿勢は、機体内のTFから取得します。

[起動手順](../../docs/guide/getting-started.md)に従ってビルド・同期し、KSPの通常操作で機体を開いてください。

Flightが開いたら別ターミナルでbridgeを起動します。`--disable-ground-truth`は真値パケットを破棄し、真値Topicとworld TFを配信しません。機体ID・lifecycleもIMUパケットから取得します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pylon_bridge udp_bridge \
  --host 127.0.0.1 --port 49010 --disable-ground-truth
```

KSPの通常操作で軌道投入・分離を行い、機体の状態が安定したら別ターミナルでデモを起動します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_debris_orbit pylon_demo_debris_orbit.launch.py \
  enabled:=true demo_instance_id:=test_a_imu rviz:=true
```

未発見時はIMUで姿勢を追跡しつつ後方まで探索します。点群から3回続けて対象を取得したら指向・接近を開始し、半径15 mへ到達後に周回します。大きな姿勢誤差がある間は相対速度を制動します。`Ctrl-C`で制御指令をゼロにしてleaseを解放します。

推定だけを表示するには`controller_enabled:=false`を指定し、`enabled:=true`を付けずに起動します。独自推定器をつなぐ場合は`estimator_enabled:=false`とし、同じtarget・navigation Topicを配信します。

## 座標系と推定の範囲

IMUには絶対姿勢がありません。初回の機体姿勢を単位quaternionとし、SciPyのRotationでbody角速度を積分します。`pylon_debris_inertial_<demo_instance_id>`の軸はこの初期姿勢に固定され、ENUや惑星の絶対座標とは一致しません。

LiDARの観測面の包囲箱中心を、取付TFとIMU姿勢でこの座標系へ変換します。NumPyで間引き、[SciPy cKDTree](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html)と疎行列の連結成分でクラスタを抽出します。比力を積分してスキャン間の相対運動を予測し、点群観測で位置・速度を補正します。

近距離で共に自由落下する、推力を出していない対象を仮定します。自機の比力は対象−自機の相対加速度に負符号で入り、共通の重力加速度を足す必要はありません。重力勾配や対象自身の加速度はモデル誤差として残ります。地上用の重力方向推定を自由落下中へ流用していません。

制御用の原点は推定した対象中心です。自機位置は`−relative_position`、自機速度は`−relative_velocity`です。絶対位置・絶対速度・絶対方位は求めません。見える表面の中心と実際の重心には偏差があり、6軸IMUのジャイロバイアスによる長時間の方位ドリフトも完全には観測できません。現在のModはIMUに人工ノイズ・バイアスを加えていません。

0.5秒を超えるIMUサンプルの欠落では`imu_restart_required`となり、古い状態での制御を止めます。この場合はlaunch全体を再起動して基準座標系を揃えます。機体切替・lifecycle更新でも推定をリセットします。

## 36度ごとの撮影

周回開始位置を0度として、**0、36、72、…、324、360、396、…度**で搭載カメラの画像を保存します。`stop_after_capture: false`が既定なので2周目以降も続きます。

保存先は`pylon_demo_debris_orbit_captures/<demo_instance_id>/<起動日時>/`です。画像名に角度と連番を付けるため、次周の同じ角度や再起動で上書きしません。PNGごとのJSONに、画像timestamp・カメラframe・指定角度・画像取得時点の推定周回角を記録します。

画像timestampを推定角の履歴へ照合し、指定角度を通過した後、既定2度以内のフレームを保存します。カメラ遅延や未配信で間に合わない場合は`capture_missed`を報告し、古い画像で埋めません。カメラのフレーム周期による角度誤差はJSONで確認できます。

## RVizと設定

`rviz:=true`で点群、選択クラスタ、対象中心、自機、視線、目標半径と軌跡を表示します。表示用の`pylon_debris_view_<demo_instance_id>`は推定対象中心が原点です。軌跡は2 Hzで最大30分保持します。

設定は[`config/pylon_demo_debris_orbit.yaml`](config/pylon_demo_debris_orbit.yaml)。主なlaunch引数は`orbit_radius`、`demo_instance_id`、`lidar_sensor_id`、`camera_sensor_id`、`lidar_frame`、`config_file`です。YAMLには`imu_topic`、`imu_max_gap_sec`、点群フィルタと追跡雑音、RCSゲイン、撮影間隔・許容角があります。

Topicルートは`/ksp_vessel/demos/debris_orbit/<demo_instance_id>`:

| Topic末尾 | 内容 |
|---|---|
| `target` | `RelativeTarget`: 対象−自機の相対位置・速度、初期IMU姿勢に固定した軸 |
| `navigation/pose` | 対象基準の自機位置とIMU姿勢 |
| `navigation/twist` | 同座標系での自機相対速度と角速度 |
| `navigation/twist_body` | body軸での推定速度とIMU角速度 |
| `setpoint` | 共通制御器への位置・姿勢・速度目標 |
| `status`, `estimator_status`, `controller_status` | JSON状態 |
| `points`, `target_points`, `markers`, `path` | RViz表示 |

センサー時刻はKSP universal timeに一定offsetを加えた連続時刻です。遅れた画像やゲームの処理速度に合わせて飛行中にoffsetを飛ばすことはありません。nodeの受信鮮度判定には別途ROS時計を使います。

## テスト

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH="Demo/pylon_demo_debris_orbit:Ros2/pylon_vehicle_control:Ros2/pylon_bridge:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s Demo/pylon_demo_debris_orbit/test -v
```

ROS結合テストは独立したdomainで動作し、試験中のKSPへ指令を送りません。

一度動作を開始した後にセッション変更・姿勢や速度の欠測・制御権喪失が起きると停止を保持します。通信復帰で周回目標を再開せず、再開にはデモを起動し直してください。
