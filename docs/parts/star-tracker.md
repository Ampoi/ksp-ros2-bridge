# スタートラッカー

`Kerbal ROS2 Star Tracker`は、宇宙空間で機体の絶対姿勢を測る表面取付パーツです。VAB/SPHのUtilityにあります。金色の筐体、黒い中空遮光筒、奥にある青いレンズ、4点取付フランジを持つ専用モデルです。外形は22 × 19 × 29.3 cm、質量8 kgです。

## 取り付けと起動

1. 機体の外側へ取り付け、遮光筒の開口を宇宙へ向けます。光軸はパーツのlocal −Z、取付面はz=0です。
2. 右クリックの`Edit ROS2 Sensor ID`で識別子を指定します。未指定なら`star_tracker_...`が自動生成されます。
3. ElectricChargeを供給します。既定の消費は0.05 EC/sです。
4. 大気圏外で太陽・天体の縁・機体構造を避け、回転を2°/s以下にすると、約2秒で`tracking`になります。
5. 既存のUDP bridgeを起動します。新しいパーツとDLLの読み込みにはKSPの再起動が必要です。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge
# 別ターミナルで、実際のIDへ置き換える
ros2 topic echo /ksp_vessel/star_tracker/star_tracker_example/state
ros2 topic echo /ksp_vessel/star_tracker/star_tracker_example/attitude
```

## Topic

| Topic | 型 | 配信条件 |
|---|---|---|
| `/ksp_vessel/star_tracker/<id>/state` | `ksp_ros2_interfaces/msg/StarTrackerState` | 正常・測定不能とも既定5 Hz。姿勢、valid、理由、共分散、機体・センサーIDを同時配信 |
| `/ksp_vessel/star_tracker/<id>/attitude` | `geometry_msgs/msg/QuaternionStamped` | `valid=true`の測定だけ |

両方Reliable / Volatile / depth 10です。`--topic-prefix`に追従し、`--disable-ground-truth`でも利用できます。推定器は`state`を購読し、`valid`とtimestampを確認してください。`attitude`単独には無効通知が含まれません。

`state.valid=false`ではquaternionは全要素0、`orientation_covariance[0]=-1`です。これを姿勢として利用・正規化してはいけません。最後の正常値やidentity quaternionを測定値として再配信しません。位置推定は提供しません。

### 座標と精度

- `header.frame_id=kerbol_inertial`：KSPの固定天球基準。Unity world vectorを`Planetarium.right / forward / up`に射影した右手座標です。J2000や地球のICRFとは別物です。
- `measured_frame_id=base_link`：機体のX前方・Y左・Z上。quaternionはこの機体座標から慣性座標への回転です。取付パーツ自身の姿勢ではありません。
- パーツの取付角度は観測可能性に影響します。姿勢出力は機体へ校正済みとして扱います。
- 標準偏差は各軸20 arcsec、共分散は機体軸の微小回転誤差について`σ² I`（rad²）です。小角度の独立Gaussianノイズを加えます。
- timestampは既存bridgeのシミュレーション時刻写像を使用します。`universal_time`にはKSP UTを残します。
- 位置が分からないため、`kerbol_inertial`と既存のENU `world`の間のTFやPoseは生成しません。

### 測定不能の理由

| reason | 意味 |
|---|---|
| `tracking` | 有効な姿勢測定 |
| `acquiring` | 初期捕捉または再捕捉中 |
| `disabled` | パーツのスイッチがOff |
| `no_power` | ElectricCharge不足 |
| `atmosphere` | 大気内。このモデルは宇宙用として大気内を一律無効化 |
| `sun_exclusion` | 太陽円盤＋除外角内へ光軸が向いている |
| `body_in_fov` | 天体円盤＋半FOV＋縁の余裕角が光軸と重なる |
| `occluded` | 機体、フェアリング、他機体、地形による視野遮蔽 |
| `slew_rate_exceeded` | 光学ヘッドの慣性回転速度が上限超過、または速度測定の準備中 |
| `packed` | KSPの物理演算対象外 |
| `sensor_unavailable` / `invalid_time` | 光軸・基準情報・時刻が利用不能 |
| `inactive` | 操作機体でなくなった、またはパーツの終了通知 |
| `stale` | bridgeが`--topic-timeout-sec`の通信断を検出 |

障害が解消しても連続2秒の捕捉を必要とします。UT巻き戻りと2秒を超えるサンプル間隔でも捕捉をやり直します。ポーズ中は新しい測定を出さず、bridge側ではtimeoutで`stale`になります。通信断後は4 Hzの無効heartbeatを出し、最後の受信から`max(30秒, timeout×3)`後にTopicを削除します。終了・packed通知がゲームから届かない場合もtimeoutで無効化します。bridge自体の停止も検出できるよう、購読側でもtimestampの期限を確認してください。

同じセッション内の重複・逆順UDPはsequenceで破棄します。センサーごとに独立した状態を持ちます。

## シミュレーションの範囲

星画像をレンダリングして星表と照合するアルゴリズムではありません。KSPが持つ機体姿勢を、観測条件と測定誤差を含むセンサーモデルへ通します。実星数、星の明るさ、放射線、雲、詳細な迷光、光学歪み、温度による校正変化は再現しません。

天体遮蔽は球としての見かけ角で評価します。機体遮蔽は開口から2 kmまでの中心＋2リングの17本のレイで評価し、細い構造を見逃す場合があります。自パーツの筐体コライダーは除外します。太陽が他天体に隠れている場合も除外角を適用し、大気内も夜昼によらず無効にする保守的なモデルです。

実機が星配置から姿勢を求めること、太陽除外角や角速度の制約を持つことを参考にしています。既定値はゲーム用の設定で特定製品の保証値ではありません。[ESAの姿勢・太陽除外角の説明](https://resilience.esa.int/archives/projects/alphasat-tdp6-feasibility-study-star-tracker)、[NASAの星による姿勢決定の説明](https://www.nasa.gov/missions/lasers-stars-and-sensors-will-guide-nasas-orion-spacecraft/)。

## part.cfg

| キー | 既定値 | 内容 |
|---|---:|---|
| `sampleRateHz` | 5 | 配信周波数（wall-time上限、1～20 Hz） |
| `fieldOfViewDegrees` | 20 | 円形視野の全角（5～60°） |
| `sunExclusionDegrees` | 35 | 太陽円盤からの除外角 |
| `bodyLimbMarginDegrees` | 5 | 天体の縁からの余裕角 |
| `maxAngularRateDegrees` | 2 | 光学ヘッドの回転速度上限（°/s） |
| `acquisitionSeconds` | 2 | 連続clearを要求するシミュレーション秒数 |
| `noiseArcsec` | 20 | 各軸の姿勢誤差標準偏差 |
| `electricChargePerSecond` | 0.05 | 消費EC/s |
| `udpHost` / `udpPort` | `127.0.0.1` / `49010` | 既存bridgeへの送信先 |
