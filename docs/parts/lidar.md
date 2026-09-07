# 2D / 3D LiDAR

2種類のLiDARは同じ`ModuleKerbalLidar`を使用し、`sensorMode`に応じて異なるROS2メッセージへ変換されます。どちらもROSセンサー座標の`+X`前方、`+Z`上方です。

## 入出力Topic

| パーツ | 入力Topic | 出力Topic | 型 |
|---|---|---|---|
| Kerbal LiDAR 2D | なし | `/ksp_vessel/lidar_2d/<lidar_2d_id>/scan` | `sensor_msgs/msg/LaserScan` |
| Kerbal LiDAR 3D | なし | `/ksp_vessel/lidar_3d/<lidar_3d_id>/points` | `sensor_msgs/msg/PointCloud2` |

## 2D LaserScan

- `angle_min`は水平FOVの`-1/2`です。
- 360度スキャンでは`angle_increment = FOV / horizontalCount`、360度未満では両端を含むため`FOV / (horizontalCount - 1)`です。
- `scan_time`は`1 / scanRateHz`、`time_increment`は`scan_time / horizontalCount`です。
- `range_min`は`0.0`、`range_max`はパーツの最大距離です。
- KSP側の未ヒット値`-1`、非有限値、範囲外値はROS側で`+Inf`になります。
- `intensities`は空配列です。

```bash
ros2 topic echo /ksp_vessel/lidar_2d/front_lidar/scan
```

## 3D PointCloud2

- `height = 1`のunorganized point cloudです。
- fieldは`x`、`y`、`z`の3つで、すべてlittle-endian `FLOAT32`です。
- `point_step = 12` byteです。
- 未ヒットは点群から除外され、`is_dense = true`になります。
- 現行の3DパーツはFibonacci半球配置を使います。KSPはrangeだけを送り、bridgeが既知の方向列を復元して点へ変換します。

```bash
ros2 topic echo --once /ksp_vessel/lidar_3d/roof_lidar/points
```

## 既定設定

| 設定 | 2D | 3D |
|---|---:|---:|
| スキャン周期 | 10 Hz | 10 Hz |
| 水平FOV | 360° | 360° |
| ray数 | 180 | Mediumで約2011 |
| 最大距離 | 2000 m | Mediumで100 m |
| UDP送信先 | `127.0.0.1:49010` | `127.0.0.1:49010` |
| 自船collision除外 | 有効 | 有効 |

3DはPart Action Windowから次のプロファイルを選択できます。

| Profile | 距離設定範囲 | 密度 | 既定距離 | おおよそのray数 |
|---|---:|---:|---:|---:|
| Near | 10〜30 m | 160 rays/sr | 30 m | 1005 |
| Medium | 50〜150 m | 320 rays/sr | 100 m | 2011 |
| Long | 150〜250 m | 1024 rays/sr | 250 m | 4096（ray上限） |

## Topic作成と削除

`lidarEnabled`と`udpEnabled`が有効な状態でFlightへ入ると送信を開始します。最初のスキャンを受けてTopicが作られ、Flight終了時のinactive通知または受信タイムアウトで削除されます。`streamAt20Fps`を有効にすると`scanRateHz`を無視して20 Hz固定になります。

各LiDARはVAB/SPHの`Edit ROS2 Sensor ID`で編集できる永続IDを持ちます。新規パーツには2Dなら`lidar_2d_<8桁UID>`、3Dなら`lidar_3d_<8桁UID>`が自動設定され、UDPの`sensorId`としてbridgeへ渡されます。

## Frame

active vesselモデルと`partFlightId`が対応した場合、`frame_id`は機体linkの子LiDAR frameです。モデルがない場合は次のfallbackになります。

```text
<frame_prefix>_<sensor_id>_lidar
```

既定の`frame_prefix`は`ros2_ksp`です。

## 実装確認先

- `GameData/KerbalLiDAR/Parts/Lidar2D/part.cfg`
- `GameData/KerbalLiDAR/Parts/Lidar3D/part.cfg`
- `Source/KerbalLiDAR/Api/Ksp/ModuleKerbalLidar.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/packet_conversion.py`
