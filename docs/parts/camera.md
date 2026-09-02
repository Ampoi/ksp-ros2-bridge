# RGBカメラ

`Kerbal ROS2 RGB Camera`はFlight描画をraw RGBフレームとして取得します。KSPからはチェックサム付きUDPチャンクで送り、bridgeは全チャンクが揃ってSHA-256検証に成功したフレームだけをpublishします。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 入力 | なし | — | ROS2からのカメラ制御Topicはありません |
| 出力 | `/ros2_ksp/<part_name>/camera/image_raw` | `sensor_msgs/msg/Image` | raw RGB画像 |
| 出力 | `/ros2_ksp/<part_name>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 内部パラメーター |

## Image

| Field | 値 |
|---|---|
| `encoding` | `rgb8` |
| `is_bigendian` | `false` |
| `step` | `width * 3` |
| 行順 | 上端始まり |
| `header` | CameraInfoと同じstamp / frame_id |

```bash
ros2 topic hz /ros2_ksp/rgb_camera/camera/image_raw
ros2 run rqt_image_view rqt_image_view /ros2_ksp/rgb_camera/camera/image_raw
```

## CameraInfo

歪みのないpinhole cameraとして生成します。

- `distortion_model = "plumb_bob"`
- `d = [0, 0, 0, 0, 0]`
- focal lengthは垂直FOVと画像高さから`height / (2 * tan(vertical_fov / 2))`で算出
- principal pointは`((width - 1) / 2, (height - 1) / 2)`
- stereo baselineは0

```bash
ros2 topic echo --once /ros2_ksp/rgb_camera/camera/camera_info
```

## 既定設定と操作

| 設定 | 既定値 | 実装上の範囲 |
|---|---:|---:|
| 解像度 | 320 × 240 | 内部clampは幅16〜1280、高さ16〜720 |
| frame rate | 5 Hz | 1〜15 Hz |
| 垂直FOV | 60° | 20〜120° |
| near clip | 0.05 m | 0.01〜10 m |
| far clip | 20000 m | near + 1 m以上 |
| 1チャンク | 12000 byte | 512〜36000 byte |
| 最大frame | 4 MiB | 1 KiB〜16 MiB |

Part Action Windowには160 × 120、320 × 240、640 × 480の解像度presetがあり、frame rateとFOVも変更できます。

## Frame

camera frameはREP-103 optical規約です。

- `+X`: 画像の右
- `+Y`: 画像の下
- `+Z`: カメラ前方

active vesselモデルへ接続できる場合は対応linkの子`camera_optical_frame`になります。モデルがない場合のfallbackは次です。

```text
<frame_prefix>_<part_name>_camera_optical_frame
```

## 欠落フレーム

1チャンクでも欠けたフレームはpublishしません。未完成のframe assemblyは2秒で破棄されます。センサーTopic自体は、完全なフレームの最終publishから`--topic-timeout-sec`経過後に削除されます。

## 実装確認先

- `GameData/KerbalLiDAR/Parts/RgbCamera/part.cfg`
- `Source/KerbalLiDAR/ModuleKerbalRgbCamera.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/camera_packets.py`
