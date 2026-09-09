# RGBカメラ

`PyLoN RGB Camera`は、カメラパーツの視点からFlightシーンを撮影し、RGB画像と内部パラメーターをROS2へ配信します。画像には銀河背景、地形、雲・大気の描画が含まれ、画面UIは含みません。

撮影方向、解像度、フレームレート、FOVはパーツの設定で指定します。撮影負荷はセンサー設定に加え、ゲーム画面の解像度と描画MODにも依存します。EVE・Scattererの描画に対応し、センサー画像にはScattererのTAAを適用しません。他の描画MODによって画像の見え方が異なる場合があります。

既定の`useGameClipPlanes = true`ではゲームと同じ近距離／遠距離の描画範囲を使います。`false`にすると、`nearClipMeters` / `farClipMeters`でローカル描画範囲を制限できます。Scaled Spaceと銀河背景はこの制限の対象外です。

## 入出力Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 入力 | なし | — | ROS2からのカメラ制御Topicはありません |
| 出力 | `/ksp_vessel/camera/<camera_id>/image_raw` | `sensor_msgs/msg/Image` | raw RGB画像 |
| 出力 | `/ksp_vessel/camera/<camera_id>/camera_info` | `sensor_msgs/msg/CameraInfo` | 内部パラメーター |

## Image

| Field | 値 |
|---|---|
| `encoding` | `rgb8` |
| `is_bigendian` | `false` |
| `step` | `width * 3` |
| 行順 | 上端始まり |
| `header` | CameraInfoと同じstamp / frame_id |

```bash
ros2 topic hz /ksp_vessel/camera/orbit_camera/image_raw
ros2 run rqt_image_view rqt_image_view /ksp_vessel/camera/orbit_camera/image_raw
```

## CameraInfo

歪みのないpinhole cameraとして生成します。

- `distortion_model = "plumb_bob"`
- `d = [0, 0, 0, 0, 0]`
- focal lengthは垂直FOVと画像高さから`height / (2 * tan(vertical_fov / 2))`で算出
- principal pointは`((width - 1) / 2, (height - 1) / 2)`
- stereo baselineは0

```bash
ros2 topic echo --once /ksp_vessel/camera/orbit_camera/camera_info
```

## 既定設定と操作

モデルは専用のオンボードカメラです。取付台と左右の支柱は固定され、カメラ本体とレンズだけが上下に回転します。
VAB/SPHとFlightの右クリックメニューにある`Camera Tilt`で0〜180°を設定できます。
角度はcraft／セーブに保存され、`part.cfg`内の`ModulePyLoNRgbCamera`の`cameraTiltDegrees`で新規パーツの初期角度を指定できます。

```cfg
cameraTiltDegrees = 90
cameraPivotTransformName = CameraPivot
cameraOpticalTransformName = CameraOptical
```

| 角度 | レンズの方向（パーツローカル座標） |
|---|---|
| 0° | +Y：取付面に沿って上向き |
| 90°（既定） | −Z：取付面から外向き |
| 180° | −Y：取付面に沿って下向き |

「上／下」はパーツの取付姿勢に対する方向です。パーツ全体を回して取り付けると首振りの方向も変わります。
90°時の画像上方向は+Yです。映像、プレビュー、送信する撮影位置・姿勢はすべて回転後の`CameraOptical`に追従します。
bridgeは画像の時刻に対応する光学frameの姿勢を動的TFとして送信します。
モデル原点は台座の取付面です。VAB/SPHでは角度を変更すると本体の向きがその場で更新されます。

| 設定 | 既定値 | 設定範囲 |
|---|---:|---:|
| 解像度 | 320 × 240 | 幅16〜1280、高さ16〜720 |
| frame rate | 5 Hz | 1〜15 Hz |
| 垂直FOV | 60° | 20〜120° |
| ゲームのclip planeを使用 | true | `useGameClipPlanes`で設定 |
| near clip（独自制限時） | 0.05 m | 0.01〜10 m |
| far clip（独自制限時） | 20000 m | near + 1 m以上 |
| 1チャンク | 12000 byte | 512〜36000 byte |
| 最大frame | 4 MiB | 1 KiB〜16 MiB |

Part Action Windowには160 × 120、320 × 240、640 × 480の解像度presetがあり、frame rateとFOVも変更できます。

VAB/SPHの組み立て中、または飛行中のactive vesselで、RGBカメラパーツを右クリックして`Show RGB Preview`を選ぶと、カメラ映像のライブプレビューを開けます。ウィンドウはタイトルバーで移動でき、右上の`X`またはパーツメニューの`Hide RGB Preview`で閉じます。センサーIDを表示するため、複数のカメラを開いて見比べることもできます。

プレビューは飛行中のUDP送信用と同じ撮影処理を使い、解像度・frame rate・FOV・Camera Tiltの変更を反映します。`Camera: On`なら`UDP: Off`でもローカル表示でき、ROS2 bridgeの起動は不要です。VAB/SPHでは格納庫内をカメラ視点で表示し、UDP設定にかかわらず送信しません。`Camera: Off`では映像表示を停止します。F2でゲームUIと一緒に非表示になり、active vesselの切り替えやシーン終了時には閉じます。

VAB/SPHの`Edit ROS2 Sensor ID`で`<camera_id>`を設定します。新規パーツには`camera_<8桁UID>`が自動設定され、UDPの`sensorId`としてbridgeへ渡されます。

## Frame

camera frameはREP-103 optical規約です。

- `+X`: 画像の右
- `+Y`: 画像の下
- `+Z`: カメラ前方

active vesselモデルへ接続できる場合は対応linkの子`camera_optical_frame`になります。モデルがない場合のfallbackは次です。

```text
<frame_prefix>_<camera_id>_camera_optical_frame
```

## 欠落フレーム

1チャンクでも欠けたフレームはpublishしません。未完成のframe assemblyは2秒で破棄されます。センサーTopic自体は、完全なフレームの最終publishから`--topic-timeout-sec`経過後に削除されます。
