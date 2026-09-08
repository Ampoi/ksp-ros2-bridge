# RGBカメラ

`Kerbal ROS2 RGB Camera`はFlight描画をraw RGBフレームとして取得します。KSPからはチェックサム付きUDPチャンクで送り、bridgeは全チャンクが揃ってSHA-256検証に成功したフレームだけをpublishします。

画像はKSPの銀河背景、Scaled Space、Flightの近距離／遠距離カメラをセンサー視点で合成します。画面UIは含みません。描画先はフレームごとにクリアされるため、空や背景が前フレームの透明画素として残ることはありません。

撮影にはゲーム本体のカメラを一時的にセンサー視点へ移して使い、カメラに登録された雲・大気MODの描画コールバック、image effect、command bufferも実行します。撮影後は例外発生時もカメラ設定と位置・姿勢を戻します。視点とFOVはカメラパーツの設定であり、プレイヤー視点のスクリーンショットではありません。

MODの深度バッファを撮影ごとに作り直させないよう、内部の描画先はゲーム画面と同じサイズで維持します。描画後に指定したセンサー解像度へ縮小し、プレビューとUDPへ渡します。出力解像度を切り替えても内部の描画先は使い回します。FOV・画像の縦横比・CameraInfoはセンサー設定を維持します。内部描画の負荷はゲーム画面の解像度にも依存します。

既定の`useGameClipPlanes = true`ではゲームと同じ近距離／遠距離の描画範囲を使い、地形や雲を20 kmで切りません。独自の距離制限が必要な場合だけpart.cfgで`false`にすると、`nearClipMeters` / `farClipMeters`でローカル描画範囲を制限します。Scaled Spaceと銀河背景にはこの制限を適用しません。

ScattererのTAA（過去フレームを使うアンチエイリアス）は、撮影中だけ一時停止します。センサー映像がゲーム画面用の履歴へ混入して撮影周期でちらつくのを防ぐためです。撮影後はエラー時も元の有効状態へ戻し、ゲーム画面ではTAAを継続します。センサー画像にはこのTAAを適用せず、EVEの雲・Scattererの大気と遠方地形は引き続き描画します。

撮影の一時的な負荷だけでScattererの低FPS判定にかかった場合も、ゲーム画面のTAAを維持します。実測した撮影時間を差し引いて判定し、撮影以外の処理でも低FPSになる場合はScattererの設定に従います。画面解像度や保存された画質設定は変更しません。撮影処理そのものの負荷をなくす機能ではありません。

EVE・Scattererのゲームカメラに付いた処理を利用する方式ですが、他のMOD独自のフレーム履歴や画面サイズ依存処理までの一致は保証しません。MOD入りの実ゲームで雲・地平線・大気と撮影後の通常画面を確認してください。

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
角度はcraft／セーブに保存され、`part.cfg`内の`ModuleKerbalRgbCamera`の`cameraTiltDegrees`で新規パーツの初期角度を指定できます。

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
モデル原点が台座の取付面です。保存済みの表面取付ノードにモデルを合わせるため、旧スポットライトモデルの機体も付け直しは不要です。VAB/SPHでは角度を変更すると本体の向きがその場で更新されます。

モデルの編集元と再生成手順はGit管理外のローカル開発領域で管理します。

| 設定 | 既定値 | 実装上の範囲 |
|---|---:|---:|
| 解像度 | 320 × 240 | 内部clampは幅16〜1280、高さ16〜720 |
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

## 実装確認先

- `GameData/KerbalLiDAR/Parts/RgbCamera/part.cfg`
- `Source/KerbalLiDAR/Api/Ksp/ModuleKerbalRgbCamera.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/camera_packets.py`
