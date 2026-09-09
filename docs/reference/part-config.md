# パーツ設定

インストール先の標準設定は`GameData/PyLoN/Parts/*/part.cfg`にあります。ソースから開発する場合の原本は`Assets/PyLoN/Parts/*/part.cfg`です。リポジトリ内の`GameData/`はビルドごとに再生成されます。変更後はKSPを再起動してDLLとCFGを読み直してください。

## LiDAR共通

| Key | 内容 |
|---|---|
| `sensorMode` | `2D`または`3D` |
| `horizontalLaserCount` | 2Dの水平ray数。UI上1〜2048 |
| `verticalLaserCount` | 矩形走査の垂直ray数。UI上1〜128 |
| `horizontalFovDegrees` | 0〜360° |
| `verticalFovDegrees` | 0〜180° |
| `maxDistance` | 最大測距距離[m] |
| `scanRateHz` | 1〜60 Hz |
| `streamAt20Fps` | 有効時は`scanRateHz`を無視して20 Hz |
| `udpEnabled` | UDP送信の有効・無効 |
| `lidarEnabled` | Flightスキャンの有効・無効 |
| `ignoreOwnVessel` | 自機collisionを除外 |
| `forwardAxis` / `upAxis` | 2Dまたは手動3Dのパーツlocal軸 |
| `rayOriginLocalPosition` | ray原点のパーツlocal座標 |
| `originOffsetMeters` | 原点から前方への追加offset |
| `noHitValue` | UDP上の未ヒット値。既定`-1` |
| `maxLaserCount` | 1 scanのray上限。既定4096 |
| `maxDatagramBytes` | LiDAR JSON datagram上限。既定60000 |
| `includeHitPoints` | hit座標をUDP JSONへ含める |
| `includeDirections` | ray方向をUDP JSONへ含める |

3D標準パーツは`align3DToAttachNormal = true`で、取付面の外向き法線を半球前方に使います。手動軸へ切り替える場合だけ`false`にします。

## 3D LiDAR profile

| Key | 既定値 | UI範囲 |
|---|---:|---:|
| `rangeProfile` | `medium` | `near` / `medium` / `long` |
| `nearRangeMeters` | 30 | 10〜30 m |
| `mediumRangeMeters` | 100 | 50〜150 m |
| `longRangeMeters` | 250 | 150〜250 m |
| `hemisphereDensity` | profile選択時に160 / 320 / 1024 | 1〜1024 rays/sr |

## 共通設定と機体モデル

`GameData/PyLoN/Config/Runtime.cfg`で管理します。センサーパーツの有無に依存しません。

| Node | Key | 既定値 | 内容 |
|---|---|---|---|
| `PYLON_TRANSPORT` | `stateHost` / `statePort` | `127.0.0.1` / `49010` | bridgeへの送信先 |
| `PYLON_TRANSPORT` | `commandPort` | `49011` | command受信port |
| `PYLON_MODEL` | `enabled` | `true` | 機体モデル送信 |
| `PYLON_MODEL` | `refreshSeconds` | `2` | 再送間隔 |
| `PYLON_MODEL` | `chunkBytes` / `maxChunks` | `12000` / `256` | 圧縮モデル分割上限 |
| `PYLON_MODEL` | `allowRemoteUrdf` | `false` | loopback以外への送信許可 |

センサーIDは各パーツの`ModulePyLoNSensorId.sensorId`で設定します。ユーザーが設定したIDは改名しません。

## RGBカメラ

| Key | 既定値 | 内容 |
|---|---:|---|
| `imageWidth` / `imageHeight` | 320 / 240 | 解像度 |
| `frameRateHz` | 5 | 1〜15 Hz |
| `verticalFovDegrees` | 60 | 20〜120° |
| `useGameClipPlanes` | `true` | ゲームカメラの描画範囲を維持し、遠方の地形・雲を含める |
| `nearClipMeters` / `farClipMeters` | 0.05 / 20000 | `useGameClipPlanes = false`時のローカルclip plane制限 |
| `frameChunkBytes` | 12000 | 1 UDP chunkのraw byte数 |
| `maxFrameBytes` | 4194304 | raw RGB frame上限 |
| `maxFrameChunks` | 512 | KSP側のchunk上限 |
| `cameraEnabled` / `udpEnabled` | `true` | capture / UDP送信 |
| `cameraTiltDegrees` | `90` | 本体の上下首振り。0〜180°、craft／セーブに永続化 |
| `cameraPivotTransformName` | `CameraPivot` | ローカルX軸で回転する本体。0°の基準回転はidentity |
| `cameraOpticalTransformName` | `CameraOptical` | 回転本体内の撮影基準Transform。+Zが撮影方向、+Yが画像上方向 |
| `cameraOriginLocalPosition` | `0, 0, -0.272` | 光学Transformがない旧モデルで使う撮影原点 |
| `forwardAxis` / `upAxis` | `-Z` / `+Y` | 光学Transformがない場合のcamera local軸 |

## モーター共通

| Key | サーボ既定 | リニア既定 | 内容 |
|---|---:|---:|---|
| `motorName` | 空 | 空 | ROS joint名。空ならIDから生成 |
| `stateRateHz` | 20 | 20 | state送信Hz。実行時1〜60へclamp |
| `commandTimeoutSeconds` | 0.5 | 0.5 | velocity mode timeout |
| 定格effort | `ratedEffortNm = 250` | `ratedEffortN = 4000` | effort上限 |
| 電流換算 | `torquePerAmpNm = 40` | `forcePerAmpN = 800` | 推定電流用 |
