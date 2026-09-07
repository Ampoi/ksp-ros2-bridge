# パーツ設定

標準設定は`GameData/KerbalLiDAR/Parts/*/part.cfg`にあります。変更後はKSPを再起動してDLLとCFGを読み直してください。

## LiDAR共通

| Key | 内容 |
|---|---|
| `sensorMode` | `2D`または`3D` |
| `horizontalLaserCount` | 2Dの水平ray数。UI上1〜2048 |
| `verticalLaserCount` | legacy grid用の垂直ray数。UI上1〜128 |
| `horizontalFovDegrees` | 0〜360° |
| `verticalFovDegrees` | 0〜180° |
| `maxDistance` | 最大測距距離[m] |
| `scanRateHz` | 1〜60 Hz |
| `streamAt20Fps` | 有効時は`scanRateHz`を無視して20 Hz |
| `partName` | Topicの`<part_name>`。最大64文字に正規化 |
| `udpHost` / `udpPort` | bridgeの受信先。既定`127.0.0.1:49010` |
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

## Active vessel URDF

| Key | 既定値 | 内容 |
|---|---:|---|
| `activeVesselUrdfEnabled` | `true` | proxy送信 |
| `activeVesselUrdfRefreshSeconds` | 2 | 再送間隔 |
| `activeVesselUrdfChunkBytes` | 12000 | gzip後の1チャンクbyte |
| `maxActiveVesselUrdfChunks` | 256 | 最大チャンク数 |
| `allowRemoteUrdf` | `false` | loopback以外への送信許可 |

## RGBカメラ

| Key | 既定値 | 内容 |
|---|---:|---|
| `imageWidth` / `imageHeight` | 320 / 240 | 解像度 |
| `frameRateHz` | 5 | 1〜15 Hz |
| `verticalFovDegrees` | 60 | 20〜120° |
| `nearClipMeters` / `farClipMeters` | 0.05 / 20000 | clip plane |
| `frameChunkBytes` | 12000 | 1 UDP chunkのraw byte数 |
| `maxFrameBytes` | 4194304 | raw RGB frame上限 |
| `maxFrameChunks` | 512 | KSP側のchunk上限 |
| `partName` | 空 | Topic名。空なら`rgb_camera` |
| `cameraEnabled` / `udpEnabled` | `true` | capture / UDP送信 |
| `cameraOriginLocalPosition` | `0, 0, -0.2` | 光学原点 |
| `forwardAxis` / `upAxis` | `-Z` / `+Y` | camera local軸 |

## モーター共通

| Key | サーボ既定 | リニア既定 | 内容 |
|---|---:|---:|---|
| `motorName` | 空 | 空 | ROS joint名。空ならIDから生成 |
| `commandUdpPort` | 49011 | 49011 | KSP側command bind port |
| `stateUdpHost` | 127.0.0.1 | 127.0.0.1 | state送信先 |
| `stateUdpPort` | 49010 | 49010 | state送信先port |
| `stateRateHz` | 20 | 20 | state送信Hz。実行時1〜60へclamp |
| `commandTimeoutSeconds` | 0.5 | 0.5 | velocity mode timeout |
| 定格effort | `ratedEffortNm = 250` | `ratedEffortN = 4000` | effort上限 |
| 電流換算 | `torquePerAmpNm = 40` | `forcePerAmpN = 800` | 推定電流用 |
