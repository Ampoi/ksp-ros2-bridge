# Active vesselモデル

Flight中の操作機体を、RViz向けのランタイム用プロキシURDFとTFとして公開します。KSPのmeshやtextureを再配布する仕組みではありません。

## 出力Topic

| Topic | 型 | QoS | 内容 |
|---|---|---|---|
| `/ros2_ksp/active_vessel/robot_description` | `std_msgs/msg/String` | Reliable / transient local / depth 1 | URDF文字列 |
| `/ros2_ksp/active_vessel/root_frame` | `std_msgs/msg/String` | Reliable / transient local / depth 1 | root link名 |
| `/tf` | `tf2_msgs/msg/TFMessage` | TransformBroadcaster既定 | 固定jointとセンサーframe |

```bash
ros2 topic echo --once /ros2_ksp/active_vessel/root_frame
ros2 topic echo --once /ros2_ksp/active_vessel/robot_description
```

## RViz2

1. `ros2 topic echo --once /ros2_ksp/active_vessel/root_frame`でroot名を確認
2. RViz2のFixed Frameへその値を設定
3. RobotModelのDescription SourceをTopicへ変更
4. Description Topicを`/ros2_ksp/active_vessel/robot_description`へ設定

bridgeがURDF固定jointを`/tf`へ既定5 Hzでpublishするため、この表示だけなら別の`robot_state_publisher`は不要です。

## 更新と期限切れ

- LiDARパーツのうち1個がactive vesselモデルの送信担当になります。
- 機体構成や相対姿勢が変わると新しいURDFを送ります。
- 既定の再送間隔は2秒です。
- 受信モデルの有効期限は`max(3秒, refresh間隔 × 3)`です。既定では6秒です。
- active vessel切替やFlight終了ではclearを送ります。
- clear時と期限切れ時は、URDFとroot frameへ空文字列をpublishします。

## センサーframeとの接続

受信センサーパケットの`partFlightId`がURDFのpart mappingに存在すると、bridgeは対応linkの子へセンサーposeをTF配信します。LiDAR / Image / CameraInfoの`frame_id`もこの子frameに一致します。

モデルがない、期限切れ、またはpart mappingにない場合は、センサーごとのfallback frameを使います。fallbackは機体TFへ接続されません。

## 資産保護と受信検証

- link名は起動ごとのrandom session IDを含む匿名名です。
- visual / collisionはcollider由来のbox、cylinder、sphereだけです。
- mesh colliderも近いprimitiveへ単純化し、元meshは含みません。
- `GameData` path、part名、メーカー名、textureを含みません。
- gzip、base64、SHA-256、chunk数、展開サイズを検証します。
- XMLは許可したURDFタグと属性だけを受け入れ、`mesh`、外部URI、DTD、entityを拒否します。
- bridgeはURDFをファイルへ保存せず、メモリ上だけで保持します。

既定では非loopback送信元のモデルパケットを拒否します。別ホストで使う場合だけ、KSP側`allowRemoteUrdf = true`とbridge側`--allow-remote-models`を両方指定してください。ROS2 Topicの到達範囲はDDS設定に従うため、必要に応じて`ROS_LOCALHOST_ONLY=1`やSROS2も使用します。

## 実装確認先

- `Source/KerbalLiDAR/ModuleKerbalLidar.VesselUrdf.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/vessel_model.py`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/udp_bridge.py`
