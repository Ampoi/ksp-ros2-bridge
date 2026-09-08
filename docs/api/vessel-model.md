# Active vesselモデル

Flight中の操作機体を、RViz向けのランタイム用プロキシURDFとTFとして公開します。KSPのmeshやtextureを再配布する仕組みではありません。

## 出力Topic

| Topic | 型 | QoS | 内容 |
|---|---|---|---|
| `/ksp_vessel/robot_description` | `std_msgs/msg/String` | Reliable / transient local / depth 1 | URDF文字列 |
| `/ksp_vessel/root_frame` | `std_msgs/msg/String` | Reliable / transient local / depth 1 | root link名 |
| `/tf` | `tf2_msgs/msg/TFMessage` | dynamic | `base_link`からCoM基準proxy root |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Reliable / transient local | 固定jointとセンサー取付frame |
| `/ksp_vessel/lifecycle` | `VesselLifecycle` | Reliable / transient local | vessel ID、generation、model readiness |

```bash
ros2 topic echo --once /ksp_vessel/root_frame
ros2 topic echo --once /ksp_vessel/robot_description
```

## RViz2

1. `ros2 topic echo --once /ksp_vessel/root_frame`でroot名を確認
2. RViz2のFixed Frameへその値を設定
3. RobotModelのDescription SourceをTopicへ変更
4. Description Topicを`/ksp_vessel/robot_description`へ設定

bridgeがURDF固定jointを`/tf_static`へpublishし、CoM変化があるroot edgeだけを`/tf`へ既定5 Hzで更新するため、この表示だけなら別の`robot_state_publisher`は不要です。

## 更新と期限切れ

- Flight共通の`KerbalRosVesselModelManager`がactive vesselモデルを送信します。LiDAR非搭載の機体にも対応し、送信担当は常に一つです。
- 毎回の更新で全パーツの現在の形状と相対姿勢を取得します。構成変更だけでなく、展開・可動・サイズ変更もURDFとTFへ反映します（連続アニメーションではなく更新間隔ごとのスナップショットです）。
- 既定の再送間隔は2秒です。
- 受信モデルの有効期限は`max(3秒, refresh間隔 × 3)`です。既定では6秒です。
- active vessel切替やFlight終了ではclearを送ります。
- 同じvessel IDとmodel hashの定期再送ではproxyを再生成しません。
- Ground Truthで機体切替を先に検出した場合、不一致modelを直ちに切断します。
- clear時と期限切れ時は、URDFとroot frameへ空文字列をpublishします。

## センサーframeとの接続

受信センサーパケットの`partFlightId`がURDFのpart mappingに存在すると、bridgeは対応linkの子へセンサーposeを`/tf_static`で配信します。LiDAR / Image / CameraInfoの`frame_id`は`ros2_ksp_<sensor_id>_<kind>_frame`形式の安定した子frameに一致します。

モデルがない、期限切れ、またはpart mappingにない間も同じsensor frame名を使いますが、機体TFへは接続されません。`VesselLifecycle.model_ready`で区別できます。

## 資産保護と受信検証

- link名はKSPの永続的なvessel IDの短縮prefixを含む匿名名です。同じ機体の再ロードで安定し、機体間では衝突しません。
- 標準・DLC・MODのパーツ名による対応表は使わず、機体の全パーツを走査します。
- visual / collisionは描画要素の形状に近いbox、cylinder、sphereだけです。SkinnedMeshはアニメーション用local boundsを直方体で近似します。
- 描画要素がない場合は非trigger colliderを近似し、それもないパーツには25 cmの直方体を置きます。
- 1パーツあたり最大48形状とし、超過分は全体を覆う1つの直方体にまとめます。
- 各linkの座標はメートル単位です。パーツのスケールを位置・寸法へ反映し、円柱の軸をURDFのZ軸へ変換します。
- mesh colliderも近いprimitiveへ単純化し、元meshは含みません。
- `GameData` path、part名、メーカー名、textureを含みません。
- gzip、base64、SHA-256、chunk数、展開サイズを検証します。
- XMLは許可したURDFタグと属性だけを受け入れ、`mesh`、外部URI、DTD、entityを拒否します。
- bridgeはURDFをファイルへ保存せず、メモリ上だけで保持します。

既定では非loopback送信元のモデルパケットを拒否します。別ホストで使う場合だけ、KSP側`allowRemoteUrdf = true`とbridge側`--allow-remote-models`を両方指定してください。ROS2 Topicの到達範囲はDDS設定に従うため、必要に応じて`ROS_LOCALHOST_ONLY=1`やSROS2も使用します。

## 実装確認先

- `Source/KerbalLiDAR/Api/Ksp/KerbalRosVesselModelManager.Geometry.cs`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/vessel_model.py`
- `Ros2/ksp_lidar_bridge/ksp_lidar_bridge/udp_bridge.py`

## 共通モデル設定

`KERBAL_ROS2_MODEL` ConfigNodeで設定します。設定ノードがない場合は従来の送信担当に相当する最初の有効なLiDAR設定を使用し、LiDARがない場合はloopback:49010へ2秒間隔で送信します。全機体で無効化する場合は共通設定の`enabled = false`を指定します。

```text
KERBAL_ROS2_MODEL
{
    enabled = true
    udpHost = 127.0.0.1
    udpPort = 49010
    refreshSeconds = 2
    chunkBytes = 12000
    maxChunks = 256
    allowRemoteUrdf = false
}
```

共通設定ノードが存在する場合は共通設定を優先し、未指定フィールドには上記の既定値を使用します。機体を切り替えるかFlightを開始すると設定を読み直します。従来のLiDAR個別設定は互換用で、LiDARからのモデル二重送信は行いません。
