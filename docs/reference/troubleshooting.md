# トラブルシュート

## bridge statusが見えない

bridgeをsourceしたターミナルで起動しているか確認します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pylon_bridge udp_bridge --host 127.0.0.1 --port 49010
```

別ターミナルでも同じROS setupとworkspaceをsourceしてください。`/pylon/status`はKSP未起動でも存在します。

`ROS_DISTRO`が`jazzy`にならない場合は、Humbleをsourceしたシェルを使い回さず、新しいターミナルで`/opt/ros/jazzy/setup.bash`からsourceし直してください。

## センサーTopicが見えない

LiDARとカメラTopicは動的です。次を確認します。

1. 対象パーツを搭載した機体でFlightへ入っている
2. Part Action WindowでセンサーとUDPが有効
3. KSP側`PYLON_TRANSPORT.stateHost` / `PYLON_TRANSPORT.statePort`とbridge側`--host` / `--port`が対応
4. スキャンまたは完全なcamera frameが実際にbridgeへ届いている
5. `--topic-timeout-sec`がセンサー周期より長い

## Topicが約3秒で消える

動的センサーpublisherは最終データから既定3秒で削除されます。低速センサーを使う場合は長くします。

```bash
ros2 run pylon_bridge udp_bridge \
  --host 127.0.0.1 \
  --topic-timeout-sec 10
```

## モーターが動かない

- `/ksp_vessel/joint_states.name`から実際のjoint名をコピーする
- `MotorCommand.id`とlease、増加するsequenceを確認する
- サーボはrad、リニアはmで指定する
- `bottom`側を親、動かす構造を`top`側へ取り付ける
- KSPのPart Action Windowで電力、engage、lock状態を確認する
- bridgeの`--command-host` / `--command-port`とパーツの`commandUdpPort`を合わせる

## 速度指令・推力がすぐ止まる

正常なフェイルセーフ動作です。正式commandはtimeoutより短い周期で、同じlease内の`sequence`を増やしながら送信してください。固定sequenceを繰り返す`ros2 topic pub -r`はreplayとして拒否されるため、連続制御には`pylon_vehicle_control`などのcontroller nodeを使います。

## RVizに機体が出ない

```bash
ros2 topic echo --once /ksp_vessel/root_frame
ros2 topic echo --once /ksp_vessel/robot_description
```

空文字列の場合、モデルがclearまたは期限切れです。共通設定の`PYLON_MODEL.enabled`、UDP経路、remote接続時の両側許可を確認してください。RVizのFixed Frameは`root_frame` Topicから得た値を使います。

## カメラ画像が途切れる

camera frameは全UDPチャンクが揃った場合だけpublishします。まず160 × 120または低いframe rateで確認してください。ネットワークを跨ぐ場合、raw RGBを多数のUDP datagramへ分割するため、packet lossの影響を受けやすくなります。

## KSP DLLが更新されない

KSP実行中は新しいDLLを読み直せません。KSPを終了してから`./sync.sh`を再実行し、KSPを起動し直してください。

## ログ

KSP側の関連logは`[PyLoN]`で始まります。bridge側はinvalid packet、publisherの作成・削除、UDP送信失敗をnode loggerへ出力します。
