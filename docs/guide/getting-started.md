# 起動手順

ここでは、リポジトリからKSP modとROS2 bridgeを同期し、実際にTopicを確認するまでを説明します。既定環境はKSP 1.x、Ubuntu 24.04上のROS2 Jazzy、bashです。

## 1. ビルドと同期

KSP 1.x、.NET SDK、ROS2 Jazzy、colcon、rsyncを準備します。KSP本体のDLLはビルド時の参照にのみ使用します。

```bash
./sync.sh
# 必要なデモだけ追加
./sync.sh --demo mun_rover
# すべてのデモ
./sync.sh --all-demos
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

本体は`pylon_interfaces`・`pylon_bridge`・`pylon_vehicle_control`の3パッケージです。デモ選択時だけNav2や点群処理などの依存が必要になります。以前のインストールからの変更は[移行](../reference/migration.md)を先に実施してください。

## 2. ROS2 bridgeを起動

新しいターミナルでworkspaceをsourceしてから起動します。同一PC内だけで使う場合は、受信hostをloopbackへ限定する設定を推奨します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pylon_bridge udp_bridge --host 127.0.0.1 --port 49010
```

起動直後にbridgeは`listening`をpublishします。

```bash
ros2 topic echo --once /pylon/status
```

期待値:

```yaml
data: listening
```

## 3. KSPを起動

通常どおりKSPを起動し、VAB/SPHで必要なパーツを機体へ取り付けます。

- センサーは右クリックの`Edit ROS2 Part Name`でわかりやすい名前を付ける
- サーボとリニアモーターは`bottom`側を親、駆動対象を`top`側へ取り付ける
- Flightへ移動する

## 4. Topicを確認

Flightでデータ送信が始まった後に確認します。

```bash
ros2 topic list
ros2 topic echo /ksp_vessel/joint_states
ros2 topic echo /ksp_vessel/lidar_2d/front_lidar/scan
ros2 topic hz /ksp_vessel/camera/rgb_camera/image_raw
ros2 topic echo /ksp_vessel/ground_truth/pose
ros2 topic echo --once /ksp_vessel/lifecycle
ros2 topic echo /ksp_vessel/control/authority/state
ros2 topic list | grep '^/ksp_vessel/actuators/'
```

センサーTopicは最初のフレームを受け取るまで作られません。bridgeだけを起動した段階で見えないのは正常です。

## 別ホストのKSPへ接続する場合

bridgeからKSPへ返す指令先を指定します。

```bash
ros2 run pylon_bridge udp_bridge \
  --host 0.0.0.0 \
  --port 49010 \
  --command-host 192.168.1.50 \
  --command-port 49011
```

KSP側の共通設定`PYLON_TRANSPORT.stateHost`をbridgeホストへ合わせてください。active vesselのURDFを別ホストへ流す場合に限り、KSP側`allowRemoteUrdf = true`とbridge側`--allow-remote-models`の両方が必要です。信頼できるネットワーク内だけで有効にしてください。

## ドキュメントをローカル起動

リポジトリのルートから次を実行します。

```bash
cd docs
pnpm install
pnpm run docs:dev
```

本番相当の静的ビルドは次のコマンドです。

```bash
pnpm run docs:build
```

ビルド結果をローカルで確認する場合は、続けて`pnpm run docs:preview`を実行します。これらはローカル操作だけで、デプロイは行いません。


Vercelで公開する場合は、プロジェクトのRoot Directoryを`docs`に設定します。CLIから操作する場合も`docs/`で実行してください。
