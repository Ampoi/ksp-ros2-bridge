# 起動手順

ここでは、リポジトリからKSP modとROS2 bridgeを同期し、実際にTopicを確認するまでを説明します。既定環境はKSP 1.x、Ubuntu 24.04上のROS2 Jazzy、bashです。

## 1. ビルドと同期

リポジトリのルートで実行します。

```bash
./sync.sh
```

このスクリプトは次を順番に行います。

1. KSPプラグインDLLをReleaseビルド
2. `GameData/KerbalLiDAR`をKSPの`GameData`へ同期
3. `Ros2`のinterfaces、bridge、共通機体制御、Nav2 packageと`Demo`をROS2 workspaceへ同期
4. 同期したROS2 packageを依存順に`colcon build`

既定パスと異なる場合は環境変数で指定します。

```bash
KSPDIR="/path/to/Kerbal Space Program" \
ROS2_WS="$HOME/ros2_ws" \
ROS_SETUP="/opt/ros/jazzy/setup.bash" \
./sync.sh
```

`sync.sh`はsource後の`ROS_DISTRO`も検査し、Jazzy以外の環境を誤って使った場合はビルド前に停止します。

::: details 手動でROS2 bridgeだけを配置する場合
```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_ros2_interfaces ~/ros2_ws/src/
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cp -r Ros2/ksp_vehicle_control ~/ros2_ws/src/
cp -r Ros2/ksp_nav2_bringup ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to ksp_lidar_bridge ksp_vehicle_control ksp_nav2_bringup
```
:::

Humbleで使っていた同じworkspaceをJazzyへ移行する場合、Python 3.10向け生成物を再利用しないでください。最初のJazzyビルド前にworkspaceの`build`、`install`、`log`を削除してから、上記コマンドで再ビルドします。`src`内のパッケージはそのまま利用できます。

## 2. ROS2 bridgeを起動

新しいターミナルでworkspaceをsourceしてから起動します。同一PC内だけで使う場合は、受信hostをloopbackへ限定する設定を推奨します。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

起動直後にbridgeは`listening`をpublishします。

```bash
ros2 topic echo --once /ros2_ksp/status
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
ros2 run ksp_lidar_bridge udp_bridge \
  --host 0.0.0.0 \
  --port 49010 \
  --command-host 192.168.1.50 \
  --command-port 49011
```

KSP側の各センサー・モーター設定もbridgeホストへ合わせてください。active vesselのURDFを別ホストへ流す場合に限り、KSP側`allowRemoteUrdf = true`とbridge側`--allow-remote-models`の両方が必要です。信頼できるネットワーク内だけで有効にしてください。

## ドキュメントをローカル起動

```bash
npm install
npm run docs:dev
```

本番相当の静的ビルドは次のコマンドです。

```bash
npm run docs:build
```

ビルド結果をローカルで確認する場合は、続けて`npm run docs:preview`を実行します。これらはローカル操作だけで、デプロイは行いません。
