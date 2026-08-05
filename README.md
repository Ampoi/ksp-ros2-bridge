# Kerbal LiDAR

Kerbal Space Program 1.x向けの表面取付LiDAR modです。2D LiDARと3D LiDARの2パーツを追加し、レイが当たった距離をUDP JSONとして外部プログラムへ送信します。

## できること

- 表面取付の2D LiDARパーツ
- 表面取付の3D LiDARパーツ
- `part.cfg`からレーザー本数、FOV、最大距離、スキャン周波数、UDP送信先を変更
- レイキャスト結果をUDP JSONで外部へ送信
- 別ROS2パッケージでUDP JSONを標準ROS2 Topicへ中継
- 自船コライダーを無視する設定

## ビルド

KSP本体のManaged DLLを参照してビルドします。KSPのインストール先を`KSPDIR`として渡してください。

```powershell
.\build.ps1 -KspDir "C:\SteamLibrary\steamapps\common\Kerbal Space Program"
```

ビルドに成功するとDLLが`GameData/KerbalLiDAR/Plugins/KerbalLiDAR.dll`へ出力されます。

## インストール

`GameData/KerbalLiDAR`フォルダをKSPの`GameData`へコピーします。

## 設定

レーザー本数などは各パーツのCFGで変更します。

- `GameData/KerbalLiDAR/Parts/Lidar2D/part.cfg`
- `GameData/KerbalLiDAR/Parts/Lidar3D/part.cfg`

主な設定値:

- `horizontalLaserCount`: 水平方向のレーザー数
- `verticalLaserCount`: 垂直方向のレーザー数。2Dでは`1`
- `horizontalFovDegrees`: 水平FOV
- `verticalFovDegrees`: 垂直FOV
- `scanRateHz`: 1秒あたりのスキャン回数
- `streamAt20Fps`: `true`にすると`scanRateHz`ではなく20Hz固定でUDP配信
- `lidarName`: ROS2 Topic名に使うLiDAR名。空の場合はパーツ名と`partFlightId`から自動生成
- `maxDistance`: 最大測距距離[m]
- `udpHost`: UDP送信先
- `udpPort`: UDP送信先ポート
- `ignoreOwnVessel`: 自船コライダーを無視
- `forwardAxis` / `upAxis`: レイを飛ばすパーツローカル軸
- `includeHitPoints`: ヒット座標もJSONに含める
- `includeDirections`: レイ方向もJSONに含める

初期設定では、表面法線の外向きに合わせて`forwardAxis = -Y`を使っています。スキャン方向がずれる場合は、CFG内の`forwardAxis`と`upAxis`を`+X`、`-X`、`+Y`、`-Y`、`+Z`、`-Z`のいずれかへ変更してください。

## レーザープレビュー

VAB/SPHまたはFlightでLiDARパーツを右クリックし、Part Action Windowの`Show Laser Preview`を押すと、そのパーツだけ赤いレーザー線を表示します。表示中は同じボタンが`Hide Laser Preview`になり、そのパーツだけ非表示にできます。

アクショングループには`Toggle Laser Preview`として登録できます。

## ROS2

KSP側からROS2プロセスは起動しません。別プロセスとして`Ros2/ksp_lidar_bridge`パッケージを起動し、KSPから飛んでくるUDP JSONをLiDAR名ごとのTopicへ変換します。

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select ksp_lidar_bridge
source install/setup.bash
ros2 run ksp_lidar_bridge udp_bridge --host 0.0.0.0 --port 49010
```

Topicは`/ksp_ros2/lidar/<name>`です。2D LiDARは`sensor_msgs/msg/LaserScan`、3D LiDARは`sensor_msgs/msg/PointCloud2`としてpublishします。

```bash
ros2 topic list | grep /ksp_ros2/lidar
ros2 topic echo /ksp_ros2/lidar/front_lidar
```

## UDP JSON

送信されるJSONは1スキャン1パケットです。距離配列は垂直方向を外側、水平方向を内側にしたフラット配列です。未ヒットはデフォルトで`-1`です。

```json
{
  "type": "ksp_lidar_scan",
  "version": 1,
  "mode": "3D",
  "name": "front_lidar",
  "lidarName": "front_lidar",
  "vessel": "Rover",
  "partFlightId": 12345,
  "universalTime": 42.0,
  "horizontalCount": 64,
  "verticalCount": 16,
  "horizontalFovDeg": 360.0,
  "verticalFovDeg": 30.0,
  "maxDistance": 2000.0,
  "layout": "vertical-major",
  "hitCount": 512,
  "ranges": [1.23, 1.25, -1.0],
  "hitMask": [1, 1, 0]
}
```

簡易受信:

```powershell
python .\Tools\lidar_udp_listener.py --port 49010
```

## 並列開発（Git worktree）

機能ごとに独立したブランチと作業ディレクトリを作成できます。worktreeは既定でリポジトリ内の`.worktrees/`へ作られ、このディレクトリ自体はGit管理から除外されます。

```bash
# work/ros2-refactorブランチと対応するworktreeを作成
./Tools/worktree.sh create ros2-refactor
cd .worktrees/ros2-refactor

# 確認と削除
./Tools/worktree.sh list
./Tools/worktree.sh remove ros2-refactor
git branch -d work/ros2-refactor
```

各worktreeでは変更を小さくコミットし、元のworktreeから`git merge --no-ff work/<name>`で統合します。同じブランチを複数のworktreeで同時にcheckoutすることはできません。

`dev_sync.sh`の同期先であるKSP本体とROS2ワークスペースは全worktreeで共有されます。各worktree内のローカルなテストとビルドは並行できますが、`dev_sync.sh`による共有先への同期は統合後に一つのworktreeから実行してください。
