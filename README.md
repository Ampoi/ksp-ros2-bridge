# Kerbal LiDAR

Kerbal Space Program 1.x向けの表面取付LiDAR modです。2D LiDARと3D LiDARの2パーツを追加し、レイが当たった距離をUDP JSONとして外部プログラムへ送信します。

## できること

- 表面取付の2D LiDARパーツ
- 表面取付の3D LiDARパーツ
- `part.cfg`からレーザー本数、FOV、最大距離、スキャン周波数、UDP送信先を変更
- レイキャスト結果をUDP JSONで外部へ送信
- 別ROS2パッケージでUDP JSONを標準ROS2 Topicへ中継
- Flight中の操作機体を、資産を含まないランタイム用プロキシURDFとしてROS2へ中継
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
- `partName`: ROS2 Topic名に使う機体内で一意なパーツ名。VAB/SPHで編集可能
- `maxDistance`: 最大測距距離[m]
- `udpHost`: UDP送信先
- `udpPort`: UDP送信先ポート
- `activeVesselUrdfEnabled`: Flight中の操作機体プロキシURDFを送信
- `activeVesselUrdfRefreshSeconds`: URDFを再送・更新する間隔。既定値は2秒
- `activeVesselUrdfChunkBytes`: gzip圧縮後の1チャンクの最大バイト数
- `maxActiveVesselUrdfChunks`: 1モデルに許可する最大チャンク数
- `allowRemoteUrdf`: loopback以外へのURDF送信を明示的に許可。既定値は`false`
- `ignoreOwnVessel`: 自船コライダーを無視
- `forwardAxis` / `upAxis`: レイを飛ばすパーツローカル軸
- `includeHitPoints`: ヒット座標もJSONに含める
- `includeDirections`: レイ方向もJSONに含める

初期設定では、表面法線の外向きに合わせて`forwardAxis = -Y`を使っています。スキャン方向がずれる場合は、CFG内の`forwardAxis`と`upAxis`を`+X`、`-X`、`+Y`、`-Y`、`+Z`、`-Z`のいずれかへ変更してください。

## レーザープレビュー

VAB/SPHまたはFlightでLiDARパーツを右クリックし、Part Action Windowの`Show Laser Preview`を押すと、そのパーツだけ赤いレーザー線を表示します。表示中は同じボタンが`Hide Laser Preview`になり、そのパーツだけ非表示にできます。

アクショングループには`Toggle Laser Preview`として登録できます。

## ROS2パーツ名

VAB/SPHでLiDARパーツを右クリックし、`Edit ROS2 Part Name`を押すとパーツ名を編集できます。入力値はROS2名として使える英数字とアンダースコアへ正規化され、同じ機体内に同名がある場合は`_2`、`_3`のような接尾辞が自動で付きます。未入力のパーツにも自動で一意な名前が設定されます。

この名前はcraftファイルへ保存され、各LiDARパーツのTopic名前空間になります。以前のcraftに保存された`lidarName`も引き続き読み込めます。

## ROS2

KSP側からROS2プロセスは起動しません。別プロセスとして`Ros2/ksp_lidar_bridge`パッケージを起動し、KSPから飛んでくるUDP JSONをLiDAR名ごとのTopicへ変換します。

```bash
mkdir -p ~/ros2_ws/src
cp -r Ros2/ksp_lidar_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select ksp_lidar_bridge
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ros2 run ksp_lidar_bridge udp_bridge --host 127.0.0.1 --port 49010
```

Topicはパーツごとに作られます。2D LiDARは`sensor_msgs/msg/LaserScan`、3D LiDARは`sensor_msgs/msg/PointCloud2`としてpublishします。

- 2D LiDAR: `/ros2_ksp/<part_name>/lidar/scan`
- 3D LiDAR: `/ros2_ksp/<part_name>/lidar/points`

同じ機体に複数のLiDARを搭載した場合も、各パーツ名のTopicへすべてpublishされます。将来カメラを追加する場合も、`/ros2_ksp/<part_name>/camera/image_rgb`のように同じ階層へ拡張できます。

TopicはFlight中にスキャンを受信したときだけ作成されます。Flight終了時に自動削除され、停止通知が欠落してもスキャン停止から3秒後に削除されます。次にFlightへ入ると最初のスキャンから自動でpublishを再開します。タイムアウトはbridgeの`--topic-timeout-sec`で変更できます。

```bash
ros2 topic list | grep /ros2_ksp
ros2 topic echo /ros2_ksp/front_lidar/lidar/scan
```

### Active vesselのランタイムURDF

Flight中は、現在操作している機体に搭載されたLiDARのうち1個だけが送信担当になります。機体のパーツ構成や相対姿勢が変わるとURDFも更新され、操作機体を切り替えると古いモデルはclearまたは有効期限切れになります。

- URDF: `/ros2_ksp/active_vessel/robot_description` (`std_msgs/msg/String`, transient local)
- RVizのFixed Frameに使うルート名: `/ros2_ksp/active_vessel/root_frame` (`std_msgs/msg/String`)
- 固定ジョイントTF: `/tf`（既定5Hz）

確認例:

```bash
ros2 topic echo --once /ros2_ksp/active_vessel/root_frame
ros2 topic echo --once /ros2_ksp/active_vessel/robot_description
```

RViz2ではRobotModel表示のDescription Topicを`/ros2_ksp/active_vessel/robot_description`にし、Fixed Frameには`root_frame` Topicで得た値を指定します。TFはbridgeが配信するため、この用途だけなら別の`robot_state_publisher`は不要です。LiDARの`partFlightId`が現行モデルに含まれる場合、センサメッセージの`frame_id`も対応するURDF linkへ自動的に揃います。

### URDF資産保護の境界

送信するものはKSP機体そのものの再配布用URDFではなく、そのセッション中だけ使うプロキシです。

- KSPのmesh、texture、パーツ名、メーカー名、`GameData`パスは含めません。
- visual/collisionはコライダーから作る0.1m単位のbox近似だけです。link名は起動ごとのランダムsession IDを含みます。
- gzipチャンクはSHA-256検証され、ROS2側ではサイズ上限とXML許可リストを適用します。`mesh`や外部URIは拒否します。
- bridgeはURDFをファイル保存せず、メモリ上だけで保持します。KSP・ROS2とも既定ではloopback限定で、ROS2モデルTopicは`ROS_LOCALHOST_ONLY=1`がないと無効です。
- 保護されたROSネットワークで意図的に共有する場合だけ、KSPの`allowRemoteUrdf = true`とbridgeの`--allow-remote-models --allow-network-model-topics`を指定します。

ROS2 Topicへ平文をpublishした後、同じホスト上の別プロセスによる購読・rosbag保存まで完全に防ぐことはできません。この実装は再利用可能なゲーム資産を最初から含めず、ネットワーク配布と永続化を既定で避ける設計です。より強いアクセス制御が必要な環境では、OSユーザー分離とSROS2/DDS Securityも併用してください。

## UDP JSON

送信されるJSONは1スキャン1パケットです。距離配列は垂直方向を外側、水平方向を内側にしたフラット配列です。未ヒットはデフォルトで`-1`です。

```json
{
  "type": "ksp_lidar_scan",
  "version": 1,
  "mode": "3D",
  "name": "front_lidar",
  "partName": "front_lidar",
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
cd ../..
./Tools/worktree.sh remove ros2-refactor
git branch -d work/ros2-refactor
```

各worktreeでは変更を小さくコミットし、元のworktreeから`git merge --no-ff work/<name>`で統合します。同じブランチを複数のworktreeで同時にcheckoutすることはできません。

主worktreeにローカル.NET SDK（`.dotnet` / `.dotnet-linux-net8`）やNuGetキャッシュ（`.nuget`）がある場合、作成したworktreeからsymlinkで共有されます。大容量の開発依存をworktreeごとに複製せず、KSPプラグインを同じ環境でビルドできます。

`dev_sync.sh`の同期先であるKSP本体とROS2ワークスペースは全worktreeで共有されます。スクリプト同士はファイルロックで直列化されますが、後から実行したブランチの内容が共有先へ反映されます。各worktree内のローカルなテストとビルドは並行し、共有先への最終同期は統合後に一つのworktreeから実行してください。ロックファイルは`DEV_SYNC_LOCK_FILE`で変更できます。
