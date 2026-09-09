# デモ

PyLoNのセンサーと制御APIを使うサンプルアプリケーションです。各ページに必要な機体、追加パッケージ、起動順、結果の見方、停止・再開方法をまとめています。

## デモを選ぶ

| デモ | 試せること | 必要な機体・センサー | bridgeの起動 |
|---|---|---|---|
| [軌道上のデブリ周回・撮影](debris-orbit.md) | 相対運動の推定、RCS周回制御、36度ごとの撮影 | 6軸RCS、3D LiDAR、RGBカメラ、近くのデブリ | 別ターミナルで起動 |
| [2D LiDARとSLAM](lidar-slam.md) | 地図作成・保存、保存地図でのNav2走行 | 水平に固定した2D LiDAR、平面移動できる機体 | 別ターミナルで起動 |
| [月面Nav2](mun-nav2.md) | 局所地図の生成、RVizのゴールへの自律走行 | Mun上の前輪操舵ローバー、3D LiDAR | launchが起動 |

リポジトリには、3D LiDARだけで6DoF位置を推定する`pylon_demo_position_estimator`もあります。起動手順は`Demo/pylon_demo_position_estimator/README.md`を参照してください。

## 共通の準備

1. [Getting Started](../guide/getting-started.md)でUbuntu 24.04とROS2 Jazzy、PyLoN MODとbridgeを導入します。
2. KSPの通常操作で各デモに合う機体を準備します。デモは機体やセーブを自動生成しません。
3. 各ページの手順で追加の依存パッケージを導入し、デモを同期・ビルドします。
4. 起動するすべてのターミナルでROS環境を読み込みます。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

ワークスペースを変更している場合は、`~/ros2_ws`を実際のパスへ置き換えてください。同期コマンドにも同じ`ROS2_WS`を指定します。

全デモをまとめて導入する場合は、リポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths Ros2 Demo --ignore-src --rosdistro jazzy -y
./sync.sh --skip-ksp-build --skip-ksp-sync --all-demos
source ~/ros2_ws/install/setup.bash
```

この同期は、Getting Startedでインストール済みのMODを使ってROS2側を追加する手順です。MODも更新するときはKSPを終了し、`./sync.sh --all-demos`を実行してから再起動します。

## Sensor IDと起動順

各ページの`front_lidar`や`orbit_camera`は例です。VAB/SPHのパーツ右クリックメニューにある`Edit ROS2 Sensor ID`で設定するか、launch引数を実際のIDへ変更してください。

同じKSPへ接続するbridgeは1つだけ起動します。軌道周回と2D SLAMでは別途bridgeを起動し、月面Nav2ではlaunchが起動するbridgeを使います。複数の制御デモも同時に動かさず、現在のデモを停止してから切り替えてください。

## 入力を受信できない場合

まず`/ksp_vessel/lifecycle`がACTIVEか、対象のセンサーTopicが届いているかを確認します。詳しい確認手順は[トラブルシュート](../reference/troubleshooting.md)、通信先の設定は[Bridge起動オプション](../reference/bridge-options.md)にあります。
