# PyLoN

PyLoNは、Kerbal Space Program 1.xのセンサーと機体をROS2から扱うためのMODです。KSPとROS2 bridgeを接続し、点群・画像を使う認識ノードや、ローバー・宇宙機の制御アプリケーションを開発できます。

## 導入

[Getting Started](docs/guide/getting-started.md)で、Ubuntu 24.04とROS2 Jazzyの準備、MODとROS2パッケージのインストール、bridgeの起動、機体情報と3D LiDARの受信まで説明しています。

導入後は[ROS2アプリケーションを作る](docs/guide/application-development.md)へ進んでください。[システム概要](docs/guide/overview.md)では通信経路、Topicの寿命、IDと座標系を確認できます。

配布版MODは[Releases](https://github.com/Ampoi/KSP_ROS2/releases)の`PyLoN-vX.Y.Z.zip`を展開し、`GameData/PyLoN`をKSPの`GameData`へコピーします。更新前にインストール先の`Config/Runtime.cfg`を控えてください。「Source code」アーカイブにはビルド済みMODは含まれません。ROS2パッケージはソースからビルドします。

## APIリファレンス

| 目的 | 参照先 |
|---|---|
| 入出力とメッセージ型を調べる | [Topic一覧](docs/api/topics.md) |
| 機体の制御権を取得し、力・トルクを指令する | [機体制御とGround Truth](docs/api/vehicle-control.md) |
| 機体モデルとTFを受信し、RVizで表示する | [Active vesselモデル](docs/api/vessel-model.md) |
| 距離・画像・姿勢を取得する | [LiDAR](docs/parts/lidar.md)、[RGBカメラ](docs/parts/camera.md)、[スタートラッカー](docs/parts/star-tracker.md) |
| 関節・車輪・推進系を操作する | [モーターと可動フィン](docs/parts/motors.md)、[ホイール](docs/parts/wheels.md)、[エンジン・RCS](docs/parts/propulsion.md) |
| 切離しやドッキングポートを操作する | [分離機構](docs/parts/separation.md)、[ドッキングポート](docs/parts/docking.md) |

各APIページにTopic名、型、単位、座標系、設定値、指令の条件を記載しています。

## 設定と運用

- [Bridge起動オプション](docs/reference/bridge-options.md)：受信アドレス、指令の送信先、Topic名、タイムアウト
- [パーツ設定](docs/reference/part-config.md)：センサー・モーターの設定と共通の`Runtime.cfg`
- [トラブルシュート](docs/reference/troubleshooting.md)：Topicの受信、モーター制御、画像・機体モデル表示の確認
- [移行ガイド](Migration/README.md)：既存セーブ・機体・ワークスペースの更新

## デモ

[デモ一覧・共通準備](docs/demos/index.md)から、使うセンサーと目的に合うデモを選べます。各ページに機体の準備から起動、動作確認、停止までをまとめています。

- [軌道上のデブリ周回・撮影](docs/demos/debris-orbit.md)
- [2D LiDARとSLAM](docs/demos/lidar-slam.md)：地図作成・保存・Nav2走行
- [月面Nav2](docs/demos/mun-nav2.md)

## PyLoN本体への貢献

MOD、ROS2 bridge、メッセージ定義、ドキュメントへの変更は[本体開発ガイド](docs/contributing/index.md)を参照してください。ビルド・同期、変更の検証、コミットに含める内容、ドキュメントのプレビュー方法をまとめています。

内部構成は[アーキテクチャ](docs/contributing/architecture.md)、変更箇所の案内は[ソース案内](docs/contributing/source-map.md)にあります。

配布資産の原本は`Assets/PyLoN`、`GameData/`はビルドごとに作り直す生成物です。ソースからのビルドには`./build.sh --ksp-dir "$KSPDIR"`（Windowsでは`build.ps1`）を使います。配布ZIPの作成と手動公開は[リリース手順](RELEASING.md)、通信境界は[Bridge設計](Ros2/pylon_bridge/ARCHITECTURE.md)を参照してください。

## ライセンス

[MIT License](LICENSE)。配布物には著作権表示とライセンス本文を同梱します。
