---
layout: home

hero:
  name: PyLoN
  text: KSPをROS2ロボットへ
  tagline: センサー、モーター、ホイール、推進系、機体WrenchとGround TruthをROS2から扱うための導入ガイドとAPIリファレンス。
  actions:
    - theme: brand
      text: Getting Started
      link: /guide/getting-started
    - theme: alt
      text: Topic一覧を見る
      link: /api/topics

features:
  - icon: ◉
    title: センサー
    details: LaserScan、PointCloud2、Image、CameraInfoとして、Sensor IDごとのTopicへpublishします。
    link: /parts/lidar
  - icon: ↻
    title: ロボティクス
    details: lease付きの型付きTopicで回転・直動軸とKSP標準ホイールを制御します。
    link: /parts/motors
  - icon: ▲
    title: 推進系
    details: KSP標準のエンジンとRCSを自動検出し、個別推力、メインスロットル、6軸入力を制御します。
    link: /parts/propulsion
  - icon: ◇
    title: 機体I/O
    details: Body Wrench、Ground Truth、パーツ単位の型付きアクチュエータ、プロキシURDFとTFを公開します。
    link: /api/vehicle-control
---

## PyLoNで開発を始める

PyLoNを使って、KSPのセンサーデータを処理するROS2ノードや機体を制御するアプリケーションを作るためのドキュメントです。

1. [Getting Started](/guide/getting-started)で環境を準備し、機体情報と点群の受信を確認します。
2. [ROS2アプリケーションを作る](/guide/application-development)で、Topicの購読と制御ノードの接続方法を確認します。
3. [Topic一覧](/api/topics)とパーツ別APIで、必要なメッセージ型・単位・座標系・動作条件を調べます。

## デモ

[デモ一覧](/demos/)から、機体の準備・起動・動作確認・停止の手順を確認できます。

- [軌道上のデブリ周回・撮影](/demos/debris-orbit)
- [2D LiDARとSLAM](/demos/lidar-slam)：地図作成・保存・Nav2走行
- [月面Nav2](/demos/mun-nav2)

## APIリファレンス

- [Topic一覧](/api/topics)：センサー・機体状態・アクチュエータの入出力
- [機体制御](/api/vehicle-control)：制御権の取得、Wrench指令、Ground Truth
- [機体モデルとTF](/api/vessel-model)：URDFの受信とRViz表示
- [Bridge起動オプション](/reference/bridge-options)・[パーツ設定](/reference/part-config)：通信先と各機能の設定
- [トラブルシュート](/reference/troubleshooting)：導入・運用時の確認手順

## PyLoN本体への貢献

PyLoNのMOD・bridge・メッセージ定義・ドキュメントを変更する手順は、末尾の[PyLoN本体への貢献](/contributing/)にまとめています。
