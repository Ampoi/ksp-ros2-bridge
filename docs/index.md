---
layout: home

hero:
  name: PyLoN API
  text: KSPをROS2ロボットへ
  tagline: センサー、モーター、ホイール、推進系、機体WrenchとGround TruthをROS2から扱うための実装準拠リファレンス。
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

## まず確認するもの

このドキュメントは、KSPプラグインのC#実装、各`part.cfg`、ROS2 bridgeのPython実装、`pylon_interfaces`のメッセージ定義を照合して作成しています。最初に[Getting Started](/guide/getting-started)を実行し、次に[Topic一覧](/api/topics)から使用する入出力を確認してください。

::: tip kRPCは不要です
KSPプラグインとROS2 bridgeはUDP JSONで直接通信します。kRPCサーバーや`krpc`クライアントライブラリは使用しません。
:::
