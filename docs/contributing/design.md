# 設計と検証の観点

PyLoN本体を変更するときに保つ責務と、確認する動作をまとめています。

| 対象 | 責務 | 変更時の確認 |
|---|---|---|
| 座標と単位 | `FrameConversions`に座標変換とSI単位への変換を集約 | 既知姿勢、姿勢差分、角速度とトルクの回転方向 |
| シミュレーション時刻 | `SimulationClock`と`FlightService`でセッション内の時刻対応を管理 | 通信遅延、順序逆転、時刻の巻き戻り |
| セッション | heartbeatと`SessionTracker`で世代変更時に状態を初期化 | 機体再ロード、bridge再起動、以前のセッションのパケット受信 |
| 機体情報 | 設定・モデル・セッションの各サービスで機体IDとモデルを管理 | センサー非搭載時とGround Truth無効時のlifecycle |
| 撮影 | 撮影処理、`SourceCamera`、`RgbCaptureResources`で描画とリソースを管理 | 解像度変更、描画失敗時の復元、複数センサー、描画MODとの組合せ |
| 状態取得 | `ActuatorTelemetry`、`VesselTelemetry`、`VesselParts`で状態を取得 | 型付き状態、車輪geometry、Topic上の単位と値 |
| 共通制御とデモ | `pylon_perception`に共通の認識処理、`LeaseCoordinator`にlease管理を配置 | パッケージ単体のビルド、lease再発行、欠測・取消時の停止 |

lifecycleはKSPの`runtime_instance`・`runtime_epoch`・`runtime_generation`を保持します。ROS側の`generation`はbridge再起動も識別します。世代変更時には推定・目標を破棄し、新しいセッションで取得し直します。
