# デモ開発から整理した設計

| 症状 | 原因 | 保持した既存対処 | 新しい責務 | 回帰確認 |
|---|---|---|---|---|
| 角速度・トルクの符号が姿勢と合わない | ベクトルと軸性ベクトルの混同 | 慣性系補正と左右の座標系の変換 | FrameConversionsへ変換・SI単位を集約 | 既知姿勢、姿勢差分、回転方向 |
| 遅延でIMU積分が跳ぶ | 到着時刻に合わせてoffsetを変更 | セッション内で時刻対応を固定 | SimulationClockとFlightService | 遅延、順序逆転、巻き戻り |
| リロード後に古いTF・推定が残る | 機能ごとに異なる世代管理 | runtime epochで状態を破棄 | 独立heartbeat、SessionTracker、一括リセット | 機体再ロード、bridge再起動、旧パケット拒否 |
| LiDARや真値がないと機体情報を取得しづらい | センサー・評価出力から設定とIDを取得 | 独立した機体モデル管理 | RuntimeSettings、モデル、セッションの各サービス | センサーなし・真値なしのlifecycle |
| 撮影中にゲーム画面の品質が変わる | 共有描画リソースへの干渉 | 画面サイズの中間描画、状態復元、既存MOD連携 | 撮影、SourceCamera、RgbCaptureResources | サイズ変更、描画失敗、復元、複数センサー |
| 制御クラスが状態配信も抱える | 取得と書込みの混在 | 同じ機体・車輪情報と単位 | ActuatorTelemetry、VesselTelemetry、VesselParts | 型付き状態、車輪geometry、実機到達 |
| デモ間importと停止処理が増える | 共通処理が個別デモに所在 | 数値処理とローバー停止経路 | pylon_perception、LeaseCoordinator、停止保持 | 単体build、lease再発行、欠測・取消停止 |

lifecycleはKSPの`runtime_instance`・`runtime_epoch`・`runtime_generation`を保持します。ROS側の`generation`はbridge再起動も識別し、受信再開で以前の推定・目標を再利用しません。詳細な観測記録と実機probeはローカルのDevelopmentに保持します。
