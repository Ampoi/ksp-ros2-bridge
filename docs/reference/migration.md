# PyLoNへの移行

既存の機体・セーブをコピーし、リポジトリの`Migration/pylon_migrate.py`で変換してください。既定では変更予定だけを表示します。

```bash
python3 Migration/pylon_migrate.py '/path/to/save'
python3 Migration/pylon_migrate.py '/path/to/save' --output '/path/to/converted'
./sync.sh --all-demos
```

正式なSensor ID・機体名・他MODのデータを維持します。出力されたRuntime.cfgは`GameData/PyLoN/Config/Runtime.cfg`へ配置します。詳しいバックアップ・復元手順はリポジトリのMigration/README.mdを参照してください。

`/ksp_vessel`配下の現行Topicは維持します。型は`pylon_interfaces`、bridgeは`pylon_bridge`、共通制御器は`pylon_vehicle_control`です。所有権なしの旧指令を使っていたクライアントは、lease取得と型付きcommandへ移行してください。ドッキングcommandにもvessel_id・controller_id・lease_id・sequenceが必要です。

UDPはPyLoN v1へ一括更新します。KSP MODとROS2 bridgeを同時に更新してください。旧版との混在動作はサポートしません。
