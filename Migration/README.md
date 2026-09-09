# PyLoNへの一括移行

PyLoNは旧KerbalLiDAR DLL、旧ROS2パッケージ、旧UDP形式をロードしません。`/ksp_vessel`配下の現行Topicは維持し、型は`pylon_interfaces/msg/...`へ更新します。所有権なしの指令は廃止しました。モーター・推進・ドッキングはleaseを取得して型付きcommandを送ります。

## 保存機体・セーブ

KSPを終了してから、まず変更予定を確認します。元ファイルはこのコマンドでは変更されません。

```bash
python3 Migration/pylon_migrate.py '/path/to/save'
python3 Migration/pylon_migrate.py '/path/to/save' --output '/path/to/converted-save'
```

変換対象は`.craft`・`.sfs`・`.cfg`です。構造を読み、自作パーツ・PartModule・Scenario名、接続・対称配置のパーツ参照、モデル参照を変換します。機体名、persistentId、他MODのフィールド、既存の正式なsensorIdを保持します。正式なsensorIdがない場合だけpartName／lidarNameから移します。廃止されたフィールドは削除します。

出力したファイルを、元セーブの**コピー**へ同じ相対パスで重ねて使用してください。出力は変換対象のファイルだけを含むため、サムネイル等を含むセーブ全体のコピーではありません。元セーブをバックアップとして保持します。

パーツごとの通信先とURDF設定は出力の`Runtime.cfg`へ集約します。このファイルはセーブ内ではなく、KSPの`GameData/PyLoN/Config/Runtime.cfg`として設置してください。複数の通信先、重複したSensor IDモジュール、異なる既存出力を検出した場合は書込み前に停止します。既存の正式なSensor IDと古い名前の食い違いは、旧runtimeと同じく正式なSensor IDを優先します。

同じ入力と出力の再実行は無変更です。`--in-place`は変更ファイルごとに日時付きバックアップを作りますが、共通設定の抽出が必要な入力には使えません。復元はバックアップを元のファイル名へコピーします。

## インストール

```bash
./sync.sh
./sync.sh --all-demos
source ~/ros2_ws/install/setup.bash
```

同期は旧`GameData/KerbalLiDAR`をKSP直下の`PyLoN-migration-backups`へ退避します。ROS2旧パッケージは`package.xml`で確認し、対応する`src`・`build`・個別`install`をワークスペース直下の`pylon-migration-backups`へ退避します。他プロジェクトは削除しません。標準の分離installワークスペースを対象とし、共有されたmerge-installを検出した場合は停止します。専用の新しいinstall prefixでビルドしてください。

新しいシェルでROS環境を読み直してください。旧環境をsource済みのシェルには古い探索パスが残る場合があります。PyLoNのRuntime.cfgは初回のみ配置し、以後の同期ではユーザー設定を保持します。

| 旧パッケージ | 新パッケージ |
|---|---|
| ksp_lidar_bridge | pylon_bridge |
| ksp_ros2_interfaces | pylon_interfaces |
| ksp_vehicle_control | pylon_vehicle_control |
| debris_orbit | pylon_demo_debris_orbit |
| position_estimator | pylon_demo_position_estimator |
| mun_rover_demo | pylon_demo_mun_rover |
| ksp_nav2_bringup | `pylon_demo_lidar_slam`（`./sync.sh --demo lidar_slam`で導入） |

旧OBJ追加モデルは廃止し、配布済みの自作パーツはネイティブモデルを使います。独自OBJを指定した設定は、先にネイティブ`MODEL`設定へ変換してください。

正式な対応表は`identifiers.json`です。移行コードは本番DLL・ROSノードからimportしません。
