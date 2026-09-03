# ドッキングポート

Flight中のactive vesselにある全`ModuleDockingNode`を自動検出します。KSP標準ポートだけでなく、同じモジュールを使う互換パーツも専用設定なしで対象になります。

ポート名は`docking_port_<persistentId>_<moduleIndex>`です。実際の名前は次で確認できます。

```bash
ros2 topic list | grep docking_ports
```

## Topic

| 方向 | Topic | 型 | 内容 |
|---|---|---|---|
| 出力 | `/ksp_vessel/docking_ports/<id>/state` | `ksp_ros2_interfaces/msg/DockingPortState` | 接続・捕捉・切離し可否とカメラ選択状態。10 Hz、Best Effort |
| 入力 | `/ksp_vessel/docking_ports/<id>/command` | `ksp_ros2_interfaces/msg/DockingPortCommand` | カメラ選択・停止・切離し。Reliable |
| 出力 | `/ksp_vessel/docking_ports/<id>/camera/image_raw` | `sensor_msgs/msg/Image` | 選択中ポートの320×240、5 Hz、`rgb8`画像 |
| 出力 | `/ksp_vessel/docking_ports/<id>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 垂直FOV 60度のpinhole内部パラメーター |

`state`にはKSPの生のstate文字列に加え、`docked`、`acquiring`、`releasable`、`camera_active`、接続相手の名前とpart IDが入ります。

## カメラを切り替える

Flight開始時とactive vessel切替時は、persistent ID順の先頭ポートが自動選択されます。同時に配信するポートは1個だけです。

```bash
ros2 topic pub --once \
  /ksp_vessel/docking_ports/docking_port_12345_2/command \
  ksp_ros2_interfaces/msg/DockingPortCommand \
  '{action: 1, sequence: 1}'
```

`action: 1`は`SELECT_CAMERA`、`action: 2`は`STOP_CAMERA`です。停止後は自動的に再選択されず、別の`SELECT_CAMERA`を受けるまで画像を配信しません。ポートがactive vesselから消えた場合は残っている先頭ポートへ自動的に切り替えます。

カメラはドッキングノードの開口方向を向きます。shielded/inlineポートが閉じている場合は閉鎖状態がそのまま映ります。画像と`CameraInfo`は同じtimestampとREP-103 optical frame（+X右、+Y下、+Z前方）を共有します。

## Undock / Decouple

```bash
ros2 topic pub --once \
  /ksp_vessel/docking_ports/docking_port_12345_2/command \
  ksp_ros2_interfaces/msg/DockingPortCommand \
  '{action: 3, sequence: 2}'
```

`RELEASE`は現在KSPで有効な`Undock`を優先し、なければ`Decouple`を実行します。`releasable: false`の状態では何も実行しません。同じポートで過去に処理した値以下の`sequence`は、UDP再送による重複切離しを防ぐため無視されます。`sequence: 0`をbridgeへ送った場合はbridgeが正の連番を割り当てます。

相対Pose/Twist、ターゲット指定、Control From HereはこのAPIには含まれません。
