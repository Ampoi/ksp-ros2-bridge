# Nav2月面ローバーデモ

Nav2連携は任意導入の`pylon_demo_mun_rover`に含まれます。3D LiDAR・IMU・車輪情報を使用し、推定・誘導・制御へ真値を入力しません。

```bash
./sync.sh --demo mun_rover
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_mun_rover demo.launch.py lidar_sensor_id:=front_lidar
```

機体は通常のゲーム操作で準備します。前輪操舵、接地、電源、LiDARの視野を確認し、Ready表示後にRVizのNav2 Goalを指定してください。取消・センサー欠測・制御権喪失で停止し、古いgoalを自動再開しません。真値による評価は`evaluate:=true`で独立した評価ノードだけに接続します。

デモのREADMEに機体条件・起動引数・検証基準を記載しています。本体bridgeにはNav2依存はありません。
