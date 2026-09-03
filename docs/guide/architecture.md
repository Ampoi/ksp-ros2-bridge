# モノレポ設計

このリポジトリは、配置先ではなく責務の境界を基準に分けています。依存はadapterからapplication、applicationからdomainの内向きだけです。domainはKSP、Unity、ROS2、デモへ依存しません。

## Bounded context

| ディレクトリ | 責務 |
|---|---|
| `Source/KerbalLiDAR/Domain` | RCS配分、制御所有権、安全制限などの純粋なルール |
| `Source/KerbalLiDAR/Application` | KSP内で制御ユースケースを組み立てる層 |
| `Source/KerbalLiDAR/Api/Ksp` | KSP/Unity、UDP、PartModuleのadapter |
| `Ros2/ksp_ros2_interfaces` | KSPとROS2アプリ間の明示的な契約 |
| `Ros2/ksp_lidar_bridge/domain` | UDP packet検証とKSP時刻整合 |
| `Ros2/ksp_lidar_bridge/udp_bridge.py` | UDP・ROS2・QoS・TFのadapter |
| `Ros2/ksp_vehicle_control` | 再利用可能な6DoF制御domainとlease workflow |
| `Ros2/ksp_nav2_bringup` | SLAM/Nav2固有のintegration adapter |
| `Demo` | 公開APIだけを使う実行例。共通制御は置かない |
| `Development` / `Tools` | build、sync、KSP live debugなどの開発専用ツール |

ルートの`dev_sync.sh`、`dev_debug.sh`、`dev_teleport.sh`は後方互換の薄い入口です。実装は`Development/commands`にあり、KSP modやROS2 packageにはインストールされません。

## 制御の境界

機体制御の整合性を決めるauthority aggregateはKSP process内にあります。controllerは`/ksp_vessel/lifecycle`から現在の`vessel_id`を取得し、その機体に対する期限付きleaseを獲得してから指令します。すべての正式なWrench・アクチュエータ指令は次を持ちます。

- KSP機体を特定する`vessel_id`
- 指令元を特定する`controller_id`
- 所有権を特定する`lease_id`
- replayや並び替わりを拒否する`sequence`

KSP側はSAS排他、timeout、角速度上限、Wrench変化率、連続噴射時間、emergency stopを最終安全境界として適用します。ROS nodeが停止してもこの制約は残ります。

RCSは「総最大推力で割る」方式ではなく、現在有効な各ノズルについて、KSPが実際に使う噴射軸、重心までのモーメントアーム、並進・回転axis enableを評価して12個の正負操作channelへ配分します。ただし、KSPのnormalized flight inputを経由するため厳密なforce sourceではありません。`WrenchFeedback`は`requested / allocated / achieved / residual`を分け、この差を観測可能にします。

## TFと機体ライフサイクル

| TF / Topic | 扱い |
|---|---|
| `ground_truth_enu -> base_link` | KSP universal time基準のdynamic TF |
| `base_link -> ksp_<vessel-id>_link_0000` | 重心変化を含むdynamic TF |
| proxyの固定joint | `/tf_static` |
| part link -> sensor frame | Sensor ID由来の安定名を持つ`/tf_static` |
| `/ksp_vessel/lifecycle` | `UNAVAILABLE / ACTIVE / CHANGED / STALE`と実`vessel_id` |

点群や画像のtimestampでは、同じKSP universal timeへGround Truth poseを外挿してdynamic TFを補います。固定取付姿勢をセンサー周期で再送しないため、TFのfuture extrapolationと不要なproxy再生成を避けます。

速度はworld frameの`/ground_truth/twist`とbody frameの`/ground_truth/twist_body`を別Topicで公開します。高レベルcontrollerはworld-frame setpointを受け、座標変換を共通実装内で一度だけ行います。
