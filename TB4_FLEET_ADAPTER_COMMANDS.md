# TB4 Fleet Adapter Commands

このファイルは `fleet_adapter_template_tb4_ws` を本命にして進めるときのコマンド集です。
現時点では **Host PC から robot graph に直接入る ROS 直結構成** を本線にしています。
TCP relay / Zenoh bridge は fallback として残しています。

## 目的

- 将来の研究向けに、`free_fleet` ではなく custom `fleet adapter` 方向で進める
- ただし Robot 側 Nav2 が安定していることが前提
- まずは ROS 直結で adapter の経路を単純化する

## ワークスペース

- Adapter workspace:
  - `/home/masu_ubu/fleet_adapter_template_tb4_ws`
- RMF workspace:
  - `/home/masu_ubu/rmf_main_ws`
- TB4 workspace:
  - `/home/masu_ubu/turtlebot4_ws`
- direct robot env helper:
  - `/home/masu_ubu/turtlebot4_ws/scripts/robot2_env.bash`
- hybrid fallback config:
  - `/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_hybrid_tcp.yaml`

## 1. Host 側ビルド

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build
```

## 2. Host 側 source

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

## 2-1. Traffic Editorで地図を作り直す通し手順（`rmf_main_ws`）

### TE-1. 作業ディレクトリ準備

```bash
mkdir -p ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs
```

### TE-2. Traffic Editor起動

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
traffic-editor
```

### TE-3. 編集時のポイント

- `L1` レベルで map 画像を読み込む
- waypoint は最低 `robot2_charger/LP1/LP2/LP3` を作る
- lane を結ぶ
- `start_idx / end_idx` は lane の両端 vertex 番号
- `graph_idx` は lane の追加パラメータで、見えない場合は YAML 側で `0` のまま扱われることがある
- `robot2_charger` には `is_charger: true` を付ける
- 保存先例: `~/rmf_main_ws/maps/tb4_rebuild_20260518/tb4_rebuild_20260518.building.yaml`

### TE-4. `wgs84` の場合は `generate_crs` を定義

`coordinate_system: wgs84` なら、`.building.yaml` に以下を入れる:

```yaml
parameters:
  generate_crs: [1, EPSG:3857]
  suggested_offset_x: [3, 0]
  suggested_offset_y: [3, 0]
```

### TE-5. nav graph生成

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

ros2 run rmf_building_map_tools building_map_generator nav \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/tb4_rebuild_20260518.building.yaml \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs
```

### TE-6. `0.yaml/1.yaml` の扱い

- `graph_idx: 0` なら生成ファイルは `0.yaml`
- 運用先では `1.yaml` 名で置く場合がある
- GUI では lane の端点として `start_idx / end_idx` を見るのが正しく、`graph_idx` は lane の属性として持つ

```bash
mkdir -p ~/rmf_main_ws/maps/tb4/nav_graphs

cp ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs/0.yaml \
  ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### TE-7. adapter設定へ反映

更新対象:

- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/rmf_main_ws/maps/tb4/tb4_20260518.building.yaml` または対応する作業中 `.building.yaml`

整合チェック:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### TE-8. 再起動

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

この terminal は閉じずに開いたままにする。`Beginning traffic schedule node` が出てから adapter を起動する。

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

## 3. Host 側 `zenohd`

別ターミナル:

```bash
/root/zenoh_bin/zenohd
```

Host で直接バイナリが無い場合は、これまで使っていた container 側 `zenohd` を継続利用してもよいです。

## 4. Host 側 `zenoh-bridge-ros2dds`

別ターミナル:

```bash
source /opt/ros/humble/setup.bash

~/Downloads/zenoh_bridge_extract_x86_64/zenoh-bridge-ros2dds \
  -c ~/fleet_adapter_template_tb4_ws/config/zenoh/robot2_host_zenoh_bridge_ros2dds_client_config.json5
```

これは robot 側から来た

- `/robot2/amcl_pose`
- `/robot2/battery_state`
- `/robot2/navigate_to_pose`

を Host 側 ROS graph に再生するための bridge です。

## 5. package 確認

```bash
ros2 pkg executables tb4_fleet_adapter
```

期待:

```text
tb4_fleet_adapter fleet_adapter
```

## 6. Robot 側ログイン

```bash
ssh ubuntu@192.168.11.22
```

## 7. Robot 側生データ確認

Robot 側でまず時刻と生 topic を確認:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

date +%s
timeout 5 ros2 topic echo /robot2/scan --once
timeout 5 ros2 topic echo /robot2/tf --once
timeout 5 ros2 topic echo /robot2/odom --once
```

## 8. Robot 側 localization 起動

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=robot2 \
  use_sim_time:=false \
  map:=/home/ubuntu/maps/tb4/tb4_map.yaml
```

## 9. Robot 側 initial pose

Robot を `robot2_charger` 付近に置いてから、実測済みの charger pose を入れます。

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

timeout 3 ros2 topic pub /robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -1.260854, y: -0.556720, z: 0.0}, orientation: {z: 0.044027, w: 0.999030}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}" \
--rate 5 \
--qos-reliability best_effort
```

実測値:

- `x=-1.260854`
- `y=-0.556720`
- `yaw=0.088083 rad`
- quaternion 概算: `z=0.044027`, `w=0.999030`

## 10. Robot 側 localization 確認

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

timeout 5 ros2 topic echo /robot2/amcl_pose --once
timeout 10 ros2 run tf2_ros tf2_echo map odom --ros-args -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static
```

期待:

- `amcl_pose` が返る
- `map -> odom` が最終的に返る

## 11. Robot 側 Nav2 起動

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 launch turtlebot4_navigation nav2.launch.py \
  namespace:=robot2 \
  use_sim_time:=false
```

## 12. Robot 側 Nav2 確認

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 action list | grep navigate_to_pose
```

## 13. Robot 側 direct Nav2 goal

free_fleet や RMF に進む前に、Robot 側単体で goal 実行を確認:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: -1.05, y: -0.55, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 13-1. Host 側 direct robot env

Host PC の別ターミナルで:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
```

期待:

- `ROS_SUPER_CLIENT=True`
- `ROS_DISCOVERY_SERVER=192.168.11.22:11811;`

## 13-2. Host 側 direct topic / action 確認

同じ direct robot env ターミナルで:

```bash
timeout 10 ros2 topic echo /robot2/amcl_pose --once
ros2 action list | grep navigate_to_pose
```

期待:

- `/robot2/amcl_pose` が見える
- `/robot2/navigate_to_pose` が見える

## 13-3. Host 側 direct schedule node

別ターミナル。まずは direct 構成なので、schedule も同じ `robot2_env.bash` で揃えて試します:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

## 13-4. Host 側 direct adapter 起動

別ターミナル。同じく `robot2_env.bash` を読みます:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

現在の `config.yaml` は direct 用に:

- `pose_topic: /robot2/amcl_pose`
- `battery_topic: /robot2/battery_state`
- `navigate_action: /robot2/navigate_to_pose`

へ戻しています。

## 14. legacy hybrid / bridge 手順

以下は直結が不安定なときの fallback です。

### Robot 側 `zenoh-bridge-ros2dds`

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

/home/ubuntu/zenoh_bridge/zenoh-bridge-ros2dds \
  -c /home/ubuntu/zenoh_bridge/robot2_zenoh_bridge_ros2dds_client_config.json5
```

注:

- robot 側 config は Host 側で更新した
  `~/jazzy_ff_ws/config/zenoh/robot2_zenoh_bridge_ros2dds_client_config.json5`
  を robot 側へ同期して使う
- `amcl_pose` を橋渡し対象に含める
- endpoint は `tcp/192.168.11.104:7447`

## 15. Host 側で bridge 経由 topic / action 確認

`robot2_env.bash` は **読まない** で確認します。

```bash
source /opt/ros/humble/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

timeout 5 ros2 topic echo /robot2/amcl_pose --once
ros2 action list | grep navigate_to_pose
```

期待:

- `/robot2/amcl_pose` が Host 側で見える
- `/robot2/navigate_to_pose` が Host 側 action list に見える

## 15-1. bridge が robot ROS graph を見つけられない場合の fallback

Humble では `zenoh-bridge-ros2dds` が TurtleBot4 の Discovery Server に入れず、
robot 側 bridge ログに `/robot2/amcl` が出ないことがあります。

その場合は、Robot 側 package に依存せず、Host PC 上で2プロセスの TCP relay を使います。
`robot2_env.bash` を読む sender と、clean RMF graph に publish する receiver を分けるのがポイントです。

先に再ビルド:

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
```

Receiver 用ターミナル。これは adapter / RMF と同じ clean graph 側です:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

ros2 run tb4_fleet_adapter host_robot_topic_tcp_receiver
```

Sender 用ターミナル。これは robot graph を見るために `robot2_env.bash` を読みます:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run tb4_fleet_adapter host_robot_topic_tcp_sender
```

確認:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

timeout 5 ros2 topic echo /robot2/amcl_pose_local --once
timeout 5 ros2 topic echo /robot2/battery_state_local --once
```

この fallback は `/robot2/amcl_pose_local` と `/robot2/battery_state_local` 用です。
`/robot2/navigate_to_pose` action は、別途 bridge か直結 ROS 経路が必要です。

## 15-2. 非推奨: ROS graph 内 relay

以下の `host_robot_topic_relay` は、sender と publisher が同じ ROS graph に残るため、
clean adapter graph から `/robot2/amcl_pose_local` が見えないことがあります。
基本的には 15-1 の TCP relay を使います。

Relay 用ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run tb4_fleet_adapter host_robot_topic_relay
```

Adapter 用ターミナル:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

確認:

```bash
timeout 5 ros2 topic echo /robot2/amcl_pose_local --once
```

`config.yaml` はこの relay に合わせて:

- `pose_topic: /robot2/amcl_pose_local`
- `battery_topic: /robot2/battery_state_local`
- `navigate_action: /robot2/navigate_to_pose`

を使います。

## 16. RMF schedule node

別ターミナル。`robot2_env.bash` は読まず、必ず clean graph で起動します:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

起動ログに次が出れば schedule node 自体は起動できています:

```text
Beginning traffic schedule node
```

この環境では `/rmf_traffic/heartbeat` が `ros2 topic echo` で見えないことがあります。
adapter から見えるかは、別の clean terminal で直接 probe します:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

python3 - <<'PY'
import rmf_adapter as adpt
adpt.init_rclcpp()
adapter = adpt.Adapter.make("probe_adapter")
print("adapter_ok", bool(adapter))
PY
```

期待:

```text
adapter_ok True
```

注: `Adapter.make()` は discovery のために最大60秒ほど待つことがあります。
すぐ判断せず、`adapter_ok True` または `adapter_ok False` が出るまで待ちます。

## 17. Adapter 起動

別ターミナル。こちらも `robot2_env.bash` は読まず、schedule node と同じ clean graph で起動します:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

重要:

- ここでは `source ~/turtlebot4_ws/scripts/robot2_env.bash` を読まない
- robot 通信は `zenoh-bridge-ros2dds` に任せる
- RMF schedule と adapter は通常の Host ROS graph で動かす

## 18. Fleet 状態確認

```bash
ros2 topic list | grep fleet
ros2 topic echo /fleet_states --once
```

## 19. 参照座標の再較正

今の `Coordinate transformation error` と `scale` が不自然な場合は、
`reference_coordinates` のどれかがずれている可能性があります。

### 19-1. map と nav graph の可視化

Host PC で:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

python3 ~/fleet_adapter_template_tb4_ws/scripts/plot_tb4_map_navgraph.py \
  --topic /robot2/amcl_pose
```

### 19-2. 現在の reference_coordinates の残差確認

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py
```

目安:

- `scale` は 1.0 に近い方が自然
- 各 point の `error_m` は 0.3 m 未満が望ましい
- `omit ... omitted_error` が大きい point は怪しい

### 19-3. 各 waypoint で amcl_pose を再計測

Host PC で `robot2_env.bash` を読んだ terminal から:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py \
  --topic /robot2/amcl_pose \
  --label robot2_charger
```

同様に:

- `LP1`
- `LP2`
- `LP3`

を取り直します。

### 19-4. `LP2` ゴールで違う場所へ行くときの即時修正手順

症状:

- `dispatch_go_to_place -p LP2` で実機は動く
- ただし `LP2` 想定位置と実機到達位置がずれる

まず、現在の adapter が読んでいる `-c` / `-n` を確認:

```bash
ps -ef | rg "ros2 run tb4_fleet_adapter fleet_adapter|fleet_adapter -c|tb4_fleet_adapter/.*fleet_adapter"
```

次に、誤差を数値確認:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

`reference_coordinates` と nav graph の `robot2_charger/LP1/LP2/LP3` を同じ実測値にそろえる。
今回の実測値:

```text
robot2_charger: [-1.713150, -0.487242]  # yaw=0.093749
LP1:            [-2.398943, -0.159392]
LP2:            [-2.314348, -0.370411]
LP3:            [-2.354361, -0.368523]
```

対象ファイル:

- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`

更新後の再検証:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

期待:

- `scale: 1.000000`
- `error_m: 0.0000`（4点とも）

最後に adapter を再起動（`-c/-n` は起動時読込のため）:

```bash
ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### 19-5. hybrid fallback 設定

TCP relay に戻す場合は:

```text
~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_hybrid_tcp.yaml
```

を使います。

## 20. config の主要ファイル

- Adapter config:
  - `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- Hybrid fallback config:
  - `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_hybrid_tcp.yaml`
- Nav graph:
  - `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- Robot map:
  - `~/rmf_main_ws/maps/tb4/tb4_map.yaml`
- Host bridge config:
  - `~/fleet_adapter_template_tb4_ws/config/zenoh/robot2_host_zenoh_bridge_ros2dds_client_config.json5`
- Robot bridge config:
  - `~/jazzy_ff_ws/config/zenoh/robot2_zenoh_bridge_ros2dds_client_config.json5`

## 21. 起動順まとめ

1. Host 側 build
2. Host 側 source
3. Host 側 `zenohd`
4. Host 側 `zenoh-bridge-ros2dds`
5. Robot 側ログイン
6. Robot 側生データ確認
7. Robot 側 `localization`
8. Robot 側 `initialpose`
9. Robot 側 `amcl_pose` / `map -> odom` 確認
10. Robot 側 `nav2.launch.py`
11. Robot 側 direct Nav2 goal
12. Robot 側 `zenoh-bridge-ros2dds`
13. Host 側 bridge 経由 `/robot2/amcl_pose` / action 確認
14. Host 側 `rmf_traffic_schedule`
15. Host 側 `tb4_fleet_adapter fleet_adapter`
16. Host 側 fleet state 確認

## 22. 研究用途で先に詰めるべき点

1. `reference_coordinates` を実測で埋める
2. Robot 側 Nav2 の `rplidar_link` drop を減らす
3. Host bridge 経由で `/robot2/amcl_pose` / action が安定することを確認する
4. Dock / charging の振る舞いを `RobotClientAPI.py` に実装する
5. 失敗 goal 時の再送・復帰戦略を調整する

## 23. 参照座標の実測メモ取り

Robot を既知 waypoint に置いて、Host 側から現在の `amcl_pose` を YAML 形式で抜く:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py \
  --topic /robot2/amcl_pose \
  --label LP1
```

出力例:

```text
# LP1
- [-0.983347, -0.535228]  # yaw_rad=0.006997
```

これを `config.yaml` の `reference_coordinates.robot` 側へ集めていく。

## 24. `rmf_main_ws` + `rmf-web` 連携手順

`rmf-web` の `api-server` は `rmf_task_msgs/Alert` を要求するため、
この連携は `rmf_main_ws` を使う。

用語整理:

- `source` する RMF 環境: `~/rmf_main_ws/install/setup.bash`
- `rmf-web` ソース配置: `~/rmf_main_ws/rmf-web`

重要:

- `rmf_traffic_schedule`
- `tb4_fleet_adapter`
- `rmf-web api-server`

は同じ ROS graph / 同じ source 順で起動する。

### 24-1. 連携用 clean terminal 共通環境

各 terminal で最初に:

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

確認:

```bash
python3 - <<'PY'
from rmf_task_msgs.msg import Alert, AlertResponse
print("rmf_task_msgs Alert OK")
PY
```

### 24-2. adapter を `rmf_main_ws` で再ビルド

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
```

### 24-3. RMF schedule 起動

別ターミナル（24-1 を実行済み）:

```bash
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

### 24-4. TB4 adapter 起動

別ターミナル（24-1 を実行済み）:

```bash
ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### 24-5. `rmf-web` API server 起動

別ターミナル（24-1 を実行済み）:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

cd ~/rmf_main_ws/rmf-web/packages/api-server
pnpm start
```

注:

- `cd ~/rmf_main_ws/rmf-web/...` はコードの場所を指すだけ
- 実行時の ROS 環境は 24-1 の `rmf_main_ws` source を使い続ける
- `~/rmf_ws/install/setup.bash`（旧環境）は読まない

起動確認:

```text
Uvicorn running on http://0.0.0.0:8000
```

### 24-6. `rmf-web` Dashboard 起動

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

cd ~/rmf_main_ws/rmf-web/packages/rmf-dashboard-framework
pnpm start:example examples/demo
```

ブラウザ:

- `http://localhost:5173`
- `Tasks` タブから `go_to_place` 系タスクを投入

### 24-7. API 経由でタスク投入を直接確認

`api-server` terminal と同じ環境で:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

curl -sS -X POST http://localhost:8000/tasks/dispatch_task \
  -H 'Content-Type: application/json' \
  -d '{
    "type": "dispatch_task_request",
    "request": {
      "category": "compose",
      "description": {
        "category": "go_to_place",
        "phases": [
          {
            "activity": {
              "category": "go_to_place",
              "description": {
                "waypoint": "LP2"
              }
            }
          }
        ]
      }
    }
  }'
```

### 24-8. よくある失敗

- `ImportError: cannot import name 'Alert' from rmf_task_msgs.msg`
- 原因: `~/rmf_ws/install/setup.bash`（旧環境）を source している
- 対処: 該当 terminal を閉じ、24-1 から `~/rmf_main_ws/install/setup.bash` でやり直す

## 25. SLAMで地図を作り直す手順（実機ずれ対策）

`LP1/LP2/LP3` を合わせても実機の到達位置がずれる場合は、
occupancy map 自体が現場と合っていない可能性があるため、SLAM からやり直す。

### 25-1. Robot 側で SLAM 起動

Robot 側 terminal:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 launch turtlebot4_navigation slam.launch.py \
  namespace:=robot2 \
  use_sim_time:=false
```

### 25-2. 実機を手動走行して周囲を収集

- 壁沿いを一周して、ループ閉じを必ず作る
- 同じ場所を別角度で1回以上通る
- 鏡面/ガラス前はゆっくり

注:

- 持ち上げ移動中に SLAM を進めない
- 置いてから数秒待ってから再開する

### 25-3. 新しい map を保存

Robot 側 terminal（別 terminal で可）:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

mkdir -p /home/ubuntu/maps/tb4
ros2 run nav2_map_server map_saver_cli \
  -f /home/ubuntu/maps/tb4/tb4_map_20260518 \
  --ros-args -r map:=/robot2/map
```

生成物:

- `/home/ubuntu/maps/tb4/tb4_map_20260518.yaml`
- `/home/ubuntu/maps/tb4/tb4_map_20260518.pgm`

### 25-4. Host 側へ map を同期

Host 側 terminal:

```bash
scp ubuntu@192.168.11.22:/home/ubuntu/maps/tb4/tb4_map_20260518.yaml ~/rmf_main_ws/maps/tb4/
scp ubuntu@192.168.11.22:/home/ubuntu/maps/tb4/tb4_map_20260518.pgm  ~/rmf_main_ws/maps/tb4/
```

### 25-5. 新mapで localization / Nav2 を起動

Robot 側:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=robot2 \
  use_sim_time:=false \
  map:=/home/ubuntu/maps/tb4/tb4_map_20260518.yaml
```

続けて:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 launch turtlebot4_navigation nav2.launch.py \
  namespace:=robot2 \
  use_sim_time:=false
```

### 25-6. 新map基準で waypoint を再計測

Host 側（robot2_env を読んだ terminal）:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --topic /robot2/amcl_pose --label robot2_charger
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --topic /robot2/amcl_pose --label LP1
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --topic /robot2/amcl_pose --label LP2
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --topic /robot2/amcl_pose --label LP3
```

### 25-7. nav graph / config を更新

更新対象:

- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`

最初は `reference_coordinates.rmf` と `reference_coordinates.robot` を同じ実測値で合わせ、
`analyze_reference_coordinates.py` で誤差ゼロ確認してから dispatch を再開する。

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### 25-8. adapter 再起動と確認

```bash
ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

確認:

```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -F tb4_fleet -R robot2 -p LP1
```

### 25-9. Traffic Editor が `wgs84` で `generate_crs` エラーになる場合

`building_map_generator nav` 実行時に:

```text
ValueError: generate_crs must be defined in wgs84 maps
```

が出る場合は、`.building.yaml` の末尾に `parameters` を追加する。

```yaml
parameters:
  generate_crs: [1, EPSG:3857]
  suggested_offset_x: [3, 0]
  suggested_offset_y: [3, 0]
```

その後、再生成:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

ros2 run rmf_building_map_tools building_map_generator nav \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/tb4_rebuild_20260518.building.yaml \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs
```

### 25-10. `nav_graphs/1.yaml` が無い場合（`0.yaml` 出力）

`graph_idx: 0` で lane を作っていると、出力は `0.yaml` になる。

```bash
ls -la ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs
```

`0.yaml` の場合は運用ファイルへコピー時に `1.yaml` 名で配置する:

```bash
mkdir -p ~/rmf_main_ws/maps/tb4/nav_graphs

cp ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs/0.yaml \
  ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml

cp ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs/0.yaml \
  ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### 25-11. 2026-05-18 最終反映座標

今回の最終値（RMF/Robot とも一致で運用）:

```text
robot2_charger: [-1.713150, -0.487242]  # yaw=0.093749
LP1:            [-2.398943, -0.159392]
LP2:            [-2.314348, -0.370411]
LP3:            [-2.354361, -0.368523]
```

反映先:

- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`

### 25-12. `nav2` 範囲外警告と schedule timeout の即時対処

`Robot is out of bounds of the costmap!` が出る場合は、現在位置が map 範囲外。
まず `localization/nav2` を止め、地図内で `initialpose` を再投入して再起動する。

`failed to send response to /rmf_traffic/register_query (timeout)` は、
`schedule/adapter/api-server` の環境不一致で出やすい。以下を全 terminal で統一:

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

### 25-13. adapter 起動時に `Segmentation fault` していた件の対処

2026-05-18 時点では、`schedule` 発見直後に `fleet state` publish が先に走ると
`rmf_fleet_adapter` 側で落ちるケースがあった。

現在の `tb4_fleet_adapter/fleet_adapter.py` では、

- 起動直後の `fleet_state_topic_publish_period()` を一旦無効化
- 最初の robot 登録完了後に有効化

という流れに変更済み。

再ビルド:

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

### 25-14. `rmf_main_ws` で `rmf_demos_tasks` を使えるようにする

`Package 'rmf_demos_tasks' not found` が出るときは、`rmf_main_ws` で該当 package を build する。

```bash
cd ~/rmf_main_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmf_demos_tasks
source ~/rmf_main_ws/install/setup.bash
```

確認:

```bash
ros2 pkg list | grep rmf_demos_tasks
ros2 pkg executables rmf_demos_tasks | grep dispatch_go_to_place
```

### 25-15. 2026-05-18 時点の安定起動順

Terminal 1: schedule

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

Terminal 2: adapter

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

Terminal 3: dispatch

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash

ros2 run rmf_demos_tasks dispatch_go_to_place -F tb4_fleet -R robot2 -p LP1
```

### 25-16. `schedule` が複数に見えるときの確認

`ps -ef | grep rmf_traffic_schedule` では、通常:

- `ros2 run rmf_traffic_ros2 rmf_traffic_schedule`
- 実体の `rmf_traffic_schedule`
- `grep` 自身

の 3 行が見える。

見分けづらいときはこれを使う:

```bash
pgrep -af rmf_traffic_schedule
```

一旦全部止めて 1 系統に戻すなら:

```bash
pkill -f rmf_traffic_schedule
pkill -f tb4_fleet_adapter
```
