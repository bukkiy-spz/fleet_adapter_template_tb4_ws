# TB4 Fleet Adapter Troubleshooting

このファイルは `fleet_adapter_template_tb4_ws` 用のメモです。

## 1. どの adapter を本命にするか

現時点では:

- `fleet_adapter_tb4_ws` より
- `fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter`

を本命にする方が現実的です。

理由:

- `RobotClientAPI.py`
- `RobotCommandHandle.py`
- `config.yaml`

がすでに TB4 実機向けに書き換わっているためです。

## 1-1. 現在の本線は direct ROS 構成

まずは direct ROS 構成で adapter 経路を単純化します。

ただし直結では、

- `robot2_env.bash` を読むと RMF schedule service が timeout しやすい
- 読まないと robot 実機の `/robot2/amcl_pose` や action に届きにくい

という衝突が残っています。

そのため運用方針は:

- まずは Host 側 `robot2_env.bash` で direct ROS を試す
- 直結が崩れる場合だけ TCP relay / Zenoh bridge を fallback に使う

です。

## 2. `Goal accepted` なのに robot が動かない

これは adapter より先に Robot 側 Nav2/costmap の問題であることが多いです。

まず Robot 側で直接試す:

```bash
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: -1.05, y: -0.55, z: 0.0}, orientation: {w: 1.0}}}}"
```

これで動かなければ adapter ではなく Robot 側を先に直します。

## 3. `reference_coordinates` はまだ仮値

今の `config.yaml` は、

- RMF 座標
- Robot 地図座標

の対応がまだ仮です。

そのため research 向けの本運用前には、最低 4 点以上の対応点を実測して埋める必要があります。

実測補助:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP1
```

## 4. nav graph 座標と robot 座標が違って見える

今の環境では:

- `nav_graph_1.yaml` の頂点座標
- `amcl_pose` の地図座標

がそのまま一致していない可能性があります。

これは adapter バグではなく、`reference_coordinates` 未較正の問題です。

## 4-1. `LP2` へ行くが、思った位置とずれる

典型症状:

- `dispatch_go_to_place -p LP2` で実機は動く
- しかし到達位置が `LP2` 想定からずれる

まず adapter 実行引数を確認:

```bash
ps -ef | rg "ros2 run tb4_fleet_adapter fleet_adapter|fleet_adapter -c|tb4_fleet_adapter/.*fleet_adapter"
```

次に変換誤差を確認:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

`scale` が 1.0 から大きく外れる、または `LP2` の `error_m` が大きい場合は、
`reference_coordinates` と nav graph の waypoint 座標が不整合です。

修正方針:

1. `robot2_charger/LP1/LP2/LP3` を実測し直す
2. 下記3ファイルで同じ座標値にそろえる
3. adapter を再起動する

- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/rmf_main_ws/maps/tb4/tb4_20260518.building.yaml` または対応する `.building.yaml`

今回の実測値（2026-05-18 最終）:

```text
robot2_charger: [-1.713150, -0.487242]  # yaw=0.093749
LP1:            [-2.398943, -0.159392]
LP2:            [-2.314348, -0.370411]
LP3:            [-2.354361, -0.368523]
```

補足:

- `fleet_adapter` は `-c` と `-n` を起動時に読む
- 設定更新後は停止/再起動が必須

## 5. Dock/charging は未実装

`RobotClientAPI.py` では現状:

- `start_process()`
- `docking_completed()`

は最小スタブです。

研究で必要なら次に実装する候補:

- Create3 dock command
- docking result polling
- charge completion 判定

## 6. 失敗 goal で adapter が固まりやすい

今回、最小限の改善として:

- goal request が終了
- かつ成功していない

ときに current waypoint を再試行する挙動を追加しました。

それでも復帰が弱い場合は、

- retry 回数制限
- planner 再計画要求
- stuck 判定

を追加すると良いです。

## 7. 先に安定化すべき順序

1. Robot 時刻同期
2. `/robot2/scan` の timestamp 正常化
3. `localization`
4. `map -> odom`
5. direct Nav2 goal
6. custom `fleet adapter`

## 8. 典型的な切り分け

### `amcl_pose` が出ない

- localization 側の問題

### `map -> odom` が不安定

- AMCL / TF / scan 側の問題

### direct Nav2 goal が動かない

- planner/controller/costmap 側の問題

### direct Nav2 は動くが adapter で動かない

- adapter 設定または座標変換の問題

### `Unable to initialize fleet adapter. Please ensure RMF Schedule Node is running`

schedule node が本当に死んでいる場合もありますが、
今回の環境では

- Host が `robot2_env.bash` を読んだ直結 ROS 構成
- RMF service 応答 timeout
- schedule node と adapter が別の ROS graph / discovery 環境にいる

が原因だった可能性が高いです。

まず schedule terminal のログで、schedule node が起動しているか確認します:

```text
Beginning traffic schedule node
```

重要:

- `ros2 run rmf_traffic_ros2 rmf_traffic_schedule` を別 terminal で先に起動する
- その terminal は閉じずに開いたままにする
- `schedule` と `adapter` は同じ `source` 順で起動する

この環境では `/rmf_traffic/heartbeat` が `ros2 topic echo` や `ros2 topic list`
で見えないことがあります。ROS CLI の heartbeat 確認より、adapter と同じ経路で
`Adapter.make()` を直接 probe します:

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

`adapter_ok True` なら schedule は adapter から見えています。
`Adapter.make()` は discovery のために最大60秒ほど待つことがあります。
`False` または assert が続く場合は、schedule node を止めて clean graph で起動し直します:

`tb4_fleet_adapter` 側では `Adapter.make()` が一瞬 `None` を返してもすぐ落ちないように、
最大90秒リトライする実装にしています。起動直後に次の warning が出る場合は、
schedule discovery 待ちなのでしばらく待ちます:

```text
Unable to initialize fleet adapter yet; waiting for RMF schedule node discovery...
```

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

adapter も同じ clean graph で起動します:

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

ros2 run tb4_fleet_adapter fleet_adapter \
  -c ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  -n ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

この場合は schedule node を疑うより、

- RMF schedule / adapter 側で `robot2_env.bash` を使わない
- robot 接続は TCP relay の sender / receiver に任せる

構成へ切り替えます。

### Host 側で `/robot2/amcl_pose` が見えない

確認順:

1. robot 側 `localization` が起動している
2. robot 側 bridge config に `/robot2/amcl_pose` が含まれている
3. robot 側 bridge が `tcp/192.168.11.104:7447` に接続している
4. Host 側 `zenohd` が動いている
5. Host 側 bridge が起動している

Humble で robot 側 bridge が `/robot2/amcl` を発見しない場合は、
`zenoh-bridge-ros2dds` の discovery 依存を避けて fallback relay を使います。

Robot 側:

```bash
ros2 run tb4_square ros_topics_to_zenoh \
  --ros-args \
  -p zenoh_config:=/home/ubuntu/zenoh_bridge/zenoh_client_config.json5
```

Host 側:

```bash
ros2 run tb4_fleet_adapter zenoh_topics_to_ros \
  --zenoh-config ~/jazzy_ff_ws/config/zenoh/zenoh_client_config.json5
```

### Host 側で `/robot2/navigate_to_pose` が見えない

確認順:

1. robot 側 `Nav2` が起動している
2. robot 側で `ros2 action list | grep navigate_to_pose`
3. robot 側 bridge config に `action_servers: [".*/robot2/navigate_to_pose"]` がある
4. Host 側 bridge が同じ allowlist を持っている

### Host 側 `zenoh-bridge-ros2dds` が `実行形式エラー` になる

これはたいてい CPU アーキテクチャ違いです。

- `misc-ubu1` は `x86_64`
- robot 側 binary は `aarch64`

なので Host 側には別途 x86_64 版が必要です。

現時点の Host 側配置:

- `~/Downloads/zenoh_bridge_extract_x86_64/zenoh-bridge-ros2dds`

確認:

```bash
uname -m
file ~/Downloads/zenoh_bridge_extract_x86_64/zenoh-bridge-ros2dds
~/Downloads/zenoh_bridge_extract_x86_64/zenoh-bridge-ros2dds --version
```

### Host 側 adapter は起動するが robot 追加に進まない

よくある原因:

- Host 側 `/robot2/amcl_pose` が来ていない
- `reference_coordinates` が仮値のまま
- robot 側 Nav2 が direct goal で不安定

### Host から robot graph は見えるが adapter では RMF schedule が壊れる

`robot2_env.bash` は robot 直結には便利ですが、RMF schedule service と相性が悪いことがあります。

この場合は Host 内で役割を分けます。

- sender terminal:
  - `robot2_env.bash` を読む
  - `/robot2/amcl_pose` と `/robot2/battery_state` を TCP で送る
- receiver / adapter terminal:
  - `robot2_env.bash` を読まない
  - TCP で受けた data を `/robot2/amcl_pose_local` と `/robot2/battery_state_local` として publish する

receiver terminal:

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

sender terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run tb4_fleet_adapter host_robot_topic_tcp_sender
```

確認:

```bash
timeout 5 ros2 topic echo /robot2/amcl_pose_local --once
```

`host_robot_topic_relay` は同じ ROS graph 内で relay するだけなので、
clean adapter terminal から見えない場合があります。その場合は上の TCP relay を使います。

## 9. 2026-05-18に実際に出たエラーと対処

### `ValueError: generate_crs must be defined in wgs84 maps`

`traffic-editor` で作った `.building.yaml` が `coordinate_system: wgs84` の場合は、
`parameters.generate_crs` が必須です。以下を追加して再生成します:

```yaml
parameters:
  generate_crs: [1, EPSG:3857]
  suggested_offset_x: [3, 0]
  suggested_offset_y: [3, 0]
```

```bash
ros2 run rmf_building_map_tools building_map_generator nav \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/tb4_rebuild_20260518.building.yaml \
  ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs
```

### `cp ... nav_graphs/1.yaml: No such file or directory`

`graph_idx: 0` で lane を作ると出力は `0.yaml` です。
またコピー先 `~/rmf_main_ws/maps/tb4/nav_graphs` が無い場合も失敗します。

```bash
mkdir -p ~/rmf_main_ws/maps/tb4/nav_graphs

cp ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs/0.yaml \
  ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
cp ~/rmf_main_ws/maps/tb4_rebuild_20260518/nav_graphs/0.yaml \
  ~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml
```

### `Robot is out of bounds of the costmap!`

地図範囲外に実機がいる状態です。`initialpose` だけでは根本解決しない場合があります。

対処:

1. 一旦 `localization/nav2` を再起動
2. 地図内に実機を置いて `initialpose` 再投入
3. 近距離 direct goal で単体確認
4. まだ出るなら SLAM 取り直し（外周余白を確保）

### `failed to send response to /rmf_traffic/register_query (timeout)`

`schedule` の応答前にクライアント側が timeout した警告です。
単発なら様子見可、連発なら環境不一致を疑います。

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
```

上の環境を `schedule / adapter / api-server` で揃えて再起動します。

### `map_saver_cli` で `Failed to spin map subscription`

`/map` を待っているが、実際の topic が `/robot2/map` のときに発生します。

対処:

```bash
source /opt/ros/humble/setup.bash
turtlebot4-source

ros2 run nav2_map_server map_saver_cli \
  -f /home/ubuntu/maps/tb4/tb4_map_20260518 \
  --ros-args -r map:=/robot2/map
```

補足:

- `SLAM` が停止していると map は保存できない
- `ros2 topic list` で `/robot2/map` を確認してから実行すると確実

## 10. Traffic Editor編集時チェックリスト

### 保存前

- `robot2_charger` が waypoint 名として存在する
- `is_charger: true` が charger waypoint に付いている
- `LP1/LP2/LP3` が作成されている
- lane が接続されている（孤立点なし）
- `LP2/LP3` など近接点は必要なら少し離す
- lane の `start_idx / end_idx` は、GUI で選んだ vertex の番号として確認する

### 生成前

- `.building.yaml` の `coordinate_system` を確認
- `wgs84` なら `parameters.generate_crs` を定義
- `graph_idx` は lane の属性。見えていない場合でも `0` 扱いのことがある
- `graph_idx = 0` なら `0.yaml` が出る

### 反映時

- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml` を更新
- 必要なら `~/rmf_main_ws/maps/tb4/tb4_20260518.building.yaml` も一緒に見直す
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml` を更新
- `analyze_reference_coordinates.py` で `scale ~ 1.0` / `error_m` を確認
- `schedule` と `adapter` を再起動

## 11. `schedule` には繋がるのに `Segmentation fault` する

症状:

- `Registering to query topic ...`
- `Mirror handling new sync ...`
- 直後に `Segmentation fault`

2026-05-18 時点では、`fleet state` topic publish のタイミングが早すぎると
`rmf_fleet_adapter` 側で落ちるケースがあった。

対処:

- `tb4_fleet_adapter/fleet_adapter.py` で起動直後の
  `fleet_state_topic_publish_period()` を無効化
- 最初の robot 登録後に有効化
- `tb4_fleet_adapter` を再 build

確認したいログ:

- `Creating RobotAPI for namespace [/robot2]`
- `Successfully added new robot: robot2`
- `Enabled fleet state topic publishing at 1.00 s`

## 12. `Package 'rmf_demos_tasks' not found`

`rmf_main_ws` に source はあるが、install に入っていない状態。

対処:

```bash
cd ~/rmf_main_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmf_demos_tasks
source ~/rmf_main_ws/install/setup.bash
```

## 13. `Requesting new schedule update because update timed out`

今回のログでは、robot 登録までは成功している状態でもこの情報ログが出た。

まず確認すること:

1. `rmf_traffic_schedule` を 1 系統だけ起動しているか
2. `schedule` と `adapter` の両方で同じ `source` 順になっているか
3. `robot2_env.bash` を両方で読んでいるか

確認コマンド:

```bash
pgrep -af rmf_traffic_schedule
ps -ef | grep rmf_traffic_schedule
```

補足:

- `ros2 run ...` の親と実体プロセスが見えるので、`ps -ef` では 2 行に見えることがある
- `grep` 自身を含めると 3 行見えても異常とは限らない

## 14. `failed to send response to /rmf_traffic/register_query (timeout)`

これは `schedule` が返答を返そうとした時点で、adapter 側の待ち受けが timeout していた警告。
単発なら様子見可だが、連発する場合は環境不一致を疑う。

対処:

- 古い `schedule` / `adapter` を止める
- `schedule` を先に起動して残す
- 別 terminal で `adapter` を起動
- 両方で `robot2_env.bash` を読む

## 15. 今回の到達点

2026-05-18 時点では次が確認できた:

- `schedule` と `adapter` の接続
- `robot2` の RMF participant 登録
- `rmf_demos_tasks dispatch_go_to_place` による実機移動

残課題:

- `battery_state` 未取得時は最後の値で継続する
- `schedule update timeout` が残る場合は `schedule` 側ログも見る
- waypoint 間隔や向きにより Nav2 の recovery が増えることがある
