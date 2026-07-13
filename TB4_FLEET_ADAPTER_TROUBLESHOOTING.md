# TB4 Fleet Adapter Troubleshooting

このファイルは、`fleet_adapter_template_tb4_ws` で `RMF -> adapter -> robot2` を通すときの切り分けメモです。

## 1. `Unable to initialize fleet adapter yet; waiting for RMF schedule node discovery...`

原因:

- `schedule` がまだ起動していない
- `schedule` と `adapter` の環境がずれている

対処:

1. `./scripts/run_direct_schedule.sh` を先に起動する
2. その端末を閉じない
3. 別端末で `./scripts/run_direct_adapter.sh` を起動する

## 2. `Successfully added new robot: robot2` が出ない

```bash
cd ~/turtlebot4_ws
source scripts/robot2_env.bash
timeout 5 ros2 topic echo /robot2/amcl_pose --once
timeout 5 ros2 topic echo /robot2/battery_state --once
ros2 action list | grep /robot2/navigate_to_pose
```

これが通らないと adapter から robot を登録できない。

## 3. `Unable to determine StartSet for robot2`

まず adapter を build し直す。

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
```

そのうえで overlay を確認する。

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/plot_tb4_map_navgraph.py \
  --use-robot-frame \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_20260521_overlay_robot_frame_latest.png
```

## 4. `worldToMap failed` / `goal is off the global costmap`

原因:

- `reference_coordinates` が実機 map と合っていない
- charger / LP の robot 側実測値が古い

確認:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/analyze_reference_coordinates.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml
```

必要なら再実測:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label robot2_charger
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label pre_dock
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP1
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP2
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP3
```

## 5. Marker は出ているのに RViz で waypoint が見えない

原因:

- Marker が RMF 座標のまま出ている
- 実機 `map` 上では画面外

対処:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/publish_nav_graph_markers.py \
  --use-robot-frame
```

## 6. `python3 scripts/...` が見つからない

原因:

- `source ~/turtlebot4_ws/scripts/robot2_env.bash` の後で CWD が `~/turtlebot4_ws` に変わる

対処:

- `python3 ~/fleet_adapter_template_tb4_ws/scripts/...` のように絶対パスで実行する
- または source 後に `cd ~/fleet_adapter_template_tb4_ws` し直す

## 7. `dispatch_go_to_place` で `L1` を投げて失敗する

原因:

- `L1` は階名で waypoint 名ではない

有効な place 名:

- `LP1`
- `LP2`
- `LP3`
- `pre_dock`
- `robot2_charger`

## 8. 毎回 charger に戻ろうとしてから目標へ行く / 目標後に pre_dock に戻る

原因は `Nav2` 単体ではなく、現状の RMF 設定と adapter 実装の組み合わせ。

現状:

- `finishing_request: park`
- charger lane は `pre_dock -> robot2_charger`
- `dock()` / `start_process()` は実機 docking 未実装

そのため、見かけ上:

- charger 系を経由しようとする
- タスク後に `pre_dock` 側へ戻ろうとする

## 9. `/fleet_states` が出ない

確認:

- `Successfully added new robot: robot2`
- `Enabled fleet state topic publishing at 1.00 s`

コマンド:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
timeout 5 ros2 topic echo /fleet_states --once
```

## 10. `Requesting new schedule update because update timed out`

```bash
pkill -INT -f rmf_traffic_schedule || true
pkill -INT -f tb4_fleet_adapter || true
```

その後:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_schedule.sh
./scripts/run_direct_adapter.sh
```

## 11. map コピーが古い

```bash
cd ~/fleet_adapter_template_tb4_ws
python3 scripts/sync_robot_map_to_rmf.py --also-latest
```

確認:

```bash
ls -la ~/rmf_main_ws/maps/tb4
sed -n '1,40p' ~/rmf_main_ws/maps/tb4/robot2_map_latest.yaml
```
