# TB4 Fleet Adapter Troubleshooting

このファイルは、`fleet_adapter_template_tb4_ws` で `RMF -> adapter -> robot2` を通すときの切り分けメモです。

## 1. まず direct Nav2 を通す

adapter より前に、`robot2` 単体で次が通ることを確認する。

```bash
cd ~/turtlebot4_ws
source scripts/robot2_env.bash
timeout 5 ros2 topic echo /robot2/amcl_pose --once
ros2 action list | grep /robot2/navigate_to_pose
```

ここが通らない間は `RMF` 側ではなく `turtlebot4_ws` 側を先に直す。

## 2. `Unable to initialize fleet adapter yet; waiting for RMF schedule node discovery...`

原因:

- `schedule` がまだ起動していない
- `schedule` と `adapter` の環境がずれている

対処:

1. `./scripts/run_direct_schedule.sh` を先に起動する
2. その端末を閉じない
3. 別端末で `./scripts/run_direct_adapter.sh` を起動する

## 3. `Successfully added new robot: robot2` が出ない

まず `RobotAPI` 前提を確認する。

```bash
cd ~/turtlebot4_ws
source scripts/robot2_env.bash
timeout 5 ros2 topic echo /robot2/amcl_pose --once
timeout 5 ros2 topic echo /robot2/battery_state --once
ros2 action list | grep /robot2/navigate_to_pose
```

これが通らないと adapter から robot を登録できない。

## 4. adapter が charger から始まったことになってしまう

過去の問題:

- config の `start.waypoint` が `robot2_charger`
- でも実機は別の場所にいる
- adapter が charger から始まったと誤認する

現状:

- `fleet_adapter.py` を修正済み
- 実機が configured start waypoint から `1.0m` 以上離れているときは、live pose へ自動 fallback する
- `compute_plan_starts()` の merge 距離は
  `max_merge_waypoint_distance=2.5` / `max_merge_lane_distance=2.5`
  に広げてある
- それでも merge できないときは nearest waypoint fallback を試す

adapter ログの確認点:

- `Live robot pose [robot2] ...`
- `Using the live pose instead.`
- `Running compute_plan_starts for robot: robot2 (waypoint_merge=..., lane_merge=...)`
- `Falling back to nearest waypoint [...]`

## 5. `Unable to determine StartSet for robot2`

まず本当に最新版の adapter を使っているか確認する。

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
```

そのうえで、実機が nav graph からあまりに離れていないかを map overlay で見る。

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
python3 scripts/plot_tb4_map_navgraph.py \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_newmap_navgraph_overlay.png
```

## 6. `Coordinate transformation error` は小さいのに位置がずれる

まず map と nav graph の見た目を重ねて確認する。

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
python3 scripts/plot_tb4_map_navgraph.py \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_newmap_navgraph_overlay.png
```

ズレるときの確認対象:

- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `reference_coordinates`

## 7. `/fleet_states` が出ない

現状の adapter は、最初の robot 登録が終わってから fleet state publish を有効化する。

そのため、まずは adapter ログでこれを確認する。

- `Successfully added new robot: robot2`
- `Enabled fleet state topic publishing at 1.00 s`

確認コマンド:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
timeout 5 ros2 topic echo /fleet_states --once
```

## 8. `dispatch_go_to_place` は通るが robot が動かない

切り分け順:

1. `ros2 action list | grep /robot2/navigate_to_pose`
2. `ros2 topic echo /robot2/amcl_pose --once`
3. `ros2 topic echo /fleet_states --once`
4. adapter ログで `navigate` や `failed to reach its target` を確認

direct Nav2 goal が動かなければ adapter の問題ではない。

## 9. `Requesting new schedule update because update timed out`

この warning が出続けるときは、`schedule` と `adapter` を両方止めて helper script から起動し直す。

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

## 10. helper script で `AMENT_TRACE_SETUP_FILES` や `COLCON_TRACE` の未定義エラー

現状:

- `run_direct_schedule.sh`
- `run_direct_adapter.sh`
- `run_direct_dispatch_go_to_place.sh`

には `set -u` と ROS setup の相性回避ガードを入れてある。

もし同じ症状が再発したら、古い shell を開きっぱなしにせず新しい terminal で再実行する。

## 11. map コピーが古い

今作った map を `rmf_main_ws` へ反映し忘れると、確認用の overlay や将来の Traffic Editor 作業が古いままになる。

同期:

```bash
cd ~/fleet_adapter_template_tb4_ws
python3 scripts/sync_robot_map_to_rmf.py --also-latest
```
