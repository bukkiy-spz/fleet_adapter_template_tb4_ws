# TB4 Fleet Adapter Commands

このファイルは、`fleet_adapter_template_tb4_ws` から `robot2` 実機を `RMF` へつなぐときの本線手順です。  
前提は、`~/turtlebot4_ws` 側で `localization` と `Nav2` の direct goal が正常動作していることです。

## 1. ワークスペースを build する

```bash
cd ~/rmf_main_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmf_demos_tasks

cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
colcon build --packages-select tb4_fleet_adapter
```

## 2. 今作った地図を RMF 側へ同期する

```bash
cd ~/fleet_adapter_template_tb4_ws
python3 scripts/sync_robot_map_to_rmf.py --also-latest
```

出力先:

- `~/rmf_main_ws/maps/tb4/robot2_map_YYYYMMDD.yaml`
- `~/rmf_main_ws/maps/tb4/robot2_map_YYYYMMDD.pgm`
- `~/rmf_main_ws/maps/tb4/robot2_map_latest.yaml`
- `~/rmf_main_ws/maps/tb4/robot2_map_latest.pgm`

## 3. 現在使っている本線ファイル

- nav graph: `~/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml`
- building map: `~/rmf_main_ws/maps/tb4_rebuild_20260521/tb4_20260521.building.yaml`
- adapter config: `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- world: `~/rmf_main_ws/maps/tb4_rebuild_20260521/world/tb4_20260521.world`

## 4. 参照座標を確認する

`reference_coordinates` は 5 点を使う。

- `robot2_charger`
- `pre_dock`
- `LP1`
- `LP2`
- `LP3`

解析:

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
python3 scripts/analyze_reference_coordinates.py \
  --config src/tb4_fleet_adapter/config.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml
```

## 5. nav graph と map の重なりを確認する

画像で確認:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/plot_tb4_map_navgraph.py \
  --use-robot-frame \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_20260521_overlay_robot_frame_latest.png
```

RViz 上で waypoint / charger / lane を直接表示:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/publish_nav_graph_markers.py \
  --use-robot-frame
```

RViz 側:

- `Fixed Frame` を `map`
- `MarkerArray` display を追加
- topic を `/tb4/nav_graph_markers`

## 6. waypoint の robot 側実測値を取り直す

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label robot2_charger
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label pre_dock
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP1
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP2
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py --label LP3
```

## 7. robot2 側の前提を満たす

```bash
cd ~/turtlebot4_ws
source scripts/robot2_env.bash
timeout 5 ros2 topic echo /robot2/amcl_pose --once
ros2 action list | grep /robot2/navigate_to_pose
timeout 5 ros2 topic echo /robot2/battery_state --once
```

## 8. direct schedule を起動する

別端末:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_schedule.sh
```

## 9. direct adapter を起動する

別端末:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_adapter.sh
```

期待ログ:

- `Starting RMF adapter core`
- `Adding fleet handle for [tb4_fleet]`
- `RobotAPI connected=True`
- `Successfully added new robot: robot2`
- 必要に応じて `Using the live pose instead.`

## 10. adapter 登録を確認する

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 topic list | grep /fleet_states
ros2 node list | grep tb4_fleet
timeout 5 ros2 topic echo /fleet_states --once
```

## 11. RMF から実機を動かす

別端末:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_dispatch_go_to_place.sh LP1
```

純粋な CLI:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
ros2 run rmf_demos_tasks dispatch_go_to_place -F tb4_fleet -R robot2 -p LP1
```

現在有効な place 名:

- `LP1`
- `LP2`
- `LP3`
- `pre_dock`
- `robot2_charger`

`L1` は階名なので `dispatch_go_to_place` の引数には使えない。

## 12. 反復試験

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_tb4_rebuild_20260521_checks.sh
```

## 13. 現在の既知制約

- `finishing_request` は `park`
- charger lane は `pre_dock -> robot2_charger`
- `dock()` / `RobotClientAPI.start_process()` は実機 docking 未実装
- そのため、タスク後に charger / pre_dock 系へ戻ろうとする挙動が見えることがある

## 14. nav graph を作り直す場合

最低でも次の 3 箇所をそろえる。

- `~/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml`
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4_rebuild_20260521/tb4_20260521.building.yaml`
