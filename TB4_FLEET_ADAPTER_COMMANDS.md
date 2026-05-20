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

## 3. nav graph と map の重なりを確認する

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
python3 scripts/plot_tb4_map_navgraph.py \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_newmap_navgraph_overlay.png
```

現状の本線:

- nav graph: `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- adapter config: `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `reference_coordinates` は `robot2_charger / LP1 / LP2 / LP3` の 4 点で identity に近い設定
- `config.yaml` の `rmf_config.start.max_merge_waypoint_distance` と
  `max_merge_lane_distance` は `2.5` にしてあり、graph 近傍の live pose を
  `compute_plan_starts()` へ merge しやすくしてある

## 4. robot2 側の前提を満たす

`~/turtlebot4_ws` 側で次が通っていることを確認する。

```bash
cd ~/turtlebot4_ws
source scripts/robot2_env.bash
timeout 5 ros2 topic echo /robot2/amcl_pose --once
ros2 action list | grep /robot2/navigate_to_pose
timeout 5 ros2 topic echo /robot2/battery_state --once
```

## 5. direct schedule を起動する

別端末:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_schedule.sh
```

## 6. direct adapter を起動する

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
- 必要に応じて `Using the live pose instead.` や
  `Falling back to nearest waypoint ...` が出る

## 7. adapter 登録を確認する

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

## 8. RMF から実機を動かす

別端末:

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_dispatch_go_to_place.sh LP1
```

引数:

- 第1引数: waypoint 名。例 `LP1`
- 第2引数: robot 名。省略時 `robot2`
- 第3引数: fleet 名。省略時 `tb4_fleet`

純粋な CLI で投げる場合:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash

ros2 run rmf_demos_tasks dispatch_go_to_place -F tb4_fleet -R robot2 -p LP1
```

## 9. 停止順

1. `dispatch` terminal
2. `adapter`
3. `schedule`
4. 必要なら `turtlebot4_ws` 側の `Nav2` / `localization`

## 10. nav graph を作り直す場合

現在の occupancy map を `Traffic Editor` に読み直して waypoint を再配置する場合は、最低でも次の 3 箇所をそろえる。

- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/tb4_20260518.building.yaml` または新しく作った `.building.yaml`
