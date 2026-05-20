# TB4 Simple Map

現在の `TB4` 実機運用では、`robot2_charger / LP1 / LP2 / LP3` の 4 点を使って  
`RMF` 座標と `robot2` の map 座標をほぼ同一として扱う簡易構成を使っています。

## 1. 使っているファイル

- 実機保存 map:
  - `~/maps/robot2_map.yaml`
  - `~/maps/robot2_map.pgm`
- RMF 側の同期先:
  - `~/rmf_main_ws/maps/tb4/robot2_map_latest.yaml`
  - `~/rmf_main_ws/maps/tb4/robot2_map_latest.pgm`
- nav graph:
  - `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- adapter config:
  - `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`

## 2. 4 点の対応

現在の `reference_coordinates` は次の 4 点でそろえてある。

```text
robot2_charger: [-1.713150, -0.487242]  # yaw=0.093749
LP1:            [-2.398943, -0.159392]
LP2:            [-2.314348, -0.370411]
LP3:            [-2.354361, -0.368523]
```

このため、現状の `rmf -> robot` 変換は identity にかなり近い。

## 3. 今作った map を RMF 側へコピーする

```bash
cd ~/fleet_adapter_template_tb4_ws
python3 scripts/sync_robot_map_to_rmf.py --also-latest
```

## 4. nav graph と map の重なりを確認する

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
python3 scripts/plot_tb4_map_navgraph.py \
  --topic /robot2/amcl_pose \
  --save ~/obs_recording/tb4_newmap_navgraph_overlay.png
```

## 5. いつ見直すべきか

次のどれかが変わったら、simple map 前提を見直す。

- waypoint の位置を Traffic Editor で動かした
- 実機 map を大きく作り直した
- `LP1 / LP2 / LP3` に dispatch すると目視でずれる

そのときは最低でも次の 3 つを同時に更新する。

- `~/rmf_main_ws/maps/tb4/nav_graphs/1.yaml`
- `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml`
- `~/rmf_main_ws/maps/tb4/tb4_20260518.building.yaml` または新しい `.building.yaml`
