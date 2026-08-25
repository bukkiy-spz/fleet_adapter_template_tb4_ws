# tb4_20260612 RMF Integration Notes

このディレクトリは、`traffic-editor` で作成した
`tb4_20260612.building.yaml` を元に生成した RMF 用アセットです。

## 生成済みファイル

- `tb4_20260612.building.yaml`
- `nav_graphs/0.yaml`
- `world/tb4_20260612.world`
- `models/tb4_20260612_L1/...`

## 現在の place / charger

- `robot6_charger`
- `robot6_predock`
- `robot5_charger`
- `robot5_predock`
- `robot2_predock`
- `robot2_charger`
- `LP1`
- `LP2`
- `LP3`
- `LP4`

## nav graph / world 再生成コマンド

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

ros2 run rmf_building_map_tools building_map_generator nav \
  ~/rmf_main_ws/maps/tb4_rebuild_20260612/tb4_20260612.building.yaml \
  ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs
```

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash

ros2 run rmf_building_map_tools building_map_generator gazebo \
  ~/rmf_main_ws/maps/tb4_rebuild_20260612/tb4_20260612.building.yaml \
  ~/rmf_main_ws/maps/tb4_rebuild_20260612/world/tb4_20260612.world \
  ~/rmf_main_ws/maps/tb4_rebuild_20260612/models
```

## fleet adapter 側の対応ファイル

- config:
  `~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml`
- adapter launcher:
  `~/fleet_adapter_template_tb4_ws/scripts/run_direct_adapter_tb4_20260612.sh`

## まだ必要なこと

`reference_coordinates.robot` の実測がまだ必要です。  
同じ順番で robot 側の座標を入れてから adapter を起動してください。

順番:

1. `robot6_charger`
2. `robot5_charger`
3. `robot2_charger`
4. `LP4`
5. `LP1`
6. `LP2`

## robot 側 reference_coordinates の埋め方

例として `robot2` の `amcl_pose` で埋める場合:

```bash
cd ~/fleet_adapter_template_tb4_ws
source /opt/ros/humble/setup.bash
source ~/rmf_main_ws/install/setup.bash
source ~/fleet_adapter_template_tb4_ws/install/setup.bash
source ~/turtlebot4_ws/scripts/robot2_env.bash
```

まず現在値を読む:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/record_reference_pose.py \
  --topic /robot2/amcl_pose \
  --label LP2
```

実際に config を更新する:

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint robot6_charger
```

同様に次を順番に埋めます。

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint robot5_charger
```

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint robot2_charger
```

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint LP4
```

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint LP1
```

```bash
python3 ~/fleet_adapter_template_tb4_ws/scripts/update_reference_from_amcl.py \
  --config ~/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml \
  --nav-graph ~/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml \
  --topic /robot2/amcl_pose \
  --waypoint LP2
```

## 起動例

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_schedule.sh
```

```bash
cd ~/fleet_adapter_template_tb4_ws
./scripts/run_direct_adapter_tb4_20260612.sh
```
