# One RMF Multi-Robot Plan

このメモは、`robot2` `robot5` `robot6` を

- 1つの traffic-editor マップ
- 1つの RMF nav graph
- 1つの fleet adapter

で扱うための前提をまとめたものです。

## 先に結論

`RMF を 1 つにする` と `robot が 3 台いる` こと自体は問題ありません。  
ただし今のまま

- `robot2 -> ROS_DOMAIN_ID=0`
- `robot5 -> ROS_DOMAIN_ID=55`
- `robot6 -> ROS_DOMAIN_ID=66`

で host が直DDS接続する構成だと、**1つの adapter process が3台へ同時接続することはできません。**

理由:

- `ROS_DOMAIN_ID` は 1 process で 1 つ
- `ROS_DISCOVERY_SERVER` も 1 process で 1 系統

だからです。

## では何をすればよいか

host 側で最終的に

- `/robot2/amcl_pose`
- `/robot5/amcl_pose`
- `/robot6/amcl_pose`
- `/robot2/navigate_to_pose`
- `/robot5/navigate_to_pose`
- `/robot6/navigate_to_pose`

が **同じ ROS graph / 同じ domain 上で見える状態** を作れば、  
1つの adapter から 3 台を扱えます。

## おすすめ構成

1. robot ごとに localization / Nav2 を実機側で起動
2. robot ごとに bridge / relay / Zenoh / TCP などで host 側へ必要 topic/action を転送
3. host 側では 3 台ぶんの topic/action を 1 つの ROS graph に再配置
4. `config_multi_robot_one_rmf_template.yaml` をベースに 1 本の adapter を起動

## この repo で今回入れた受け皿

- `src/tb4_fleet_adapter/tb4_fleet_adapter/fleet_adapter.py`
  - robot ごとに別 `RobotAPI` を持てるようにした
- `src/tb4_fleet_adapter/config_multi_robot_one_rmf_template.yaml`
  - `robot2/robot5/robot6` を 1 fleet に載せるための template

## traffic-editor 側で必要になる waypoint

少なくとも:

- `robot2_charger`
- `robot5_charger`
- `robot6_charger`
- 必要なら `pre_dock`
- `LP1`
- `LP2`
- `LP3`

`tb4_20260612` では、さらに次が入っています。

- `robot2_predock`
- `robot5_predock`
- `robot6_predock`
- `LP4`

## config に最終的に入れるもの

### robot ごと

- `api.pose_topic`
- `api.battery_topic`
- `api.navigate_action`
- `api.dock_action`
- `api.undock_action`
- `rmf_config.charger.waypoint`
- `manual_dispatch_gate_file`

### fleet 全体

- `reference_coordinates.rmf`
- `reference_coordinates.robot`

同じ実地図を3台で共有するなら、`reference_coordinates` は fleet 全体で 1 組です。

現在の `tb4_20260612` 向け staging config:

- [config_tb4_20260612_multi.yaml](/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml:1)

現在の `tb4_20260612` 用 nav graph:

- [0.yaml](/home/masu_ubu/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml:1)

## 注意

- 3台で同じ occupancy map / same localization frame を共有するなら、map 原点とスケールが一致していることが前提
- robot ごとに別 map を使う場合は、1本の adapter での単純共有は難しくなる
- charger の進入向きが違うときは dock lane 向きも robot ごとに合わせる
