# TB4 Hybrid Fleet Adapter Architecture

このメモは、今後の研究用本線として採用する
`custom fleet adapter + zenoh/free_fleet 的な疎結合通信`
の構成をまとめたものです。

## 目的

- RMF / schedule node と robot 直接発見を分離する
- `RobotClientAPI.py` はなるべくそのまま残す
- 将来の複数実機・縮退制御・通信切替研究に耐える構成にする

## 採用方針

上位:

- `rmf_traffic_schedule`
- `tb4_fleet_adapter`

通信層:

- `zenohd`
- host 側 `zenoh-bridge-ros2dds`
- robot 側 `zenoh-bridge-ros2dds`

下位:

- robot 側 `localization`
- robot 側 `Nav2`

## 直結構成で詰まった理由

直結構成では Host 側 adapter が

- RMF schedule node
- robot 実機 ROS graph

の両方に同時接続する必要がありました。

しかし実際には、

- `robot2_env.bash` の Discovery Server 環境を入れると
  RMF service 応答が timeout しやすい
- RMF 側を優先すると robot 側 `/robot2/amcl_pose` や
  `/robot2/navigate_to_pose` に安定接続しにくい

という問題が起きました。

## ハイブリッド構成の考え方

robot 側 ROS エンティティを一度 zenoh に逃がし、
Host 側で再び ROS graph に再生します。

その結果、Host 側 adapter から見ると

- `/robot2/amcl_pose`
- `/robot2/battery_state`
- `/robot2/navigate_to_pose`

が **ローカル ROS graph 上に見える** 形になります。

これなら `RobotClientAPI.py` は direct ROS 実装のままで使えます。

## 必要な公開対象

robot 側から zenoh に流すもの:

- `/robot2/amcl_pose`
- `/robot2/battery_state`
- `/robot2/tf`
- `/robot2/navigate_to_pose` action server

現時点で adapter 本体が必須で使うのは:

- `/robot2/amcl_pose`
- `/robot2/battery_state`
- `/robot2/navigate_to_pose`

`/robot2/tf` は切り分けや将来拡張用です。

## 使う設定ファイル

robot 側 bridge:

- `/home/masu_ubu/jazzy_ff_ws/config/zenoh/robot2_zenoh_bridge_ros2dds_client_config.json5`

host 側 bridge:

- `/home/masu_ubu/fleet_adapter_template_tb4_ws/config/zenoh/robot2_host_zenoh_bridge_ros2dds_client_config.json5`
- host 側 x86_64 binary:
  - `/home/masu_ubu/Downloads/zenoh_bridge_extract_x86_64/zenoh-bridge-ros2dds`

## 起動原則

1. Host 側で `zenohd`
2. robot 側で `localization`
3. robot 側で `Nav2`
4. robot 側で `zenoh-bridge-ros2dds`
5. Host 側で `zenoh-bridge-ros2dds`
6. Host 側で `/robot2/amcl_pose` と `/robot2/navigate_to_pose` が見えることを確認
7. Host 側で `rmf_traffic_schedule`
8. Host 側で `tb4_fleet_adapter`

## 研究向けの利点

- robot ごとに通信断や劣化を局所化しやすい
- adapter 側に縮退制御ロジックを実装しやすい
- 複数 robot を増やすときも bridge 設定追加で横展開しやすい
- 将来 `RobotClientAPI.py` を zenoh 直叩き backend に差し替える余地も残る

## 次の改善候補

1. `reference_coordinates` の実測反映
2. host 側で bridge 経由の `/robot2/amcl_pose` / action 実確認
3. bridge 経由で adapter 起動確認
4. `RobotClientAPI` に backend 設定項目を追加
5. battery / docking / degraded mode を adapter 側へ実装
