#!/usr/bin/env bash
set -euo pipefail

config_path="${TB4_CONFIG_PATH:-/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config_tb4_20260612_multi.yaml}"
nav_graph_path="${TB4_NAV_GRAPH_PATH:-/home/masu_ubu/rmf_main_ws/maps/tb4_rebuild_20260612/nav_graphs/0.yaml}"
env_script="${TB4_ENV_SCRIPT:-/home/masu_ubu/turtlebot4_ws/scripts/shared_multi_robot_env.bash}"

python3 - "${config_path}" <<'PY'
import sys
import yaml

config_path = sys.argv[1]
with open(config_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

ref = data.get("reference_coordinates", {})
rmf = ref.get("rmf") or []
robot = ref.get("robot") or []

if not rmf:
    print(f"[run_direct_adapter_tb4_20260612] rmf coordinates are empty in {config_path}", file=sys.stderr)
    sys.exit(2)

if not robot:
    print(f"[run_direct_adapter_tb4_20260612] robot coordinates are empty in {config_path}", file=sys.stderr)
    print("[run_direct_adapter_tb4_20260612] measure robot-side points first, then retry", file=sys.stderr)
    sys.exit(2)

if len(rmf) != len(robot):
    print(
        f"[run_direct_adapter_tb4_20260612] reference coordinate length mismatch: "
        f"rmf={len(rmf)} robot={len(robot)}",
        file=sys.stderr,
    )
    sys.exit(2)
PY

export TB4_CONFIG_PATH="${config_path}"
export TB4_NAV_GRAPH_PATH="${nav_graph_path}"
export TB4_ENV_SCRIPT="${env_script}"

exec /home/masu_ubu/fleet_adapter_template_tb4_ws/scripts/run_direct_adapter.sh "$@"
