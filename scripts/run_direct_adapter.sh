#!/usr/bin/env bash
set -euo pipefail

config_path="/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml"
nav_graph_path="/home/masu_ubu/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml"
dispatch_gate_file="/tmp/tb4_manual_dispatch_enabled_robot2"
retry_delay_sec="${TB4_ADAPTER_RETRY_DELAY_SEC:-3}"
max_restarts="${TB4_ADAPTER_MAX_RESTARTS:-0}" # 0 means infinite
schedule_wait_sec="${TB4_SCHEDULE_WAIT_SEC:-60}"
schedule_service="/rmf_traffic/register_query"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}"

restore_nounset=0
if [[ $- == *u* ]]; then
  restore_nounset=1
  set +u
fi

source /opt/ros/humble/setup.bash
source /home/masu_ubu/rmf_main_ws/install/setup.bash
source /home/masu_ubu/fleet_adapter_template_tb4_ws/install/setup.bash
source /home/masu_ubu/turtlebot4_ws/scripts/robot2_env.bash

if [[ ${restore_nounset} -eq 1 ]]; then
  set -u
fi

rm -f "${dispatch_gate_file}"

deadline=$((SECONDS + schedule_wait_sec))
while true; do
  if ros2 service list 2>/dev/null | grep -q "^${schedule_service}$"; then
    break
  fi
  if [[ ${SECONDS} -ge ${deadline} ]]; then
    echo "[run_direct_adapter] ${schedule_service} not found within ${schedule_wait_sec}s, starting anyway" >&2
    break
  fi
  sleep 1
done

attempt=0
while true; do
  attempt=$((attempt + 1))
  ros2 run tb4_fleet_adapter fleet_adapter -c "${config_path}" -n "${nav_graph_path}" "$@"
  rc=$?
  if [[ ${rc} -eq 0 ]]; then
    exit 0
  fi

  echo "[run_direct_adapter] fleet_adapter exited with code ${rc} (attempt ${attempt})" >&2
  if [[ ${max_restarts} -gt 0 && ${attempt} -ge ${max_restarts} ]]; then
    exit "${rc}"
  fi
  sleep "${retry_delay_sec}"
done
