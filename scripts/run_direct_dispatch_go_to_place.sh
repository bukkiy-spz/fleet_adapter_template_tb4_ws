#!/usr/bin/env bash
set -euo pipefail

place="${1:-LP1}"
robot="${2:-robot2}"
fleet="${3:-tb4_fleet}"
dispatch_gate_file="${TB4_DISPATCH_GATE_FILE:-/tmp/tb4_manual_dispatch_enabled_${robot}}"
env_script="${TB4_ENV_SCRIPT:-/home/masu_ubu/turtlebot4_ws/scripts/robot2_env.bash}"

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
# shellcheck disable=SC1090
source "${env_script}"

if [[ ${restore_nounset} -eq 1 ]]; then
  set -u
fi

date +%s > "${dispatch_gate_file}"

ros2 run rmf_demos_tasks dispatch_go_to_place -F "${fleet}" -R "${robot}" -p "${place}"
