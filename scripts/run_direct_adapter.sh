#!/usr/bin/env bash
set -euo pipefail

config_path="/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml"
nav_graph_path="/home/masu_ubu/rmf_main_ws/maps/tb4/nav_graphs/1.yaml"

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

ros2 run tb4_fleet_adapter fleet_adapter -c "${config_path}" -n "${nav_graph_path}" "$@"
