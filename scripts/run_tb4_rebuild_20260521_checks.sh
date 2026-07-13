#!/usr/bin/env bash
set -euo pipefail

fleet="${FLEET_NAME:-tb4_fleet}"
robot="${ROBOT_NAME:-robot2}"
dock_repeat="${DOCK_REPEAT:-5}"

places=(LP1 LP2 LP3 pre_dock robot2_charger)

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

echo "[Check 1/2] LP and charger reachability"
for place in "${places[@]}"; do
  echo
  echo "Dispatching: ${place}"
  ros2 run rmf_demos_tasks dispatch_go_to_place -F "${fleet}" -R "${robot}" -p "${place}"
  echo "Reached ${place} after the task starts/completes, then press Enter."
  read -r
done

echo
echo "[Check 2/2] pre_dock -> charger repeat (${dock_repeat} cycles)"
for i in $(seq 1 "${dock_repeat}"); do
  echo
  echo "Cycle ${i}: pre_dock"
  ros2 run rmf_demos_tasks dispatch_go_to_place -F "${fleet}" -R "${robot}" -p pre_dock
  echo "Reached pre_dock, then press Enter."
  read -r

  echo "Cycle ${i}: robot2_charger"
  ros2 run rmf_demos_tasks dispatch_go_to_place -F "${fleet}" -R "${robot}" -p robot2_charger
  echo "Reached robot2_charger. Check heading consistency, then press Enter."
  read -r
done

echo
echo "Done: verify that all dispatches succeeded and charger approach heading was consistent."
