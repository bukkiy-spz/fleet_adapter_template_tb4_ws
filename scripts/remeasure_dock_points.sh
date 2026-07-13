#!/usr/bin/env bash
set -euo pipefail

ws_root="/home/masu_ubu/fleet_adapter_template_tb4_ws"
updater="${ws_root}/scripts/update_reference_from_amcl.py"
samples="${SAMPLES:-20}"
timeout="${TIMEOUT:-12}"

cfg_main="${ws_root}/src/tb4_fleet_adapter/config.yaml"
cfg_simple="${ws_root}/src/tb4_fleet_adapter/config_simple.yaml"

source /opt/ros/humble/setup.bash
source "${ws_root}/install/setup.bash"

run_update() {
  local cfg="$1"
  local waypoint="$2"
  python3 "${updater}" \
    --config "${cfg}" \
    --waypoint "${waypoint}" \
    --samples "${samples}" \
    --timeout "${timeout}"
}

echo "Step 1/2: pre_dock"
echo "Robot placement:"
echo "- Face the dock head-on"
echo "- Keep about 0.25-0.35 m standoff"
read -r -p "Press Enter when robot is placed at pre_dock..."
run_update "${cfg_main}" "pre_dock"
run_update "${cfg_simple}" "pre_dock"

echo
echo "Step 2/2: robot2_charger"
echo "Robot placement:"
echo "- Place exactly at docked target pose (center/alignment)"
echo "- Keep heading exactly same as dock orientation"
read -r -p "Press Enter when robot is placed at robot2_charger..."
run_update "${cfg_main}" "robot2_charger"
run_update "${cfg_simple}" "robot2_charger"

echo
echo "Done. Rebuild and restart adapter:"
echo "  cd ${ws_root}"
echo "  colcon build --packages-select tb4_fleet_adapter"
echo "  ./scripts/run_direct_adapter.sh"
