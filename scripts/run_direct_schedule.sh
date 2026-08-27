#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/tb4_paths.sh
source "${SCRIPT_DIR}/lib/tb4_paths.sh"

set +u

unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH
unset ROS_DISCOVERY_SERVER
unset ROS_STATIC_PEERS
unset FASTRTPS_DEFAULT_PROFILES_FILE

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

source /opt/ros/humble/setup.bash

RMF_SETUP="${TB4_RMF_OFFICIAL_WS}/install/setup.bash"

if [ ! -f "${RMF_SETUP}" ]; then
  echo "[error] RMF setup file not found:" >&2
  echo "        ${RMF_SETUP}" >&2
  exit 1
fi

source "${RMF_SETUP}"

echo "========================================"
echo " RMF Traffic Schedule"
echo "========================================"
echo "RMF workspace: ${TB4_RMF_OFFICIAL_WS}"
echo "Executable   : rmf_traffic_schedule"
echo
echo "Stop with Ctrl+C."
echo

exec ros2 run rmf_traffic_ros2 rmf_traffic_schedule "$@"
