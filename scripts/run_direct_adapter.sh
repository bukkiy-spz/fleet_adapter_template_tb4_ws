#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/tb4_paths.sh
source "${SCRIPT_DIR}/lib/tb4_paths.sh"

CONFIG_PATH="${TB4_CONFIG_PATH}"
NAV_GRAPH_PATH="${TB4_NAV_GRAPH_PATH}"
ENV_SCRIPT="${TB4_ENV_SCRIPT}"

for required_file in \
  "${CONFIG_PATH}" \
  "${NAV_GRAPH_PATH}" \
  "${ENV_SCRIPT}"
do
  if [ ! -f "${required_file}" ]; then
    echo "[error] Required file not found:" >&2
    echo "        ${required_file}" >&2
    exit 1
  fi
done

for setup_file in \
  /opt/ros/humble/setup.bash \
  "${TB4_RMF_OFFICIAL_WS}/install/setup.bash" \
  "${TB4_ADAPTER_WS}/install/setup.bash"
do
  if [ ! -f "${setup_file}" ]; then
    echo "[error] Setup file not found:" >&2
    echo "        ${setup_file}" >&2
    exit 1
  fi
done

set +u

# 先に実機用またはシミュレーション用のDDS環境を設定する。
# sim_local_env.bash:
#   ローカルDDS、Discovery Serverなし
#
# shared_multi_robot_env.bash:
#   実機通信用DDS、Discovery Serverあり
# shellcheck disable=SC1090
source "${ENV_SCRIPT}"

# ENV_SCRIPTがPython仮想環境を設定しても、
# ROS/RMFはUbuntuのシステムPythonへ戻す。
unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH
unset AMENT_PYTHON_EXECUTABLE

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
hash -r

# ENV_SCRIPTが設定したROS_DOMAIN_IDやDiscovery設定は維持する。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

source /opt/ros/humble/setup.bash
source "${TB4_RMF_OFFICIAL_WS}/install/setup.bash"
source "${TB4_ADAPTER_WS}/install/setup.bash"

# NumPy ABI不一致を起動前に検出する。
if ! /usr/bin/python3 - <<'PY'
import numpy
import rmf_adapter

major = int(numpy.__version__.split(".", 1)[0])
if major >= 2:
    raise RuntimeError(
        f"ROS/RMF requires NumPy 1.x in this environment; "
        f"detected {numpy.__version__}"
    )
PY
then
  echo "[error] ROS/RMF Python environment check failed." >&2
  exit 1
fi

echo "========================================"
echo " TurtleBot4 Fleet Adapter"
echo "========================================"
echo "RMF workspace    : ${TB4_RMF_OFFICIAL_WS}"
echo "Adapter workspace: ${TB4_ADAPTER_WS}"
echo "Config           : ${CONFIG_PATH}"
echo "Navigation graph : ${NAV_GRAPH_PATH}"
echo "Environment      : ${ENV_SCRIPT}"
echo "Python           : $(command -v python3)"
echo "ROS domain       : ${ROS_DOMAIN_ID}"
echo "Discovery server : ${ROS_DISCOVERY_SERVER:-<unset>}"
echo
echo "Stop with Ctrl+C."
echo

exec ros2 run tb4_fleet_adapter fleet_adapter \
  -c "${CONFIG_PATH}" \
  -n "${NAV_GRAPH_PATH}" \
  "$@"
