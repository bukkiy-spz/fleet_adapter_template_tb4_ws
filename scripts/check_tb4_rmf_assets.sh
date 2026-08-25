#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/tb4_paths.sh
source "${SCRIPT_DIR}/lib/tb4_paths.sh"

status=0

check_file() {
  local label="$1"
  local path="$2"

  if [ -f "${path}" ]; then
    printf '[ok]      %-22s %s\n' "${label}" "${path}"
  else
    printf '[missing] %-22s %s\n' "${label}" "${path}" >&2
    status=1
  fi
}

check_dir() {
  local label="$1"
  local path="$2"

  if [ -d "${path}" ]; then
    printf '[ok]      %-22s %s\n' "${label}" "${path}"
  else
    printf '[missing] %-22s %s\n' "${label}" "${path}" >&2
    status=1
  fi
}

check_dir  "Official RMF workspace" "${TB4_RMF_OFFICIAL_WS}"
check_dir  "TurtleBot4 workspace"   "${TB4_TURTLEBOT_WS}"
check_dir  "Adapter workspace"      "${TB4_ADAPTER_WS}"
check_dir  "Demo package"           "${TB4_DEMO_PACKAGE_DIR}"

check_file "Fleet config"           "${TB4_CONFIG_PATH}"
check_file "Navigation graph"       "${TB4_NAV_GRAPH_PATH}"
check_file "Building map"           "${TB4_BUILDING_MAP_PATH}"
check_file "Generated world"        "${TB4_WORLD_PATH}"
check_file "Robot environment"      "${TB4_ENV_SCRIPT}"

exit "${status}"
