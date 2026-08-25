#!/usr/bin/env bash

TB4_SCRIPT_LIB_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export TB4_ADAPTER_WS="$(cd -- "${TB4_SCRIPT_LIB_DIR}/../.." && pwd)"
export RESEARCH_ROOT="${RESEARCH_ROOT:-${HOME}/research}"

export TB4_RMF_OFFICIAL_WS="${TB4_RMF_OFFICIAL_WS:-${RESEARCH_ROOT}/rmf_ws/rmf_humble_official}"
export TB4_TURTLEBOT_WS="${TB4_TURTLEBOT_WS:-${RESEARCH_ROOT}/turtlebot4_ws}"

export TB4_ADAPTER_PACKAGE_DIR="${TB4_ADAPTER_PACKAGE_DIR:-${TB4_ADAPTER_WS}/src/tb4_fleet_adapter}"
export TB4_DEMO_PACKAGE_DIR="${TB4_DEMO_PACKAGE_DIR:-${TB4_ADAPTER_WS}/src/tb4_rmf_demo}"

export TB4_CONFIG_PATH="${TB4_CONFIG_PATH:-${TB4_ADAPTER_PACKAGE_DIR}/config_tb4_20260612_multi.yaml}"
export TB4_NAV_GRAPH_PATH="${TB4_NAV_GRAPH_PATH:-${TB4_DEMO_PACKAGE_DIR}/maps/rmf/tb4_20260612/nav_graphs/0.yaml}"
export TB4_BUILDING_MAP_PATH="${TB4_BUILDING_MAP_PATH:-${TB4_DEMO_PACKAGE_DIR}/maps/rmf/tb4_20260612/tb4_20260612.building.yaml}"
export TB4_WORLD_PATH="${TB4_WORLD_PATH:-${TB4_DEMO_PACKAGE_DIR}/maps/rmf/tb4_20260612/world/tb4_20260612.world}"
export TB4_ENV_SCRIPT="${TB4_ENV_SCRIPT:-${TB4_TURTLEBOT_WS}/scripts/shared_multi_robot_env.bash}"
