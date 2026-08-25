#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/tb4_paths.sh
source "${SCRIPT_DIR}/lib/tb4_paths.sh"

"${SCRIPT_DIR}/check_tb4_rmf_assets.sh"

python3 - "${TB4_CONFIG_PATH}" <<'PY'
import sys
from pathlib import Path

import yaml

config_path = Path(sys.argv[1])

with config_path.open("r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream)

reference_coordinates = config.get("reference_coordinates", {})
rmf_coordinates = reference_coordinates.get("rmf") or []
robot_coordinates = reference_coordinates.get("robot") or []

if not rmf_coordinates:
    print(
        f"[error] RMF reference coordinates are empty: {config_path}",
        file=sys.stderr,
    )
    sys.exit(2)

if not robot_coordinates:
    print(
        f"[error] Robot reference coordinates are empty: {config_path}",
        file=sys.stderr,
    )
    sys.exit(2)

if len(rmf_coordinates) != len(robot_coordinates):
    print(
        "[error] Reference coordinate length mismatch: "
        f"rmf={len(rmf_coordinates)}, "
        f"robot={len(robot_coordinates)}",
        file=sys.stderr,
    )
    sys.exit(2)

print(
    "[ok] Reference coordinates: "
    f"{len(rmf_coordinates)} corresponding points"
)
PY

export TB4_CONFIG_PATH
export TB4_NAV_GRAPH_PATH
export TB4_ENV_SCRIPT

exec "${SCRIPT_DIR}/run_direct_adapter.sh" "$@"
