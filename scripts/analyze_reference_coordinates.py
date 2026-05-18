#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import sys

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "tb4_fleet_adapter"
sys.path.insert(0, str(PACKAGE_ROOT))

from tb4_fleet_adapter import nudged_compat as nudged  # noqa: E402


def load_reference_config(path: Path):
    with path.open() as f:
        data = yaml.safe_load(f)
    return data["reference_coordinates"]["rmf"], data["reference_coordinates"]["robot"]


def load_waypoint_names(nav_graph_path: Path, level: str):
    with nav_graph_path.open() as f:
        data = yaml.safe_load(f)
    vertices = data["levels"][level]["vertices"]
    return [vertex[2].get("name", f"v{i}") for i, vertex in enumerate(vertices)]


def format_vec(values):
    return "[" + ", ".join(f"{v:.6f}" for v in values) + "]"


def main():
    parser = argparse.ArgumentParser(
        description="Analyze RMF <-> robot reference coordinate pairs."
    )
    parser.add_argument(
        "--config",
        default=str(
            REPO_ROOT / "src" / "tb4_fleet_adapter" / "config.yaml"
        ),
        help="Path to tb4_fleet_adapter config YAML",
    )
    parser.add_argument(
        "--nav-graph",
        default="/home/masu_ubu/rmf_main_ws/maps/tb4/nav_graphs/1.yaml",
        help="Path to RMF nav graph",
    )
    parser.add_argument(
        "--level",
        default="L1",
        help="Nav graph level name",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    nav_graph_path = Path(args.nav_graph)

    rmf_points, robot_points = load_reference_config(config_path)
    labels = load_waypoint_names(nav_graph_path, args.level)[: len(rmf_points)]

    rmf = np.asarray(rmf_points, dtype=float)
    robot = np.asarray(robot_points, dtype=float)

    transform = nudged.estimate(rmf, robot)
    mse = nudged.estimate_error(transform, rmf, robot)
    rmse = math.sqrt(mse)

    print("Reference coordinate analysis")
    print(f"config: {config_path}")
    print(f"nav_graph: {nav_graph_path} [{args.level}]")
    print()
    print("RMF -> Robot transform")
    print(f"  rotation_rad: {transform.get_rotation():.6f}")
    print(f"  rotation_deg: {math.degrees(transform.get_rotation()):.3f}")
    print(f"  scale: {transform.get_scale():.6f}")
    print(f"  translation: {format_vec(transform.get_translation())}")
    print(f"  mse: {mse:.6f}")
    print(f"  rmse: {rmse:.6f}")
    print()
    print("Per-point residuals")

    residuals = []
    for label, rmf_point, robot_point in zip(labels, rmf, robot):
        predicted = np.asarray(transform.transform(rmf_point), dtype=float)
        error = float(np.linalg.norm(predicted - robot_point))
        residuals.append(error)
        print(
            f"  {label:>14}: predicted={format_vec(predicted)} "
            f"actual={format_vec(robot_point)} error_m={error:.4f}"
        )

    print()
    print("Leave-one-out check")
    for omit_idx, label in enumerate(labels):
        kept_indices = [i for i in range(len(labels)) if i != omit_idx]
        loo_transform = nudged.estimate(rmf[kept_indices], robot[kept_indices])
        predicted = np.asarray(loo_transform.transform(rmf[omit_idx]), dtype=float)
        omitted_error = float(np.linalg.norm(predicted - robot[omit_idx]))
        kept_errors = [
            float(
                np.linalg.norm(
                    np.asarray(loo_transform.transform(rmf[i]), dtype=float) - robot[i]
                )
            )
            for i in kept_indices
        ]
        print(
            f"  omit {label:>14}: scale={loo_transform.get_scale():.6f} "
            f"kept_mean_error={np.mean(kept_errors):.4f} "
            f"omitted_error={omitted_error:.4f}"
        )

    print()
    max_error = max(residuals) if residuals else 0.0
    if abs(transform.get_scale() - 1.0) > 0.2:
        print("WARN: scale is far from 1.0. One or more reference pairs may be mismatched.")
    if max_error > 0.3:
        print("WARN: at least one point has residual > 0.3 m. Re-measure suspicious points.")


if __name__ == "__main__":
    main()
