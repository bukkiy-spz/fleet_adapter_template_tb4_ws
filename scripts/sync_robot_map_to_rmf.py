#!/usr/bin/env python3

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import shutil

import yaml


def load_map_yaml(path: Path):
    with path.open() as f:
        return yaml.safe_load(f)


def write_map_yaml(path: Path, data) -> None:
    with path.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def copy_map_variant(source_yaml: Path, dest_dir: Path, basename: str) -> tuple[Path, Path]:
    data = load_map_yaml(source_yaml)
    source_image = source_yaml.parent / data["image"]
    image_ext = source_image.suffix or ".pgm"

    dest_yaml = dest_dir / f"{basename}.yaml"
    dest_image = dest_dir / f"{basename}{image_ext}"

    shutil.copy2(source_image, dest_image)
    data["image"] = dest_image.name
    write_map_yaml(dest_yaml, data)
    return dest_yaml, dest_image


def main():
    today = datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(
        description="Copy the latest robot occupancy map into rmf_main_ws/maps."
    )
    parser.add_argument(
        "--source-yaml",
        default="/home/masu_ubu/maps/robot2_map.yaml",
        help="Path to the source occupancy map yaml",
    )
    parser.add_argument(
        "--dest-dir",
        default="/home/masu_ubu/rmf_main_ws/maps/tb4",
        help="Destination directory inside rmf_main_ws",
    )
    parser.add_argument(
        "--basename",
        default=f"robot2_map_{today}",
        help="Basename to use for the copied yaml/image pair",
    )
    parser.add_argument(
        "--also-latest",
        action="store_true",
        help="Also refresh robot2_map_latest.yaml/image in the same directory",
    )
    args = parser.parse_args()

    source_yaml = Path(args.source_yaml).expanduser().resolve()
    dest_dir = Path(args.dest_dir).expanduser().resolve()
    dest_dir.mkdir(parents=True, exist_ok=True)

    dated_yaml, dated_image = copy_map_variant(source_yaml, dest_dir, args.basename)
    print(f"dated_yaml: {dated_yaml}")
    print(f"dated_image: {dated_image}")

    if args.also_latest:
        latest_yaml, latest_image = copy_map_variant(source_yaml, dest_dir, "robot2_map_latest")
        print(f"latest_yaml: {latest_yaml}")
        print(f"latest_image: {latest_image}")


if __name__ == "__main__":
    main()
