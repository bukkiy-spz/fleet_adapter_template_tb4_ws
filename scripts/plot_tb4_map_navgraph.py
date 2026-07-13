#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "tb4_fleet_adapter"
sys.path.insert(0, str(PACKAGE_ROOT))

from tb4_fleet_adapter import nudged_compat as nudged  # noqa: E402


def load_map_yaml(path: Path):
    with path.open() as f:
        data = yaml.safe_load(f)
    image_path = path.parent / data["image"]
    return data, image_path


def load_map_image(image_path: Path):
    image = Image.open(image_path)
    return np.array(image)


def map_extent(map_yaml, image_array):
    resolution = float(map_yaml["resolution"])
    origin_x, origin_y, _ = map_yaml["origin"]
    height, width = image_array.shape[:2]
    min_x = origin_x
    max_x = origin_x + width * resolution
    min_y = origin_y
    max_y = origin_y + height * resolution
    return [min_x, max_x, min_y, max_y]


def load_nav_graph(path: Path, level_name: str):
    with path.open() as f:
        data = yaml.safe_load(f)
    level = data["levels"][level_name]
    return level["vertices"], level["lanes"]


def load_reference_config(path: Path):
    with path.open() as f:
        data = yaml.safe_load(f)
    rmf = data["reference_coordinates"]["rmf"]
    robot = data["reference_coordinates"]["robot"]
    return rmf, robot


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_amcl_pose(topic: str, timeout: float):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    class PoseReader(Node):
        def __init__(self):
            super().__init__("tb4_map_navgraph_pose_reader")
            self.message = None
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                self._callback,
                qos,
            )

        def _callback(self, msg):
            self.message = msg

    rclpy.init()
    node = PoseReader()
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.message is not None:
                pose = node.message.pose.pose
                return (
                    pose.position.x,
                    pose.position.y,
                    yaw_from_quaternion(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize TB4 occupancy map and nav graph, optionally with current amcl_pose."
    )
    parser.add_argument(
        "--map-yaml",
        default="/home/masu_ubu/rmf_main_ws/maps/tb4/robot2_map_latest.yaml",
        help="Path to occupancy map yaml",
    )
    parser.add_argument(
        "--nav-graph",
        default="/home/masu_ubu/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml",
        help="Path to nav graph yaml",
    )
    parser.add_argument(
        "--config",
        default="/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml",
        help="Path to adapter config yaml (used for RMF->robot transform)",
    )
    parser.add_argument(
        "--level",
        default="L1",
        help="Nav graph level name",
    )
    parser.add_argument(
        "--use-robot-frame",
        action="store_true",
        help="Transform nav graph from RMF coordinates into robot map coordinates",
    )
    parser.add_argument(
        "--topic",
        default="",
        help="Optional amcl pose topic to overlay, e.g. /robot2/amcl_pose",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Seconds to wait for amcl_pose",
    )
    parser.add_argument(
        "--save",
        default="",
        help="Optional output image path. If omitted, opens an interactive window.",
    )
    args = parser.parse_args()

    map_yaml_path = Path(args.map_yaml)
    nav_graph_path = Path(args.nav_graph)
    config_path = Path(args.config)

    map_yaml, image_path = load_map_yaml(map_yaml_path)
    image = load_map_image(image_path)
    extent = map_extent(map_yaml, image)
    vertices, lanes = load_nav_graph(nav_graph_path, args.level)

    transform_rmf_to_robot = None
    if args.use_robot_frame:
        rmf_points, robot_points = load_reference_config(config_path)
        transform_rmf_to_robot = nudged.estimate(rmf_points, robot_points)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(image, cmap="gray", origin="lower", extent=extent)

    for lane in lanes:
        start_idx, end_idx, _ = lane
        x1, y1, _meta1 = vertices[start_idx]
        x2, y2, _meta2 = vertices[end_idx]
        if transform_rmf_to_robot is not None:
            x1, y1 = transform_rmf_to_robot.transform([x1, y1])
            x2, y2 = transform_rmf_to_robot.transform([x2, y2])
        ax.plot([x1, x2], [y1, y2], color="tab:blue", linewidth=1.5, alpha=0.8)

    xs = []
    ys = []
    for idx, vertex in enumerate(vertices):
        x, y, meta = vertex
        if transform_rmf_to_robot is not None:
            x, y = transform_rmf_to_robot.transform([x, y])
        name = meta.get("name", f"v{idx}")
        xs.append(x)
        ys.append(y)
        ax.scatter(x, y, color="tab:red", s=35, zorder=3)
        ax.text(x + 0.04, y + 0.04, name, color="tab:red", fontsize=9)

    if args.topic:
        pose = read_amcl_pose(args.topic, args.timeout)
        if pose is not None:
            x, y, yaw = pose
            ax.scatter(x, y, color="tab:green", s=55, zorder=4, label="amcl_pose")
            ax.arrow(
                x,
                y,
                0.18 * math.cos(yaw),
                0.18 * math.sin(yaw),
                color="tab:green",
                width=0.01,
                length_includes_head=True,
                zorder=4,
            )
            ax.text(x + 0.05, y - 0.08, "robot2", color="tab:green", fontsize=10)
            print(f"amcl_pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        else:
            print(f"No pose received from {args.topic} within {args.timeout}s")

    ax.set_title("TB4 map + nav graph")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)

    if args.save:
        output_path = Path(args.save)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=160, bbox_inches="tight")
        print(f"Saved: {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
