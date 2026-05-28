#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
import yaml


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseReader(Node):
    def __init__(self, topic: str):
        super().__init__("update_reference_from_amcl")
        self.message = None
        self.message_count = 0
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
        self.message_count += 1


def read_pose(topic: str, timeout_sec: float, samples: int):
    rclpy.init()
    node = PoseReader(topic)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        last_count = 0
        xs = []
        ys = []
        yaws = []
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.message is None:
                continue
            if node.message_count == last_count:
                continue
            last_count = node.message_count
            pose = node.message.pose.pose
            yaw = yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            xs.append(float(pose.position.x))
            ys.append(float(pose.position.y))
            yaws.append(float(yaw))
            if len(xs) >= samples:
                mean_x = sum(xs) / len(xs)
                mean_y = sum(ys) / len(ys)
                mean_yaw = math.atan2(
                    sum(math.sin(y) for y in yaws),
                    sum(math.cos(y) for y in yaws),
                )
                return mean_x, mean_y, mean_yaw, len(xs)

        if xs:
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            mean_yaw = math.atan2(
                sum(math.sin(y) for y in yaws),
                sum(math.cos(y) for y in yaws),
            )
            return mean_x, mean_y, mean_yaw, len(xs)

        raise RuntimeError(f"No pose received on {topic} within {timeout_sec}s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def waypoint_vertices(nav_graph: dict, level_name: str):
    levels = nav_graph["levels"]
    if level_name not in levels:
        raise RuntimeError(f"Level [{level_name}] is not found in nav graph")
    vertices = levels[level_name]["vertices"]
    result = []
    for i, vertex in enumerate(vertices):
        x, y, meta = vertex
        name = meta.get("name", f"v{i}")
        result.append((i, name, float(x), float(y)))
    return result


def find_reference_index(reference_rmf, nav_vertices, waypoint_name: str, max_dist: float):
    target_candidates = [v for v in nav_vertices if v[1] == waypoint_name]
    if not target_candidates:
        raise RuntimeError(f"Waypoint [{waypoint_name}] is not present in nav graph")
    target = target_candidates[0]
    tx, ty = target[2], target[3]

    best_idx = None
    best_dist = None
    for i, point in enumerate(reference_rmf):
        px, py = float(point[0]), float(point[1])
        d = math.hypot(px - tx, py - ty)
        if best_dist is None or d < best_dist:
            best_dist = d
            best_idx = i

    if best_idx is None:
        raise RuntimeError("Unable to match reference point index")
    if best_dist is None or best_dist > max_dist:
        raise RuntimeError(
            f"Matched index distance is too large ({best_dist:.3f} m > {max_dist:.3f} m). "
            "Check config/nav_graph consistency."
        )
    return best_idx, target


def update_config(config_path: Path, waypoint_name: str, pose_x: float, pose_y: float,
                  nav_graph_path: Path, level_name: str, max_match_dist: float, dry_run: bool):
    with config_path.open() as f:
        data = yaml.safe_load(f)

    with nav_graph_path.open() as f:
        nav_graph = yaml.safe_load(f)

    reference = data["reference_coordinates"]
    rmf_points = reference["rmf"]
    robot_points = reference["robot"]

    vertices = waypoint_vertices(nav_graph, level_name)
    point_index, waypoint = find_reference_index(
        rmf_points, vertices, waypoint_name, max_match_dist
    )

    old_x = float(robot_points[point_index][0])
    old_y = float(robot_points[point_index][1])
    robot_points[point_index] = [round(float(pose_x), 6), round(float(pose_y), 6)]

    print(f"config: {config_path}")
    print(
        f"waypoint: {waypoint_name} (vertex_index={waypoint[0]}, "
        f"rmf=[{waypoint[2]:.6f}, {waypoint[3]:.6f}], ref_index={point_index})"
    )
    print(f"robot old: [{old_x:.6f}, {old_y:.6f}]")
    print(f"robot new: [{pose_x:.6f}, {pose_y:.6f}]")

    if dry_run:
        print("dry-run: config file is not written.")
        return

    with config_path.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print("updated: written to config file")


def main():
    parser = argparse.ArgumentParser(
        description="Update one reference robot coordinate from current amcl pose."
    )
    parser.add_argument(
        "--config",
        default="/home/masu_ubu/fleet_adapter_template_tb4_ws/src/tb4_fleet_adapter/config.yaml",
        help="Path to adapter config yaml",
    )
    parser.add_argument(
        "--nav-graph",
        default="/home/masu_ubu/rmf_main_ws/maps/tb4_rebuild_20260521/nav_graphs/0.yaml",
        help="Path to nav graph yaml",
    )
    parser.add_argument(
        "--level",
        default="L1",
        help="Nav graph level name",
    )
    parser.add_argument(
        "--waypoint",
        required=True,
        help="Waypoint name to update, e.g. pre_dock",
    )
    parser.add_argument(
        "--topic",
        default="/robot2/amcl_pose",
        help="amcl pose topic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Seconds to wait for pose",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=12,
        help="Number of amcl samples to average",
    )
    parser.add_argument(
        "--max-match-dist",
        type=float,
        default=0.35,
        help="Max RMF distance to map a config reference point to the waypoint",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show the update without writing the file",
    )
    args = parser.parse_args()

    pose_x, pose_y, yaw, n = read_pose(args.topic, args.timeout, max(1, int(args.samples)))
    print(f"amcl pose avg: x={pose_x:.6f}, y={pose_y:.6f}, yaw_rad={yaw:.6f}, samples={n}")

    update_config(
        config_path=Path(args.config),
        waypoint_name=args.waypoint,
        pose_x=pose_x,
        pose_y=pose_y,
        nav_graph_path=Path(args.nav_graph),
        level_name=args.level,
        max_match_dist=float(args.max_match_dist),
        dry_run=bool(args.dry_run),
    )


if __name__ == "__main__":
    main()
