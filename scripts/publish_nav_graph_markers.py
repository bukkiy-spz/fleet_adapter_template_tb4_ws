#!/usr/bin/env python3

import argparse
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "tb4_fleet_adapter"
sys.path.insert(0, str(PACKAGE_ROOT))

from tb4_fleet_adapter import nudged_compat as nudged  # noqa: E402


def load_graph(path: Path, level: str):
    with path.open() as f:
        data = yaml.safe_load(f)
    level_data = data["levels"][level]
    return level_data["vertices"], level_data["lanes"]


def load_reference_config(path: Path):
    with path.open() as f:
        data = yaml.safe_load(f)
    return data["reference_coordinates"]["rmf"], data["reference_coordinates"]["robot"]


class NavGraphMarkerPublisher(Node):
    def __init__(
        self,
        nav_graph: Path,
        config: Path,
        level: str,
        frame_id: str,
        topic: str,
        period: float,
        use_robot_frame: bool = False,
        hot_reload: bool = True,
    ):
        super().__init__("nav_graph_marker_publisher")
        self._nav_graph_path = nav_graph
        self._config_path = config
        self._level = level
        self._frame_id = frame_id
        self._use_robot_frame = bool(use_robot_frame)
        self._hot_reload = bool(hot_reload)
        self._vertices = []
        self._lanes = []
        self._transform_rmf_to_robot = None
        self._last_nav_graph_mtime_ns = None
        self._last_config_mtime_ns = None

        self._reload_sources(force=True)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, topic, qos)
        self._timer = self.create_timer(period, self._publish)
        self._publish()

    def _mtime_ns(self, path: Path):
        try:
            return path.stat().st_mtime_ns
        except FileNotFoundError:
            return None

    def _reload_sources(self, force: bool = False):
        nav_graph_mtime = self._mtime_ns(self._nav_graph_path)
        config_mtime = self._mtime_ns(self._config_path) if self._use_robot_frame else None
        nav_changed = (nav_graph_mtime != self._last_nav_graph_mtime_ns)
        config_changed = (config_mtime != self._last_config_mtime_ns)
        need_reload = force or nav_changed or (self._use_robot_frame and config_changed)
        if not need_reload:
            return

        vertices, lanes = load_graph(self._nav_graph_path, self._level)
        transform = None
        if self._use_robot_frame:
            rmf_points, robot_points = load_reference_config(self._config_path)
            transform = nudged.estimate(rmf_points, robot_points)

        self._vertices = vertices
        self._lanes = lanes
        self._transform_rmf_to_robot = transform
        self._last_nav_graph_mtime_ns = nav_graph_mtime
        self._last_config_mtime_ns = config_mtime

        self.get_logger().info(
            f"Reloaded markers source: vertices={len(self._vertices)}, "
            f"lanes={len(self._lanes)}, robot_frame={self._use_robot_frame}"
        )

    def _mk_color(self, r: float, g: float, b: float, a: float) -> ColorRGBA:
        c = ColorRGBA()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        return c

    def _mk_point(self, x: float, y: float, z: float = 0.05) -> Point:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def _publish(self):
        if self._hot_reload:
            try:
                self._reload_sources(force=False)
            except Exception as e:
                # 一時的なYAML編集中に壊れた状態でも、最後に成功したデータで
                # publishを継続し、RViz表示を落とさない。
                self.get_logger().warn(
                    f"Failed to reload nav graph/config; keeping previous markers: {e}"
                )

        markers = MarkerArray()

        lane_marker = Marker()
        lane_marker.header.frame_id = self._frame_id
        lane_marker.header.stamp = self.get_clock().now().to_msg()
        lane_marker.ns = "lanes"
        lane_marker.id = 0
        lane_marker.type = Marker.LINE_LIST
        lane_marker.action = Marker.ADD
        lane_marker.pose.orientation.w = 1.0
        lane_marker.scale.x = 0.03
        lane_marker.color = self._mk_color(0.2, 0.5, 1.0, 0.9)

        for lane in self._lanes:
            s_idx, e_idx, _ = lane
            sx, sy, _ = self._vertices[s_idx]
            ex, ey, _ = self._vertices[e_idx]
            if self._transform_rmf_to_robot is not None:
                sx, sy = self._transform_rmf_to_robot.transform([sx, sy])
                ex, ey = self._transform_rmf_to_robot.transform([ex, ey])
            lane_marker.points.append(self._mk_point(sx, sy))
            lane_marker.points.append(self._mk_point(ex, ey))

        markers.markers.append(lane_marker)

        normal_marker = Marker()
        normal_marker.header.frame_id = self._frame_id
        normal_marker.header.stamp = lane_marker.header.stamp
        normal_marker.ns = "vertices"
        normal_marker.id = 1
        normal_marker.type = Marker.SPHERE_LIST
        normal_marker.action = Marker.ADD
        normal_marker.pose.orientation.w = 1.0
        normal_marker.scale.x = 0.10
        normal_marker.scale.y = 0.10
        normal_marker.scale.z = 0.10
        normal_marker.color = self._mk_color(1.0, 0.2, 0.2, 0.95)

        charger_marker = Marker()
        charger_marker.header.frame_id = self._frame_id
        charger_marker.header.stamp = lane_marker.header.stamp
        charger_marker.ns = "charger_vertices"
        charger_marker.id = 2
        charger_marker.type = Marker.SPHERE_LIST
        charger_marker.action = Marker.ADD
        charger_marker.pose.orientation.w = 1.0
        charger_marker.scale.x = 0.14
        charger_marker.scale.y = 0.14
        charger_marker.scale.z = 0.14
        charger_marker.color = self._mk_color(0.0, 0.9, 0.3, 0.95)

        markers.markers.append(normal_marker)
        markers.markers.append(charger_marker)

        text_base_id = 1000
        for i, vertex in enumerate(self._vertices):
            x, y, meta = vertex
            if self._transform_rmf_to_robot is not None:
                x, y = self._transform_rmf_to_robot.transform([x, y])
            name = meta.get("name", f"v{i}") or f"v{i}"
            is_charger = bool(meta.get("is_charger", False))

            p = self._mk_point(x, y)
            if is_charger:
                charger_marker.points.append(p)
            else:
                normal_marker.points.append(p)

            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = lane_marker.header.stamp
            label.ns = "labels"
            label.id = text_base_id + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.text = name
            label.color = self._mk_color(0.0, 0.0, 0.0, 0.95)
            markers.markers.append(label)

        self._pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(
        description="Publish RViz markers for RMF nav graph waypoints and lanes."
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
    parser.add_argument("--level", default="L1", help="Level name")
    parser.add_argument("--frame-id", default="map", help="RViz frame id")
    parser.add_argument(
        "--topic", default="/tb4/nav_graph_markers", help="MarkerArray topic"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Publish rate in Hz (default: 1.0)",
    )
    parser.add_argument(
        "--use-robot-frame",
        action="store_true",
        help="Transform nav graph from RMF coordinates into robot map coordinates",
    )
    parser.add_argument(
        "--no-hot-reload",
        action="store_true",
        help="Disable auto reload of nav_graph/config when files are edited",
    )
    args = parser.parse_args()

    nav_graph = Path(args.nav_graph).expanduser().resolve()
    config = Path(args.config).expanduser().resolve()
    if not nav_graph.exists():
        raise SystemExit(f"nav graph not found: {nav_graph}")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0")

    if args.use_robot_frame:
        if not config.exists():
            raise SystemExit(f"config not found: {config}")

    rclpy.init()
    node = NavGraphMarkerPublisher(
        nav_graph=nav_graph,
        config=config,
        level=args.level,
        frame_id=args.frame_id,
        topic=args.topic,
        period=1.0 / args.rate,
        use_robot_frame=args.use_robot_frame,
        hot_reload=(not args.no_hot_reload),
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
