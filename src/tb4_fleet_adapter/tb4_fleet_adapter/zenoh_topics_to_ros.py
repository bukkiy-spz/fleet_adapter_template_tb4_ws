"""Republish selected Zenoh keys as ROS 2 topics for the fleet adapter."""

import argparse
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import BatteryState
import zenoh


class ZenohTopicsToRos(Node):
    def __init__(self, zenoh_config: str) -> None:
        super().__init__("zenoh_topics_to_ros")
        self.session = zenoh.open(zenoh.Config.from_file(zenoh_config))
        self._lock = threading.Lock()

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        battery_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot2/amcl_pose", amcl_qos
        )
        self.battery_pub = self.create_publisher(
            BatteryState, "/robot2/battery_state", battery_qos
        )

        self.amcl_sub = self.session.declare_subscriber(
            "robot2/amcl_pose", self._amcl_cb
        )
        self.battery_sub = self.session.declare_subscriber(
            "robot2/battery_state", self._battery_cb
        )

        self._amcl_count = 0
        self._battery_count = 0
        self.get_logger().info("Republishing Zenoh robot2/* samples to ROS topics")

    def _amcl_cb(self, sample) -> None:
        msg = deserialize_message(sample.payload.to_bytes(), PoseWithCovarianceStamped)
        with self._lock:
            self.amcl_pub.publish(msg)
            self._amcl_count += 1
            count = self._amcl_count
        if count <= 5 or count % 50 == 0:
            pose = msg.pose.pose
            self.get_logger().info(
                f"Republished AMCL sample #{count}: "
                f"x={pose.position.x:.3f}, y={pose.position.y:.3f}"
            )

    def _battery_cb(self, sample) -> None:
        msg = deserialize_message(sample.payload.to_bytes(), BatteryState)
        with self._lock:
            self.battery_pub.publish(msg)
            self._battery_count += 1
            count = self._battery_count
        if count <= 5 or count % 20 == 0:
            self.get_logger().info(
                f"Republished battery sample #{count}: percentage={msg.percentage:.3f}"
            )

    def destroy_node(self) -> bool:
        try:
            self.amcl_sub.undeclare()
            self.battery_sub.undeclare()
            self.session.close()
        finally:
            return super().destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--zenoh-config",
        default="/home/masu_ubu/jazzy_ff_ws/config/zenoh/zenoh_client_config.json5",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = ZenohTopicsToRos(args.zenoh_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
