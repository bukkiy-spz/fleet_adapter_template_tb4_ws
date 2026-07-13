#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseRecorder(Node):
    def __init__(self, topic: str):
        super().__init__("record_reference_pose")
        self._message = None
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

    def _callback(self, msg: PoseWithCovarianceStamped):
        self._message = msg


def main():
    parser = argparse.ArgumentParser(
        description="Print the current amcl pose in a YAML-friendly format."
    )
    parser.add_argument(
        "--topic",
        default="/robot2/amcl_pose",
        help="Pose topic to subscribe to",
    )
    parser.add_argument(
        "--label",
        default="POINT",
        help="Label to include in the output comment",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for a pose",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PoseRecorder(args.topic)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node._message is not None:
                msg = node._message
                pose = msg.pose.pose
                yaw = yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                print(f"# {args.label}")
                print(
                    f"- [{pose.position.x:.6f}, {pose.position.y:.6f}]"
                    f"  # yaw_rad={yaw:.6f}"
                )
                return

        raise SystemExit(f"No pose received on {args.topic} within {args.timeout}s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
