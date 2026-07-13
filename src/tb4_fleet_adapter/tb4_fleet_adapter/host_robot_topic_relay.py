"""Relay robot-discovered ROS topics to local adapter topics on the host."""

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class HostRobotTopicRelay(Node):
    def __init__(self) -> None:
        super().__init__("host_robot_topic_relay")

        self.declare_parameter("source_pose_topic", "/robot2/amcl_pose")
        self.declare_parameter("target_pose_topic", "/robot2/amcl_pose_local")
        self.declare_parameter("source_battery_topic", "/robot2/battery_state")
        self.declare_parameter("target_battery_topic", "/robot2/battery_state_local")

        pose_qos = QoSProfile(
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

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("target_pose_topic").value),
            pose_qos,
        )
        self.battery_pub = self.create_publisher(
            BatteryState,
            str(self.get_parameter("target_battery_topic").value),
            battery_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("source_pose_topic").value),
            self._pose_cb,
            pose_qos,
        )
        self.create_subscription(
            BatteryState,
            str(self.get_parameter("source_battery_topic").value),
            self._battery_cb,
            battery_qos,
        )

        self._pose_count = 0
        self._battery_count = 0
        self.get_logger().info("Relaying robot topics to local adapter topics")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose_pub.publish(msg)
        self._pose_count += 1
        if self._pose_count <= 5 or self._pose_count % 50 == 0:
            pose = msg.pose.pose.position
            self.get_logger().info(
                f"Pose relay #{self._pose_count}: x={pose.x:.3f}, y={pose.y:.3f}"
            )

    def _battery_cb(self, msg: BatteryState) -> None:
        self.battery_pub.publish(msg)
        self._battery_count += 1
        if self._battery_count <= 5 or self._battery_count % 20 == 0:
            self.get_logger().info(
                f"Battery relay #{self._battery_count}: percentage={msg.percentage:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HostRobotTopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
