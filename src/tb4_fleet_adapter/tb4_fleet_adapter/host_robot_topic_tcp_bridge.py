"""Bridge robot ROS topics across two host ROS discovery environments.

The sender runs in the TurtleBot discovery-server environment and forwards
serialized messages over localhost TCP. The receiver runs in the clean RMF
environment and republishes them as adapter-local ROS topics.
"""

import socket
import struct
import threading
import time
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import BatteryState


POSE_MSG = 1
BATTERY_MSG = 2
HEADER = struct.Struct("!BI")


def _pose_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _battery_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _recv_exact(sock: socket.socket, size: int) -> Optional[bytes]:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class HostRobotTopicTcpSender(Node):
    def __init__(self) -> None:
        super().__init__("host_robot_topic_tcp_sender")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("source_pose_topic", "/robot2/amcl_pose")
        self.declare_parameter("source_battery_topic", "/robot2/battery_state")

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._sock: Optional[socket.socket] = None
        self._sock_lock = threading.Lock()
        self._stop = threading.Event()
        self._pose_count = 0
        self._battery_count = 0

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("source_pose_topic").value),
            self._pose_cb,
            _pose_qos(),
        )
        self.create_subscription(
            BatteryState,
            str(self.get_parameter("source_battery_topic").value),
            self._battery_cb,
            _battery_qos(),
        )

        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._connect_thread.start()
        self.get_logger().info(
            f"Forwarding robot topics to TCP receiver at {self._host}:{self._port}"
        )

    def close(self) -> None:
        self._stop.set()
        with self._sock_lock:
            if self._sock:
                self._sock.close()
                self._sock = None

    def _connect_loop(self) -> None:
        while not self._stop.is_set():
            with self._sock_lock:
                connected = self._sock is not None
            if connected:
                time.sleep(1.0)
                continue

            try:
                sock = socket.create_connection((self._host, self._port), timeout=2.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                time.sleep(1.0)
                continue

            with self._sock_lock:
                self._sock = sock
            self.get_logger().info("Connected to clean-env TCP receiver")

    def _send(self, msg_type: int, payload: bytes) -> None:
        packet = HEADER.pack(msg_type, len(payload)) + payload
        with self._sock_lock:
            sock = self._sock
            if sock is None:
                return
            try:
                sock.sendall(packet)
            except OSError:
                sock.close()
                self._sock = None
                self.get_logger().warn("TCP receiver disconnected; retrying")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._send(POSE_MSG, serialize_message(msg))
        self._pose_count += 1
        if self._pose_count <= 5 or self._pose_count % 50 == 0:
            p = msg.pose.pose.position
            self.get_logger().info(
                f"Pose TCP send #{self._pose_count}: x={p.x:.3f}, y={p.y:.3f}"
            )

    def _battery_cb(self, msg: BatteryState) -> None:
        self._send(BATTERY_MSG, serialize_message(msg))
        self._battery_count += 1
        if self._battery_count <= 5 or self._battery_count % 20 == 0:
            self.get_logger().info(
                f"Battery TCP send #{self._battery_count}: percentage={msg.percentage:.3f}"
            )


class HostRobotTopicTcpReceiver(Node):
    def __init__(self) -> None:
        super().__init__("host_robot_topic_tcp_receiver")

        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("target_pose_topic", "/robot2/amcl_pose_local")
        self.declare_parameter("target_battery_topic", "/robot2/battery_state_local")

        self._bind_host = str(self.get_parameter("bind_host").value)
        self._port = int(self.get_parameter("port").value)
        self._stop = threading.Event()
        self._pose_count = 0
        self._battery_count = 0

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("target_pose_topic").value),
            _pose_qos(),
        )
        self.battery_pub = self.create_publisher(
            BatteryState,
            str(self.get_parameter("target_battery_topic").value),
            _battery_qos(),
        )

        self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            f"Publishing TCP robot topics on clean ROS graph from {self._bind_host}:{self._port}"
        )

    def close(self) -> None:
        self._stop.set()

    def _server_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server.bind((self._bind_host, self._port))
                    server.listen(1)
                    server.settimeout(1.0)
                    self._accept_loop(server)
            except OSError as exc:
                self.get_logger().warn(f"TCP receiver error: {exc}; retrying")
                time.sleep(1.0)

    def _accept_loop(self, server: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            except OSError:
                return

            self.get_logger().info(f"Accepted robot topic sender from {address}")
            with client:
                self._client_loop(client)

    def _client_loop(self, client: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                header = _recv_exact(client, HEADER.size)
                if header is None:
                    return
                msg_type, size = HEADER.unpack(header)
                payload = _recv_exact(client, size)
                if payload is None:
                    return
            except (OSError, struct.error):
                return

            if msg_type == POSE_MSG:
                msg = deserialize_message(payload, PoseWithCovarianceStamped)
                self.pose_pub.publish(msg)
                self._pose_count += 1
                if self._pose_count <= 5 or self._pose_count % 50 == 0:
                    p = msg.pose.pose.position
                    self.get_logger().info(
                        f"Pose TCP receive #{self._pose_count}: x={p.x:.3f}, y={p.y:.3f}"
                    )
            elif msg_type == BATTERY_MSG:
                msg = deserialize_message(payload, BatteryState)
                self.battery_pub.publish(msg)
                self._battery_count += 1
                if self._battery_count <= 5 or self._battery_count % 20 == 0:
                    self.get_logger().info(
                        f"Battery TCP receive #{self._battery_count}: percentage={msg.percentage:.3f}"
                    )


def sender_main(args=None) -> None:
    rclpy.init(args=args)
    node = HostRobotTopicTcpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def receiver_main(args=None) -> None:
    rclpy.init(args=args)
    node = HostRobotTopicTcpReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
