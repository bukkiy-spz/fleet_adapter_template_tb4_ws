# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''


import math
import threading
import time
from collections import deque

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState

try:
    from irobot_create_msgs.action import Dock as Create3Dock
    from irobot_create_msgs.action import Undock as Create3Undock
    from irobot_create_msgs.msg import DockStatus as Create3DockStatus
    from irobot_create_msgs.msg import IrOpcode as Create3IrOpcode
except ImportError:
    Create3Dock = None
    Create3Undock = None
    Create3DockStatus = None
    Create3IrOpcode = None


class RobotAPI:
    def __init__(self, prefix: str, user: str, password: str, node, api_config=None):
        self.node = node
        self.user = user
        self.password = password
        self.api_config = api_config or {}
        self.namespace = prefix.rstrip("/")
        if not self.namespace.startswith("/"):
            self.namespace = f"/{self.namespace}"

        self.pose_topic = self.api_config.get(
            "pose_topic", f"{self.namespace}/amcl_pose")
        self.battery_topic = self.api_config.get(
            "battery_topic", f"{self.namespace}/battery_state")
        self.navigate_action = self.api_config.get(
            "navigate_action", f"{self.namespace}/navigate_to_pose")
        self.connection_timeout_sec = float(
            self.api_config.get("connection_timeout_sec", 20.0)
        )
        self.pose_wait_timeout_sec = float(
            self.api_config.get("pose_wait_timeout_sec", 20.0)
        )
        self.nav_action_wait_timeout_sec = float(
            self.api_config.get("nav_action_wait_timeout_sec", 20.0)
        )

        self.use_native_dock_action = self._as_bool(
            self.api_config.get("use_native_dock_action", True)
        )
        self.dock_action = self.api_config.get(
            "dock_action", f"{self.namespace}/dock"
        )
        self.dock_action_wait_timeout_sec = float(
            self.api_config.get("dock_action_wait_timeout_sec", 20.0)
        )
        self.dock_goal_wait_timeout_sec = float(
            self.api_config.get("dock_goal_wait_timeout_sec", 5.0)
        )

        self.use_native_undock_action = self._as_bool(
            self.api_config.get("use_native_undock_action", True)
        )
        self.undock_action = self.api_config.get(
            "undock_action", f"{self.namespace}/undock"
        )
        self.undock_action_wait_timeout_sec = float(
            self.api_config.get("undock_action_wait_timeout_sec", 20.0)
        )
        self.undock_goal_wait_timeout_sec = float(
            self.api_config.get("undock_goal_wait_timeout_sec", 5.0)
        )
        self.auto_undock_before_navigate = self._as_bool(
            self.api_config.get("auto_undock_before_navigate", True)
        )

        self.dock_status_topic = self.api_config.get(
            "dock_status_topic", f"{self.namespace}/dock_status"
        )
        self.ir_opcode_topic = self.api_config.get(
            "ir_opcode_topic", f"{self.namespace}/ir_opcode"
        )
        self.dock_visible_stable_sec = float(
            self.api_config.get("dock_visible_stable_sec", 0.8)
        )
        self.dock_visible_min_true_samples = int(
            self.api_config.get("dock_visible_min_true_samples", 4)
        )
        self.dock_visible_stable_ratio = float(
            self.api_config.get("dock_visible_stable_ratio", 0.8)
        )
        self.dock_visible_recent_sec = float(
            self.api_config.get("dock_visible_recent_sec", 1.2)
        )
        self.allow_recent_visible_fallback = self._as_bool(
            self.api_config.get("allow_recent_visible_fallback", True)
        )
        self.require_recent_ir_opcode_for_dock = self._as_bool(
            self.api_config.get("require_recent_ir_opcode_for_dock", True)
        )
        self.ir_opcode_window_sec = float(
            self.api_config.get("ir_opcode_window_sec", 1.2)
        )
        self.ir_opcode_min_count = int(
            self.api_config.get("ir_opcode_min_count", 2)
        )
        self.require_charging_state_for_docked = self._as_bool(
            self.api_config.get("require_charging_state_for_docked", True)
        )
        self.min_charging_current_a = float(
            self.api_config.get("min_charging_current_a", 0.03)
        )

        self._lock = threading.Lock()
        self._latest_pose = None
        self._latest_battery = None
        self._latest_battery_current = None
        self._latest_battery_status = None
        self._dock_visible = None
        self._is_docked = None
        self._dock_visible_history = deque(maxlen=200)
        self._last_dock_visible_time = None
        self._ir_opcode_times = deque(maxlen=300)

        self._goal_handle = None
        self._goal_done = True
        self._goal_succeeded = False
        self._goal_pose = None
        self._goal_start_time = None

        self._process_done = True
        self._process_succeeded = False
        self._dock_goal_handle = None
        self._undock_goal_handle = None
        self._last_undock_time = None

        self._max_linear_speed = 0.25

        self._nav_client = ActionClient(
            self.node,
            NavigateToPose,
            self.navigate_action,
        )

        self._dock_client = None
        if self.use_native_dock_action and Create3Dock is not None:
            self._dock_client = ActionClient(
                self.node,
                Create3Dock,
                self.dock_action,
            )

        self._undock_client = None
        if self.use_native_undock_action and Create3Undock is not None:
            self._undock_client = ActionClient(
                self.node,
                Create3Undock,
                self.undock_action,
            )

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self._pose_cb,
            pose_qos,
        )

        self._battery_sub = self.node.create_subscription(
            BatteryState,
            self.battery_topic,
            self._battery_cb,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self._dock_status_sub = None
        if Create3DockStatus is not None:
            self._dock_status_sub = self.node.create_subscription(
                Create3DockStatus,
                self.dock_status_topic,
                self._dock_status_cb,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        self._ir_opcode_sub = None
        if Create3IrOpcode is not None:
            self._ir_opcode_sub = self.node.create_subscription(
                Create3IrOpcode,
                self.ir_opcode_topic,
                self._ir_opcode_cb,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=20,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )

        self.node.get_logger().info(
            "RobotAPI using pose_topic='%s', battery_topic='%s', "
            "navigate_action='%s', dock_action='%s', undock_action='%s', "
            "native_dock=%s, native_undock=%s, ir_opcode_topic='%s'"
            % (
                self.pose_topic,
                self.battery_topic,
                self.navigate_action,
                self.dock_action,
                self.undock_action,
                self.use_native_dock_action,
                self.use_native_undock_action,
                self.ir_opcode_topic,
            )
        )

        self.connected = self.check_connection()
        if self.connected:
            print("Successfully able to query TB4 ROS interface")
        else:
            print("Unable to query TB4 ROS interface")

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._latest_pose = [pose.position.x, pose.position.y, yaw]

    def _battery_cb(self, msg: BatteryState):
        with self._lock:
            self._latest_battery = msg.percentage
            self._latest_battery_current = msg.current
            self._latest_battery_status = msg.power_supply_status

    def _dock_status_cb(self, msg):
        now = time.monotonic()
        with self._lock:
            self._dock_visible = bool(msg.dock_visible)
            self._is_docked = bool(msg.is_docked)
            visible = bool(msg.dock_visible)
            self._dock_visible_history.append((now, visible))
            if visible:
                self._last_dock_visible_time = now

    def _ir_opcode_cb(self, msg):
        if not self._is_docking_ir_opcode(int(msg.opcode)):
            return
        with self._lock:
            self._ir_opcode_times.append(time.monotonic())

    @staticmethod
    def _is_docking_ir_opcode(opcode: int):
        docking_opcodes = {
            164,  # CODE_IR_BUOY_GREEN
            168,  # CODE_IR_BUOY_RED
            172,  # CODE_IR_BUOY_BOTH
            244,  # CODE_IR_EVAC_GREEN_FIELD
            248,  # CODE_IR_EVAC_RED_FIELD
            252,  # CODE_IR_EVAC_BOTH_FIELD
        }
        return opcode in docking_opcodes

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def check_connection(self):
        if not self._nav_client.wait_for_server(
            timeout_sec=self.nav_action_wait_timeout_sec
        ):
            return False

        deadline = time.time() + self.pose_wait_timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            with self._lock:
                if self._latest_pose is not None:
                    return True

        return False

    def position(self, robot_name: str):
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            return None

        # Template側は theta を degree 想定で扱う
        return [pose[0], pose[1], math.degrees(pose[2])]

    def navigate(self, robot_name: str, pose, map_name: str):
        if len(pose) < 3:
            return False

        if not self._nav_client.wait_for_server(
            timeout_sec=self.nav_action_wait_timeout_sec
        ):
            return False

        if self.auto_undock_before_navigate:
            with self._lock:
                docked = self._is_docked
            if docked is True:
                self.node.get_logger().info(
                    f"Robot [{robot_name}] is docked; requesting undock before navigation"
                )
                if not self.start_undock(robot_name):
                    self.node.get_logger().warn(
                        f"Unable to undock robot [{robot_name}] before navigation"
                    )
                    return False
                deadline = time.time() + 30.0
                while time.time() < deadline:
                    with self._lock:
                        done = self._process_done
                        ok = self._process_succeeded
                    if done:
                        if not ok:
                            self.node.get_logger().warn(
                                f"Undock failed for robot [{robot_name}]"
                            )
                            return False
                        break
                    time.sleep(0.1)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose[0])
        goal.pose.pose.position.y = float(pose[1])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(pose[2]) / 2.0)

        with self._lock:
            self._goal_done = False
            self._goal_succeeded = False
            self._goal_pose = [float(pose[0]), float(pose[1]), float(pose[2])]
            self._goal_start_time = time.time()

        future = self._nav_client.send_goal_async(goal)

        deadline = time.time() + 5.0
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.05)

        if not future.done():
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                self._goal_done = True
            return False

        with self._lock:
            self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        return True

    def _result_cb(self, future):
        succeeded = False
        try:
            result = future.result()
            if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
                succeeded = True
        except Exception:
            succeeded = False

        with self._lock:
            self._goal_done = True
            self._goal_succeeded = succeeded
            self._goal_handle = None
            self._goal_pose = None
            self._goal_start_time = None

    def start_process(self, robot_name: str, process: str, map_name: str):
        if not self.supports_native_docking(process):
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        if self.docked_confident(robot_name):
            # Already docked and charging-confirmed. Treat as success and avoid
            # triggering a redundant re-dock behavior.
            with self._lock:
                self._process_done = True
                self._process_succeeded = True
            return True

        if not self._dock_client.wait_for_server(
            timeout_sec=self.dock_action_wait_timeout_sec
        ):
            self.node.get_logger().error(
                f"Dock action server [{self.dock_action}] is unavailable"
            )
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        goal = Create3Dock.Goal()

        with self._lock:
            self._process_done = False
            self._process_succeeded = False

        future = self._dock_client.send_goal_async(goal)
        deadline = time.time() + self.dock_goal_wait_timeout_sec
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.05)

        if not future.done():
            self.node.get_logger().error("Dock goal request timed out")
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Dock goal was rejected")
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        with self._lock:
            self._dock_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)
        return True

    def _dock_result_cb(self, future):
        succeeded = False
        try:
            result = future.result()
            status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
            dock_result = getattr(result, "result", None)
            is_docked = bool(getattr(dock_result, "is_docked", False))
            succeeded = (
                status == GoalStatus.STATUS_SUCCEEDED and is_docked
            )
        except Exception:
            succeeded = False

        with self._lock:
            self._process_done = True
            self._process_succeeded = succeeded
            self._dock_goal_handle = None
            if succeeded:
                self._is_docked = True

    def start_undock(self, robot_name: str):
        if self._undock_client is None:
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        with self._lock:
            is_docked = self._is_docked

        if is_docked is False:
            with self._lock:
                self._process_done = True
                self._process_succeeded = True
                self._last_undock_time = time.time()
            return True

        if not self._undock_client.wait_for_server(
            timeout_sec=self.undock_action_wait_timeout_sec
        ):
            self.node.get_logger().error(
                f"Undock action server [{self.undock_action}] is unavailable"
            )
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        goal = Create3Undock.Goal()

        with self._lock:
            self._process_done = False
            self._process_succeeded = False

        future = self._undock_client.send_goal_async(goal)
        deadline = time.time() + self.undock_goal_wait_timeout_sec
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.05)

        if not future.done():
            self.node.get_logger().error("Undock goal request timed out")
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("Undock goal was rejected")
            with self._lock:
                self._process_done = True
                self._process_succeeded = False
            return False

        with self._lock:
            self._undock_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_cb)
        return True

    def _undock_result_cb(self, future):
        succeeded = False
        try:
            result = future.result()
            status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
            undock_result = getattr(result, "result", None)
            is_docked = bool(getattr(undock_result, "is_docked", True))
            succeeded = (
                status == GoalStatus.STATUS_SUCCEEDED and (not is_docked)
            )
        except Exception:
            succeeded = False

        with self._lock:
            self._process_done = True
            self._process_succeeded = succeeded
            self._undock_goal_handle = None
            if succeeded:
                self._is_docked = False
                self._last_undock_time = time.time()

    def stop(self, robot_name: str):
        with self._lock:
            goal_handle = self._goal_handle
            dock_goal_handle = self._dock_goal_handle
            undock_goal_handle = self._undock_goal_handle

        if goal_handle is not None:
            future = goal_handle.cancel_goal_async()
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if future.done():
                    break
                time.sleep(0.05)

        if dock_goal_handle is not None:
            dock_future = dock_goal_handle.cancel_goal_async()
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if dock_future.done():
                    break
                time.sleep(0.05)

        if undock_goal_handle is not None:
            undock_future = undock_goal_handle.cancel_goal_async()
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if undock_future.done():
                    break
                time.sleep(0.05)

        with self._lock:
            self._goal_done = True
            self._goal_succeeded = False
            self._goal_handle = None
            self._goal_pose = None
            self._goal_start_time = None
            self._process_done = True
            self._process_succeeded = False
            self._dock_goal_handle = None
            self._undock_goal_handle = None

        return True

    def navigation_remaining_duration(self, robot_name: str):
        with self._lock:
            pose = self._latest_pose
            goal_pose = self._goal_pose

        if pose is None or goal_pose is None:
            return 0.0

        dx = goal_pose[0] - pose[0]
        dy = goal_pose[1] - pose[1]
        distance = math.hypot(dx, dy)
        return distance / self._max_linear_speed

    def navigation_completed(self, robot_name: str):
        with self._lock:
            return self._goal_done and self._goal_succeeded

    def navigation_request_completed(self, robot_name: str):
        with self._lock:
            return self._goal_done

    def navigation_succeeded(self, robot_name: str):
        with self._lock:
            return self._goal_succeeded

    def process_completed(self, robot_name: str):
        with self._lock:
            return self._process_done and self._process_succeeded

    def process_request_completed(self, robot_name: str):
        with self._lock:
            return self._process_done

    def process_succeeded(self, robot_name: str):
        with self._lock:
            return self._process_succeeded

    def docking_completed(self, robot_name: str):
        with self._lock:
            return self._process_done

    def supports_native_docking(self, process: str):
        process_name = (process or "").strip()
        if process_name == "":
            return False
        return self.use_native_dock_action and self._dock_client is not None

    def dock_visible(self, robot_name: str):
        with self._lock:
            return self._dock_visible

    def is_docked(self, robot_name: str):
        with self._lock:
            return self._is_docked

    def is_actively_charging(self, robot_name: str):
        with self._lock:
            status = self._latest_battery_status
            current = self._latest_battery_current
        if status is not None and status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            return True
        if current is not None and not math.isnan(current) and current > self.min_charging_current_a:
            return True
        if status is not None and status in (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        ):
            return False
        return None

    def has_recent_dock_ir(self):
        with self._lock:
            now = time.monotonic()
            while self._ir_opcode_times and (now - self._ir_opcode_times[0]) > self.ir_opcode_window_sec:
                self._ir_opcode_times.popleft()
            return len(self._ir_opcode_times) >= self.ir_opcode_min_count

    def dock_visible_stable(self):
        with self._lock:
            now = time.monotonic()
            recent = [v for (t, v) in self._dock_visible_history if (now - t) <= self.dock_visible_stable_sec]
            latest = self._dock_visible
        if latest is not True:
            return False
        if len(recent) < self.dock_visible_min_true_samples:
            return False
        true_count = sum(1 for v in recent if v)
        ratio = true_count / len(recent)
        return (
            true_count >= self.dock_visible_min_true_samples
            and ratio >= self.dock_visible_stable_ratio
        )

    def dock_visible_recent(self):
        with self._lock:
            if self._dock_visible is True:
                return True
            t = self._last_dock_visible_time
        if t is None:
            return False
        return (time.monotonic() - t) <= self.dock_visible_recent_sec

    def dock_detection_confident(self, robot_name: str):
        if self.is_docked(robot_name) is True:
            return True
        visible = self.dock_visible_stable()
        if (not visible) and self.allow_recent_visible_fallback:
            visible = self.dock_visible_recent()
        if not visible:
            return False
        if self.require_recent_ir_opcode_for_dock and not self.has_recent_dock_ir():
            return False
        return True

    def docked_confident(self, robot_name: str):
        if self.is_docked(robot_name) is not True:
            return False
        if not self.require_charging_state_for_docked:
            return True
        charging = self.is_actively_charging(robot_name)
        return charging is True

    def just_undocked(self, window_sec: float = 15.0):
        with self._lock:
            last_undock_time = self._last_undock_time
        if last_undock_time is None:
            return False
        return (time.time() - last_undock_time) <= float(window_sec)

    def battery_soc(self, robot_name: str):
        with self._lock:
            battery = self._latest_battery

        if battery is None:
            return None

        if battery > 1.0:
            return battery / 100.0
        return battery
