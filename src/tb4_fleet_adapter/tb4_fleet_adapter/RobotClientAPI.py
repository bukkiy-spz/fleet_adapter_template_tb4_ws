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

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState


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

        self._lock = threading.Lock()
        self._latest_pose = None
        self._latest_battery = None
        self._goal_handle = None
        self._goal_done = True
        self._goal_succeeded = False
        self._goal_pose = None
        self._goal_start_time = None
        self._process_done = True
        self._max_linear_speed = 0.25

        self._nav_client = ActionClient(
            self.node,
            NavigateToPose,
            self.navigate_action,
        )

        # AMCL publishes a latched pose on a transient-local topic. Use a
        # matching QoS so the adapter can receive the latest pose immediately
        # even if the robot is stationary when the adapter starts.
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

        self.node.get_logger().info(
            "RobotAPI using pose_topic='%s', battery_topic='%s', navigate_action='%s'"
            % (self.pose_topic, self.battery_topic, self.navigate_action)
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

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

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
            if result is not None and result.status == 4:
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
        # 最初は dock / clean / delivery 未対応
        with self._lock:
            self._process_done = True
        return True

    def stop(self, robot_name: str):
        with self._lock:
            goal_handle = self._goal_handle

        if goal_handle is None:
            return True

        future = goal_handle.cancel_goal_async()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.05)

        with self._lock:
            self._goal_done = True
            self._goal_succeeded = False
            self._goal_handle = None
            self._goal_pose = None
            self._goal_start_time = None

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
        return True

    def docking_completed(self, robot_name: str):
        with self._lock:
            return self._process_done

    def battery_soc(self, robot_name: str):
        with self._lock:
            battery = self._latest_battery

        if battery is None:
            return None

        if battery > 1.0:
            return battery / 100.0
        return battery
