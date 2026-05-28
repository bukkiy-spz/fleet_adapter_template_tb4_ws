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

from rclpy.duration import Duration

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary

import numpy as np

import threading
import math
import copy
import enum
import time
import os

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        # battery topic が一時的に途切れても毎周期 error を出し続けないようにする。
        self._warned_missing_battery = False
        self.api = api
        self.position = position  # (x,y,theta) in RMF coordinates (meters, radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter
        self._startup_time = time.time()
        self._startup_path_suppressed = False
        self._suppress_unsolicited_startup_path = bool(
            self.config.get("suppress_unsolicited_startup_path", True)
        )
        self._startup_suppress_waypoint = str(
            self.config.get("startup_suppress_waypoint", "LP1")
        )
        self._startup_path_grace_sec = float(
            self.config.get("startup_path_grace_sec", 120.0)
        )
        self._require_manual_dispatch_gate = bool(
            self.config.get("require_manual_dispatch_gate", True)
        )
        self._manual_dispatch_gate_file = str(
            self.config.get(
                "manual_dispatch_gate_file",
                f"/tmp/tb4_manual_dispatch_enabled_{self.name}",
            )
        )
        # adapter 起動直後の初期動作:
        # ドック上なら Undock し、pre_dock へ移動して待機させる。
        self._startup_undock_to_pre_dock = bool(
            self.config.get("startup_undock_to_pre_dock", True)
        )
        self._startup_init_delay_sec = float(
            self.config.get("startup_init_delay_sec", 1.0)
        )
        self._startup_undock_wait_timeout_sec = float(
            self.config.get("startup_undock_wait_timeout_sec", 35.0)
        )
        self._startup_pre_dock_nav_timeout_sec = float(
            self.config.get("startup_pre_dock_nav_timeout_sec", 45.0)
        )
        self._startup_sequence_thread = None
        self._startup_sequence_active = False

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update)

        self.initialized = True
        if self._startup_undock_to_pre_dock:
            self._start_startup_sequence()

    def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def _infer_pre_dock_waypoint_index(self):
        for lane_index in range(self.graph.num_lanes):
            lane = self.graph.get_lane(lane_index)
            if lane.exit.waypoint_index == self.charger_waypoint_index:
                if lane.entry.waypoint_index != self.charger_waypoint_index:
                    return lane.entry.waypoint_index
        return None

    def _start_startup_sequence(self):
        if self._startup_sequence_thread is not None:
            return

        def _run_startup_sequence():
            self._startup_sequence_active = True
            try:
                if self._startup_init_delay_sec > 0.0:
                    time.sleep(self._startup_init_delay_sec)

                # 1) まず Undock を試す（既に undocked なら API 側で即成功扱い）
                docked_state = self.api.is_docked(self.name)
                if docked_state is not False:
                    self.node.get_logger().info(
                        f"Startup sequence: requesting undock for robot [{self.name}]"
                    )
                    undock_deadline = time.time() + self._startup_undock_wait_timeout_sec
                    undock_requested = False
                    while time.time() <= undock_deadline:
                        if self.api.start_undock(self.name):
                            undock_requested = True
                            break
                        if self.api.is_docked(self.name) is False:
                            break
                        # undock action server が遅れて立ち上がることがあるため、
                        # 短い間隔で再試行する。
                        time.sleep(1.0)

                    if undock_requested:
                        while not self.api.process_request_completed(self.name):
                            if time.time() > undock_deadline:
                                self.node.get_logger().warn(
                                    f"Startup undock timed out for robot [{self.name}]"
                                )
                                self.api.stop(self.name)
                                return
                            time.sleep(0.1)
                        if not self.api.process_succeeded(self.name):
                            if self.api.is_docked(self.name) is not False:
                                self.node.get_logger().warn(
                                    f"Startup undock failed for robot [{self.name}]"
                                )
                                return
                    else:
                        if self.api.is_docked(self.name) is not False:
                            self.node.get_logger().warn(
                                f"Startup undock request was rejected/unavailable "
                                f"for robot [{self.name}]"
                            )
                            return

                # 2) pre_dock を推定し、ドック向きに整列して待機
                pre_dock_waypoint_index = self._infer_pre_dock_waypoint_index()
                if pre_dock_waypoint_index is None:
                    self.node.get_logger().warn(
                        f"Startup sequence: pre_dock waypoint is not found for robot [{self.name}]"
                    )
                    return

                pre_wp = self.graph.get_waypoint(pre_dock_waypoint_index)
                charger_wp = self.graph.get_waypoint(self.charger_waypoint_index)
                pre_x, pre_y = pre_wp.location[:2]
                charger_x, charger_y = charger_wp.location[:2]
                theta_rmf = math.atan2(charger_y - pre_y, charger_x - pre_x)
                pre_x_robot, pre_y_robot = self.transforms["rmf_to_robot"].transform(
                    [pre_x, pre_y]
                )
                theta_robot = theta_rmf + self.transforms["orientation_offset"]

                self.node.get_logger().info(
                    f"Startup sequence: navigating robot [{self.name}] to pre_dock"
                )
                if not self.api.navigate(
                    self.name,
                    [pre_x_robot, pre_y_robot, theta_robot],
                    self.map_name,
                ):
                    self.node.get_logger().warn(
                        f"Startup sequence: failed to send pre_dock navigation for robot [{self.name}]"
                    )
                    return

                nav_deadline = time.time() + self._startup_pre_dock_nav_timeout_sec
                while not self.api.navigation_request_completed(self.name):
                    if time.time() > nav_deadline:
                        self.node.get_logger().warn(
                            f"Startup sequence: timeout while going to pre_dock for robot [{self.name}]"
                        )
                        self.api.stop(self.name)
                        return
                    time.sleep(0.1)

                if not self.api.navigation_succeeded(self.name):
                    self.node.get_logger().warn(
                        f"Startup sequence: robot [{self.name}] failed to reach pre_dock"
                    )
                    return

                with self._lock:
                    self.on_waypoint = pre_dock_waypoint_index
                    self.last_known_waypoint_index = pre_dock_waypoint_index
                    self.on_lane = None
                    self.state = RobotState.WAITING
                self.node.get_logger().info(
                    f"Startup sequence completed: robot [{self.name}] is waiting at pre_dock"
                )
            finally:
                self._startup_sequence_active = False

        self._startup_sequence_thread = threading.Thread(
            target=_run_startup_sequence,
            daemon=True,
        )
        self._startup_sequence_thread.start()

    def _manual_dispatch_gate_enabled(self):
        if not self._require_manual_dispatch_gate:
            return True
        return os.path.exists(self._manual_dispatch_gate_file)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            self.state = RobotState.IDLE

    def stop(self):
        # Stop the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)
        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()

    def follow_new_path(
        self,
        waypoints,
        next_arrival_estimator,
        path_finished_callback):

        # Suppress a known startup artifact: immediately after adapter launch,
        # RMF may issue an unsolicited LP1 path before an operator dispatches
        # any task. Drop this one-time path and keep the robot in place.
        if self._startup_sequence_active:
            self.node.get_logger().warn(
                f"Suppressing path while startup sequence is active for "
                f"robot [{self.name}]"
            )
            assert path_finished_callback is not None
            path_finished_callback()
            return

        if self._suppress_unsolicited_startup_path and not self._startup_path_suppressed:
            elapsed = time.time() - self._startup_time
            if elapsed <= self._startup_path_grace_sec and len(waypoints) > 0:
                if (
                    self._require_manual_dispatch_gate
                    and (not os.path.exists(self._manual_dispatch_gate_file))
                ):
                    self.node.get_logger().warn(
                        f"Suppressing startup path before manual dispatch gate "
                        f"[{self._manual_dispatch_gate_file}] is enabled for "
                        f"robot [{self.name}]"
                    )
                    assert path_finished_callback is not None
                    path_finished_callback()
                    return
                final_wp = waypoints[-1]
                final_index = final_wp.graph_index
                final_name = None
                if final_index is not None:
                    try:
                        final_name = self.graph.get_waypoint(final_index).waypoint_name
                    except Exception:
                        final_name = None
                if final_name == self._startup_suppress_waypoint:
                    if not self._manual_dispatch_gate_enabled():
                        self._startup_path_suppressed = True
                        self.node.get_logger().warn(
                            f"Suppressing unsolicited startup path to [{final_name}] "
                            f"for robot [{self.name}]"
                        )
                        assert path_finished_callback is not None
                        path_finished_callback()
                        return

        self.stop()
        self._quit_path_event.clear()

        self.node.get_logger().info("Received new path to follow...")

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        if len(self.remaining_waypoints) > 1:
            # Remove consecutive duplicate waypoints that can appear around
            # dock-related paths on some RMF versions.
            deduped = []
            prev_index = None
            for pair in self.remaining_waypoints:
                wp_index = pair[1].graph_index
                if prev_index is not None and wp_index == prev_index:
                    continue
                deduped.append(pair)
                prev_index = wp_index
            self.remaining_waypoints = deduped

        if len(self.remaining_waypoints) > 1 and self.api.just_undocked(20.0):
            pre_dock_waypoint_index = self._infer_pre_dock_waypoint_index()

            skip_waypoints = {self.charger_waypoint_index}
            if pre_dock_waypoint_index is not None:
                skip_waypoints.add(pre_dock_waypoint_index)

            final_graph_index = self.remaining_waypoints[-1][1].graph_index
            while len(self.remaining_waypoints) > 1:
                first_graph_index = self.remaining_waypoints[0][1].graph_index
                if (
                    first_graph_index in skip_waypoints
                    and final_graph_index not in skip_waypoints
                ):
                    self.node.get_logger().warn(
                        f"Skipping immediate dock-area waypoint [{first_graph_index}] "
                        f"after undock for "
                        f"robot [{self.name}]"
                    )
                    self.remaining_waypoints = self.remaining_waypoints[1:]
                    continue
                break
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        def _follow_path():
            target_pose = []
            while (
                self.remaining_waypoints or
                self.state == RobotState.MOVING or
                self.state == RobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed "
                                                "path")
                    return
                # State machine
                if self.state == RobotState.IDLE:
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0][1]
                    self.path_index = self.remaining_waypoints[0][0]
                    # Move robot to next waypoint
                    target_pose = self.target_waypoint.position
                    [x, y] = self.transforms["rmf_to_robot"].transform(
                        target_pose[:2])
                    theta = target_pose[2] + \
                        self.transforms['orientation_offset']
                    # ------------------------ #
                    # IMPLEMENT YOUR CODE HERE #
                    # Ensure x, y, theta are in units that api.navigate() #
                    # ------------------------ #
                    response = self.api.navigate(self.name,
                                                 [x, y, theta],
                                                 self.map_name)

                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().info(
                            f"Robot {self.name} failed to navigate to "
                            f"[{x:.0f}, {y:.0f}, {theta:.0f}] coordinates. "
                            f"Retrying...")
                        self.sleep_for(0.1)

                elif self.state == RobotState.WAITING:
                    self.sleep_for(0.1)
                    time_now = self.adapter.now()
                    with self._lock:
                        if self.target_waypoint is not None:
                            waypoint_wait_time = self.target_waypoint.time
                            if (waypoint_wait_time < time_now):
                                self.state = RobotState.IDLE
                            else:
                                if self.path_index is not None:
                                    self.node.get_logger().info(
                                        f"Waiting for "
                                        f"{(waypoint_wait_time - time_now).seconds}s")
                                    self.next_arrival_estimator(
                                        self.path_index, timedelta(seconds=0.0))

                elif self.state == RobotState.MOVING:
                    self.sleep_for(0.1)
                    # Check if we have reached the target
                    with self._lock:
                        request_done = self.api.navigation_request_completed(
                            self.name)
                        if request_done and self.api.navigation_completed(self.name):
                            self.node.get_logger().info(
                                f"Robot [{self.name}] has reached its target "
                                f"waypoint")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                                self.last_known_waypoint_index = \
                                    self.on_waypoint
                            else:
                                self.on_waypoint = None  # still on a lane
                        elif request_done and not self.api.navigation_succeeded(self.name):
                            self.node.get_logger().warn(
                                f"Robot [{self.name}] failed to reach its target. "
                                "Retrying current waypoint...")
                            self.state = RobotState.IDLE
                        else:
                            # Update the lane the robot is on
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            else:
                                # The robot may either be on the previous
                                # waypoint or the target one
                                if self.target_waypoint.graph_index is not \
                                    None and self.dist(self.position, target_pose) < 0.5:
                                    self.on_waypoint = self.target_waypoint.graph_index
                                elif self.last_known_waypoint_index is not \
                                    None and self.dist(
                                    self.position, self.graph.get_waypoint(
                                      self.last_known_waypoint_index).location) < 0.5:
                                    self.on_waypoint = self.last_known_waypoint_index
                                else:
                                    self.on_lane = None  # update_off_grid()
                                    self.on_waypoint = None
                            # ------------------------ #
                            # IMPLEMENT YOUR CODE HERE #
                            # If your robot does not have an API to report the
                            # remaining travel duration, replace the API call
                            # below with an estimation
                            # ------------------------ #
                            duration = self.api.navigation_remaining_duration(self.name)
                            if self.path_index is not None:
                                self.next_arrival_estimator(
                                    self.path_index, timedelta(seconds=duration))
            self.path_finished_callback()
            self.node.get_logger().info(
                f"Robot {self.name} has successfully navigated along "
                f"requested path.")

        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    def dock(
            self,
            dock_name,
            docking_finished_callback):
        ''' Docking is very specific to each application. Hence, the user will
            need to customize this function accordingly. In this example, we
            assume the dock_name is the same as the name of the waypoints that
            the robot is trying to dock into. We then call api.start_process()
            to initiate the robot specific process. This could be to start a
            cleaning process or load/unload a cart for delivery.
        '''

        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        if self._startup_sequence_active:
            self.node.get_logger().warn(
                f"Suppressing dock request while startup sequence is active for "
                f"robot [{self.name}]"
            )
            assert docking_finished_callback is not None
            docking_finished_callback()
            return

        elapsed = time.time() - self._startup_time
        if (
            self._suppress_unsolicited_startup_path
            and elapsed <= self._startup_path_grace_sec
            and (not self._manual_dispatch_gate_enabled())
        ):
            self.node.get_logger().warn(
                f"Suppressing startup dock request before manual dispatch gate "
                f"[{self._manual_dispatch_gate_file}] is enabled for "
                f"robot [{self.name}]"
            )
            assert docking_finished_callback is not None
            docking_finished_callback()
            return

        self.dock_name = dock_name
        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback
        finished_cb = docking_finished_callback

        def _invoke_docking_finished_callback():
            if callable(finished_cb):
                finished_cb()
            else:
                self.node.get_logger().warn(
                    f"Docking finished callback is unavailable for robot [{self.name}]"
                )

        # Get the waypoint that the robot is trying to dock into
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index
        dock_x, dock_y = dock_waypoint.location[:2]

        def _dock():
            # Request the robot to start the relevant process
            self.node.get_logger().info(
                f"Requesting robot {self.name} to dock at {self.dock_name}")
            supports_native_docking = self.api.supports_native_docking(self.dock_name)
            if not supports_native_docking:
                self.node.get_logger().warn(
                    f"Native docking is unavailable; waiting at pre_dock for "
                    f"robot [{self.name}]"
                )
                with self._lock:
                    self.dock_waypoint_index = None
                    _invoke_docking_finished_callback()
                return

            # --- ドッキング方針（Create3の推奨に合わせる） ---
            # 1) まず pre_dock へ整列し、ドック方向を向く
            # 2) dock_visible を確認してから native dock を開始する
            # 3) 見えない場合は「その場回転」+「pre_dockから数cm前進」の
            #    小さな探索だけを行い、危険な大移動は避ける
            pre_dock_waypoint_index = None
            pre_x = None
            pre_y = None
            align_to_pre_dock_fn = None
            initial_visibility_failed = False
            if self.dock_waypoint_index == self.charger_waypoint_index:
                pre_dock_waypoint_index = self._infer_pre_dock_waypoint_index()
            else:
                for lane_index in range(self.graph.num_lanes):
                    lane = self.graph.get_lane(lane_index)
                    if lane.exit.waypoint_index == self.dock_waypoint_index:
                        if lane.entry.waypoint_index != self.dock_waypoint_index:
                            pre_dock_waypoint_index = lane.entry.waypoint_index
                            break

            if pre_dock_waypoint_index is not None:
                pre_dock_wp = self.graph.get_waypoint(pre_dock_waypoint_index)
                dock_x, dock_y = dock_waypoint.location[:2]
                pre_x, pre_y = pre_dock_wp.location[:2]
                approach_theta_rmf = math.atan2(dock_y - pre_y, dock_x - pre_x)
                pre_x_robot, pre_y_robot = self.transforms["rmf_to_robot"].transform(
                    [pre_x, pre_y]
                )

                def _align_to_pre_dock(theta_rmf):
                    theta_robot = theta_rmf + self.transforms["orientation_offset"]
                    if not self.api.navigate(
                        self.name,
                        [pre_x_robot, pre_y_robot, theta_robot],
                        self.map_name,
                    ):
                        return False

                    align_deadline = time.time() + 20.0
                    while not self.api.navigation_request_completed(self.name):
                        if self._quit_dock_event.is_set():
                            self.node.get_logger().info("Aborting docking")
                            self.api.stop(self.name)
                            return False
                        if time.time() > align_deadline:
                            self.api.stop(self.name)
                            return False
                        self.sleep_for(0.1)

                    return self.api.navigation_succeeded(self.name)
                align_to_pre_dock_fn = _align_to_pre_dock

                # pre_dock上で向きを少しずつ変えて、dock_visibleを確認する。
                # LP3など進入方向が不利なケースでも、まずはその場で見えるかを
                # 優先して確認する。
                heading_offsets = [
                    0.0,
                    0.35, -0.35,
                    0.70, -0.70,
                    1.05, -1.05,
                    1.40, -1.40,
                    1.75, -1.75,
                    2.10, -2.10,
                    2.45, -2.45,
                    2.80, -2.80,
                    math.pi,
                ]
                dock_visible = self.api.dock_detection_confident(self.name)
                aligned = False
                for offset in heading_offsets:
                    target_theta = approach_theta_rmf + offset
                    if not _align_to_pre_dock(target_theta):
                        continue

                    aligned = True
                    with self._lock:
                        self.on_waypoint = pre_dock_waypoint_index

                    self.sleep_for(0.35)
                    if self.api.dock_detection_confident(self.name):
                        dock_visible = True
                        self.node.get_logger().info(
                            f"Dock became visible for robot [{self.name}] "
                            f"after heading offset {offset:.2f} rad"
                        )
                        break

                if not aligned:
                    self.node.get_logger().warn(
                        f"Failed to align robot [{self.name}] at pre_dock; "
                        "waiting at pre_dock."
                    )
                    with self._lock:
                        self.dock_waypoint_index = None
                        _invoke_docking_finished_callback()
                    return

                if dock_visible is False:
                    initial_visibility_failed = True
                    self.node.get_logger().warn(
                        f"Dock is not visible for robot [{self.name}] even after "
                        "heading scan; switching to pre_dock micro-search."
                    )
            else:
                dock_visible = self.api.dock_detection_confident(self.name)
                if dock_visible is False:
                    initial_visibility_failed = True
                    self.node.get_logger().warn(
                        f"Dock is not visible for robot [{self.name}] and no "
                        "pre_dock alignment waypoint was found; waiting at current "
                        "position for safe retry."
                    )

            def _wait_at_pre_dock_and_finish(message):
                self.node.get_logger().warn(message)
                with self._lock:
                    self.dock_waypoint_index = None
                    _invoke_docking_finished_callback()

            def _recover_to_pre_dock():
                if (
                    pre_dock_waypoint_index is None
                    or align_to_pre_dock_fn is None
                ):
                    return False
                if not align_to_pre_dock_fn(approach_theta_rmf):
                    return False
                with self._lock:
                    self.on_waypoint = pre_dock_waypoint_index
                return True

            search_angle_step = float(self.config.get("dock_search_angle_step_rad", 0.25))
            search_angle_max = float(self.config.get("dock_search_angle_max_rad", math.pi))
            search_settle_sec = float(self.config.get("dock_search_settle_sec", 0.35))
            dock_lost_visibility_timeout_sec = float(
                self.config.get("dock_lost_visibility_timeout_sec", 1.2)
            )
            # pre_dockからの小探索パラメータ（安全重視）。
            # forward_maxは20cm程度を上限にし、壁方向へ流れるリスクを抑える。
            dock_pre_search_forward_step_m = float(
                self.config.get("dock_pre_search_forward_step_m", 0.04)
            )
            dock_pre_search_forward_max_m = float(
                self.config.get("dock_pre_search_forward_max_m", 0.20)
            )
            dock_pre_search_angle_step_rad = float(
                self.config.get("dock_pre_search_angle_step_rad", 0.17)
            )
            dock_pre_search_angle_max_rad = float(
                self.config.get("dock_pre_search_angle_max_rad", 0.70)
            )
            dock_pre_search_settle_sec = float(
                self.config.get("dock_pre_search_settle_sec", 0.40)
            )
            # 過去の大きな前進探索は、衝突リスクが高いためデフォルト無効。
            enable_far_progressive_search = bool(
                self.config.get("dock_enable_far_progressive_search", False)
            )
            dock_approach_step_m = float(self.config.get("dock_approach_step_m", 0.12))
            dock_approach_max_total_m = float(
                self.config.get("dock_approach_max_total_m", 0.60)
            )
            dock_approach_min_standoff_m = float(
                self.config.get("dock_approach_min_standoff_m", 0.18)
            )
            dock_approach_heading_bias_rad = float(
                self.config.get("dock_approach_heading_bias_rad", 0.0)
            )

            def _build_heading_offsets():
                if search_angle_step <= 0.0:
                    return [0.0]
                offsets = [0.0]
                angle = search_angle_step
                while angle <= search_angle_max + 1e-6:
                    offsets.append(angle)
                    offsets.append(-angle)
                    angle += search_angle_step
                if math.pi not in offsets:
                    offsets.append(math.pi)
                return offsets

            def _build_pre_search_heading_offsets():
                # pre_dock小探索では、180度反転までは回さず、ドック正面付近だけを
                # 細かく探索する（誤検知と迷走を抑える）。
                if dock_pre_search_angle_step_rad <= 0.0:
                    return [0.0]
                offsets = [0.0]
                angle = dock_pre_search_angle_step_rad
                while angle <= dock_pre_search_angle_max_rad + 1e-6:
                    offsets.append(angle)
                    offsets.append(-angle)
                    angle += dock_pre_search_angle_step_rad
                return offsets

            def _scan_dock_visibility():
                if (
                    pre_dock_waypoint_index is None
                    or align_to_pre_dock_fn is None
                ):
                    return self.api.dock_detection_confident(self.name)

                heading_offsets = _build_heading_offsets()
                for offset in heading_offsets:
                    target_theta = approach_theta_rmf + offset
                    if not align_to_pre_dock_fn(target_theta):
                        continue
                    with self._lock:
                        self.on_waypoint = pre_dock_waypoint_index
                    self.sleep_for(search_settle_sec)
                    if self.api.dock_detection_confident(self.name):
                        self.node.get_logger().info(
                            f"Dock re-acquired for robot [{self.name}] "
                            f"after heading offset {offset:.2f} rad"
                        )
                        return True
                return False

            def _scan_dock_visibility_in_place():
                with self._lock:
                    current_position = copy.copy(self.position)
                center_x_rmf = float(current_position[0])
                center_y_rmf = float(current_position[1])
                dx = dock_x - center_x_rmf
                dy = dock_y - center_y_rmf
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    base_theta_rmf = float(current_position[2])
                else:
                    base_theta_rmf = math.atan2(dy, dx)
                center_x_robot, center_y_robot = self.transforms["rmf_to_robot"].transform(
                    [center_x_rmf, center_y_rmf]
                )

                heading_offsets = _build_heading_offsets()
                for offset in heading_offsets:
                    target_theta_robot = (
                        base_theta_rmf + offset + self.transforms["orientation_offset"]
                    )
                    if not self.api.navigate(
                        self.name,
                        [center_x_robot, center_y_robot, target_theta_robot],
                        self.map_name,
                    ):
                        continue

                    deadline = time.time() + 15.0
                    while not self.api.navigation_request_completed(self.name):
                        if self._quit_dock_event.is_set():
                            self.node.get_logger().info("Aborting docking")
                            self.api.stop(self.name)
                            return False
                        if time.time() > deadline:
                            self.api.stop(self.name)
                            break
                        self.sleep_for(0.05)

                    if not self.api.navigation_succeeded(self.name):
                        continue

                    self.sleep_for(search_settle_sec)
                    if self.api.dock_detection_confident(self.name):
                        self.node.get_logger().info(
                            f"Dock re-acquired in-place for robot [{self.name}] "
                            f"after heading offset {offset:.2f} rad"
                        )
                        return True
                return False

            def _micro_search_from_pre_dock():
                # pre_dockを基準に「数cm前進 -> 小さく首振り」を繰り返す。
                # 視界が得られた時点で即座にDockへ移る。
                if (
                    pre_dock_waypoint_index is None
                    or align_to_pre_dock_fn is None
                    or pre_x is None
                    or pre_y is None
                ):
                    return False

                if not _recover_to_pre_dock():
                    return False

                ux = math.cos(approach_theta_rmf)
                uy = math.sin(approach_theta_rmf)
                moved_total = 0.0
                heading_offsets = _build_pre_search_heading_offsets()

                while moved_total <= dock_pre_search_forward_max_m + 1e-6:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info("Aborting docking")
                        self.api.stop(self.name)
                        return False

                    target_x = pre_x + ux * moved_total
                    target_y = pre_y + uy * moved_total
                    target_theta_rmf = math.atan2(dock_y - target_y, dock_x - target_x)
                    target_x_robot, target_y_robot = self.transforms["rmf_to_robot"].transform(
                        [target_x, target_y]
                    )

                    # まずはドック正面方向へ向けて姿勢を作る
                    if not self.api.navigate(
                        self.name,
                        [
                            target_x_robot,
                            target_y_robot,
                            target_theta_rmf + self.transforms["orientation_offset"],
                        ],
                        self.map_name,
                    ):
                        moved_total += dock_pre_search_forward_step_m
                        continue

                    deadline = time.time() + 20.0
                    while not self.api.navigation_request_completed(self.name):
                        if self._quit_dock_event.is_set():
                            self.node.get_logger().info("Aborting docking")
                            self.api.stop(self.name)
                            return False
                        if time.time() > deadline:
                            self.api.stop(self.name)
                            break
                        self.sleep_for(0.05)

                    if not self.api.navigation_succeeded(self.name):
                        moved_total += dock_pre_search_forward_step_m
                        continue

                    self.sleep_for(dock_pre_search_settle_sec)
                    if self.api.dock_detection_confident(self.name):
                        self.node.get_logger().info(
                            f"Dock became visible during pre_dock micro-search "
                            f"at +{moved_total:.2f} m for robot [{self.name}]"
                        )
                        return True

                    # 同じ位置で小さく首振りして再確認
                    for offset in heading_offsets:
                        theta_robot = (
                            target_theta_rmf
                            + offset
                            + self.transforms["orientation_offset"]
                        )
                        if not self.api.navigate(
                            self.name,
                            [target_x_robot, target_y_robot, theta_robot],
                            self.map_name,
                        ):
                            continue

                        deadline = time.time() + 12.0
                        while not self.api.navigation_request_completed(self.name):
                            if self._quit_dock_event.is_set():
                                self.node.get_logger().info("Aborting docking")
                                self.api.stop(self.name)
                                return False
                            if time.time() > deadline:
                                self.api.stop(self.name)
                                break
                            self.sleep_for(0.05)

                        if not self.api.navigation_succeeded(self.name):
                            continue
                        self.sleep_for(dock_pre_search_settle_sec)
                        if self.api.dock_detection_confident(self.name):
                            self.node.get_logger().info(
                                f"Dock became visible after micro heading offset "
                                f"{offset:.2f} rad at +{moved_total:.2f} m"
                            )
                            return True

                    moved_total += dock_pre_search_forward_step_m

                return False

            def _progressive_approach_and_scan():
                with self._lock:
                    current_position = copy.copy(self.position)
                cur_x = float(current_position[0])
                cur_y = float(current_position[1])
                dx = dock_x - cur_x
                dy = dock_y - cur_y
                current_distance = math.hypot(dx, dy)
                if current_distance < dock_approach_min_standoff_m + 0.05:
                    return _scan_dock_visibility_in_place()

                moved_total = 0.0
                while moved_total < dock_approach_max_total_m:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info("Aborting docking")
                        self.api.stop(self.name)
                        return False

                    step = min(dock_approach_step_m, dock_approach_max_total_m - moved_total)
                    moved_total += step
                    next_distance = max(
                        dock_approach_min_standoff_m,
                        current_distance - moved_total,
                    )

                    vx = cur_x - dock_x
                    vy = cur_y - dock_y
                    v_norm = math.hypot(vx, vy)
                    if v_norm < 1e-6:
                        break
                    ux = vx / v_norm
                    uy = vy / v_norm
                    target_x = dock_x + ux * next_distance
                    target_y = dock_y + uy * next_distance
                    target_theta_rmf = math.atan2(dock_y - target_y, dock_x - target_x)
                    target_theta_robot = (
                        target_theta_rmf
                        + dock_approach_heading_bias_rad
                        + self.transforms["orientation_offset"]
                    )
                    target_x_robot, target_y_robot = self.transforms["rmf_to_robot"].transform(
                        [target_x, target_y]
                    )

                    if not self.api.navigate(
                        self.name,
                        [target_x_robot, target_y_robot, target_theta_robot],
                        self.map_name,
                    ):
                        continue

                    deadline = time.time() + 20.0
                    while not self.api.navigation_request_completed(self.name):
                        if self._quit_dock_event.is_set():
                            self.node.get_logger().info("Aborting docking")
                            self.api.stop(self.name)
                            return False
                        if time.time() > deadline:
                            self.api.stop(self.name)
                            break
                        self.sleep_for(0.05)

                    if not self.api.navigation_succeeded(self.name):
                        continue

                    self.sleep_for(search_settle_sec)
                    if _scan_dock_visibility_in_place():
                        self.node.get_logger().info(
                            f"Dock re-acquired after progressive approach "
                            f"({moved_total:.2f} m) for robot [{self.name}]"
                        )
                        return True

                return False

            def _attempt_dock_visibility_recovery():
                # まず現在位置で再探索 → pre_dock整列で再探索 →
                # pre_dock小探索（安全）→ 必要なら遠距離探索（任意）
                if _scan_dock_visibility_in_place():
                    return True
                if _recover_to_pre_dock() and _scan_dock_visibility():
                    return True
                if _micro_search_from_pre_dock():
                    return True
                if enable_far_progressive_search and _progressive_approach_and_scan():
                    return True
                return False

            if initial_visibility_failed:
                if not _attempt_dock_visibility_recovery():
                    _wait_at_pre_dock_and_finish(
                        f"Dock is not visible for robot [{self.name}] after "
                        "pre_dock micro-search; waiting at pre_dock."
                    )
                    return

            max_recovery_attempts = int(self.config.get("dock_recovery_retries", 6))
            recovery_attempt = 0

            while True:
                if not self.api.start_process(self.name, self.dock_name, self.map_name):
                    if recovery_attempt < max_recovery_attempts:
                        recovery_attempt += 1
                        self.node.get_logger().warn(
                            f"Failed to start native docking process [{self.dock_name}]. "
                            f"Retrying dock search ({recovery_attempt}/{max_recovery_attempts})."
                        )
                        self.api.stop(self.name)
                        if _attempt_dock_visibility_recovery():
                            continue
                    _wait_at_pre_dock_and_finish(
                        f"Failed to start native docking process [{self.dock_name}]. "
                        f"Waiting at pre_dock for robot [{self.name}]"
                    )
                    return

                with self._lock:
                    self.on_waypoint = None
                    self.on_lane = None
                self.sleep_for(0.1)

                dock_start_time = time.time()
                lost_dock_visible_since = None
                max_dock_duration_sec = float(
                    self.config.get("dock_max_duration_sec", 40.0)
                )
                max_invisible_sec = dock_lost_visibility_timeout_sec
                max_pre_dock_drift_m = float(
                    self.config.get("dock_abort_pre_dock_drift_m", 0.45)
                )
                max_dock_distance_m = float(
                    self.config.get("dock_abort_max_distance_m", 0.85)
                )
                max_distance_rebound_m = float(
                    self.config.get("dock_abort_rebound_m", 0.10)
                )
                min_distance_to_dock = float("inf")
                abort_reason = None
                docked_by_status = False

                while not self.api.docking_completed(self.name):
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info("Aborting docking")
                        self.api.stop(self.name)
                        return

                    now = time.time()
                    if (now - dock_start_time) > max_dock_duration_sec:
                        abort_reason = (
                            f"Docking timed out for robot [{self.name}]"
                        )
                        self.api.stop(self.name)
                        break

                    dock_visible = self.api.dock_visible(self.name)
                    is_docked = self.api.is_docked(self.name)
                    if dock_visible is False:
                        if lost_dock_visible_since is None:
                            lost_dock_visible_since = now
                    else:
                        lost_dock_visible_since = None

                    with self._lock:
                        current_position = copy.copy(self.position)
                    distance_to_dock = math.hypot(
                        current_position[0] - dock_x,
                        current_position[1] - dock_y
                    )
                    min_distance_to_dock = min(min_distance_to_dock, distance_to_dock)
                    if self.api.docked_confident(self.name) or (
                        self.api.is_docked(self.name) is True and distance_to_dock < 0.18
                    ):
                        docked_by_status = True
                        self.node.get_logger().info(
                            f"Robot [{self.name}] reports docked state; "
                            "finishing docking without retry."
                        )
                        break

                    if (
                        distance_to_dock > max_dock_distance_m
                        and dock_visible is False
                        and is_docked is not True
                    ):
                        abort_reason = (
                            f"Robot [{self.name}] is {distance_to_dock:.2f} m from dock "
                            "while dock is not visible"
                        )
                        self.api.stop(self.name)
                        break

                    if (
                        (distance_to_dock - min_distance_to_dock) > max_distance_rebound_m
                        and dock_visible is False
                        and is_docked is not True
                    ):
                        abort_reason = (
                            f"Robot [{self.name}] moved away from dock by "
                            f"{(distance_to_dock - min_distance_to_dock):.2f} m "
                            "after approach"
                        )
                        self.api.stop(self.name)
                        break

                    if (
                        lost_dock_visible_since is not None
                        and (now - lost_dock_visible_since) > max_invisible_sec
                        and is_docked is not True
                    ):
                        abort_reason = (
                            f"Lost dock visibility for robot [{self.name}] during docking"
                        )
                        self.api.stop(self.name)
                        break

                    if pre_x is not None and pre_y is not None:
                        drift = math.hypot(
                            current_position[0] - pre_x,
                            current_position[1] - pre_y
                        )
                        if (
                            drift > max_pre_dock_drift_m
                            and dock_visible is False
                            and is_docked is not True
                        ):
                            abort_reason = (
                                f"Robot [{self.name}] drifted {drift:.2f} m away "
                                "from pre_dock after losing dock"
                            )
                            self.api.stop(self.name)
                            break

                    self.node.get_logger().info("Robot is docking...")
                    self.sleep_for(0.05)

                if abort_reason is not None:
                    if recovery_attempt < max_recovery_attempts:
                        recovery_attempt += 1
                        self.node.get_logger().warn(
                            f"{abort_reason}; retrying dock search "
                            f"({recovery_attempt}/{max_recovery_attempts})."
                        )
                        if _attempt_dock_visibility_recovery():
                            continue
                    _wait_at_pre_dock_and_finish(
                        f"{abort_reason}; waiting at pre_dock."
                    )
                    return

                if (not docked_by_status) and (not self.api.process_completed(self.name)):
                    if recovery_attempt < max_recovery_attempts:
                        recovery_attempt += 1
                        self.node.get_logger().warn(
                            f"Native docking did not succeed for robot [{self.name}]; "
                            f"retrying dock search ({recovery_attempt}/{max_recovery_attempts})."
                        )
                        if _attempt_dock_visibility_recovery():
                            continue
                    _wait_at_pre_dock_and_finish(
                        f"Native docking did not succeed for robot [{self.name}]. "
                        "Waiting at pre_dock."
                    )
                    return

                break

            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                _invoke_docking_finished_callback()
                self.node.get_logger().info("Docking completed")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position(self.name)
        if position is not None:
            x, y = self.transforms['robot_to_rmf'].transform(
                [position[0], position[1]])
            theta = math.radians(position[2]) - \
                self.transforms['orientation_offset']
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # Ensure x, y are in meters and theta in radians #
            # ------------------------ #
            # Wrap theta between [-pi, pi]. Else arrival estimate will
            # assume robot has to do full rotations and delay the schedule
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            self._warned_missing_battery = False
            return battery_soc
        else:
            # 実機側の battery topic が未到達でも、直前値で追従は継続させる。
            if not self._warned_missing_battery:
                self.node.get_logger().warn(
                    "Unable to retrieve battery data from robot; "
                    "continuing with the last known battery state.")
                self._warned_missing_battery = True
            return self.battery_soc

    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        if self.update_handle is not None:
            self.update_state()

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        if not self.charger_is_set:
            if ("max_delay" in self.config.keys()):
                max_delay = self.config["max_delay"]
                self.node.get_logger().info(
                    f"Setting max delay to {max_delay}s")
                self.update_handle.set_maximum_delay(max_delay)
            if (self.charger_waypoint_index < self.graph.num_waypoints):
                self.update_handle.set_charger_waypoint(
                    self.charger_waypoint_index)
            else:
                self.node.get_logger().warn(
                    "Invalid waypoint supplied for charger. "
                    "Using default nearest charger in the map")
            self.charger_is_set = True
        # Update position
        with self._lock:
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, self.position)

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_remaining_waypoints(self, waypoints: list):
        '''
        The function returns a list where each element is a tuple of the index
        of the waypoint and the waypoint present in waypoints. This function
        may be modified if waypoints in a path need to be filtered.
        '''
        assert(len(waypoints) > 0)
        remaining_waypoints = []

        for i in range(len(waypoints)):
            remaining_waypoints.append((i, waypoints[i]))
        return remaining_waypoints
