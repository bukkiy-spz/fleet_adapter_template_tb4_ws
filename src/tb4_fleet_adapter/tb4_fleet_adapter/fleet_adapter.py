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

import sys
import argparse
import yaml
import time
import threading
import math
from datetime import timedelta

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI
try:
    import nudged
except ModuleNotFoundError:
    from . import nudged_compat as nudged

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time, server_uri):
    def _wrap_to_pi(theta):
        while theta > math.pi:
            theta -= 2.0 * math.pi
        while theta < -math.pi:
            theta += 2.0 * math.pi
        return theta

    def _robot_pose_to_rmf(robot_pose, transforms):
        x, y = transforms['robot_to_rmf'].transform([robot_pose[0], robot_pose[1]])
        theta = math.radians(robot_pose[2]) - transforms['orientation_offset']
        return [x, y, _wrap_to_pi(theta)]

    def _nearest_graph_waypoint(nav_graph, rmf_position):
        nearest_waypoint = None
        nearest_distance = float("inf")
        for waypoint_index in range(nav_graph.num_waypoints):
            waypoint = nav_graph.get_waypoint(waypoint_index)
            distance = math.hypot(
                rmf_position[0] - waypoint.location[0],
                rmf_position[1] - waypoint.location[1],
            )
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_waypoint = waypoint
        return nearest_waypoint, nearest_distance

    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(geometry.make_final_convex_circle(
        fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile)
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(
        voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = fleet_config['name']
    adapter_name = f'{fleet_name}_fleet_adapter'
    adapter_discovery_timeout_sec = float(
        fleet_config.get('adapter_discovery_timeout_sec', 120.0)
    )
    adapter_init_retry_interval_sec = float(
        fleet_config.get('adapter_init_retry_interval_sec', 3.0)
    )
    adapter_init_max_retries = int(
        fleet_config.get('adapter_init_max_retries', 0)
    )  # 0 means infinite retries

    adapter = None
    attempt = 0
    while adapter is None:
        attempt += 1
        wait_time = None
        if adapter_discovery_timeout_sec > 0.0:
            wait_time = timedelta(seconds=adapter_discovery_timeout_sec)
        adapter = adpt.Adapter.make(adapter_name, wait_time=wait_time)
        if adapter is None:
            node.get_logger().warn(
                "Unable to initialize fleet adapter yet "
                f"(attempt {attempt}); waiting for RMF schedule node "
                "discovery..."
            )
            if adapter_init_max_retries > 0 and attempt >= adapter_init_max_retries:
                raise RuntimeError(
                    "Unable to initialize fleet adapter after "
                    f"{attempt} attempts. Please ensure RMF Schedule Node "
                    "is running and DDS discovery is healthy."
                )
            time.sleep(adapter_init_retry_interval_sec)

    if use_sim_time:
        adapter.node.use_sim_time()
    node.get_logger().info("Starting RMF adapter core")
    adapter.start()
    time.sleep(1.0)

    node.get_logger().info(f"Adding fleet handle for [{fleet_name}]")
    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph, server_uri)

    # `publish_fleet_state` は bool/秒数の両方を許容して扱う。
    publish_fleet_state = fleet_config.get('publish_fleet_state', True)
    fleet_state_publish_period = None
    if isinstance(publish_fleet_state, bool):
        if publish_fleet_state:
            fleet_state_publish_period = 1.0
    else:
        try:
            publish_period_sec = float(publish_fleet_state)
            if publish_period_sec > 0.0:
                fleet_state_publish_period = publish_period_sec
        except (TypeError, ValueError):
            node.get_logger().warn(
                "Invalid publish_fleet_state value [%s]; disabling fleet "
                "state topic publishing" % publish_fleet_state
            )

    # 一部の RMF build では、robot 登録前に fleet state topic publish が走ると
    # C++ 側で落ちることがあるため、最初は明示的に止めておく。
    fleet_handle.fleet_state_topic_publish_period(None)
    if fleet_state_publish_period is None:
        node.get_logger().info("Fleet state topic publishing is disabled")
    else:
        node.get_logger().info(
            "Fleet state topic publishing will be enabled after the first "
            "robot is registered at %.2f s" % fleet_state_publish_period
        )
    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = fleet_config['task_capabilities']['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, ("Unable to set task planner params")

    task_capabilities = []
    if fleet_config['task_capabilities']['loop']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Loop tasks")
        task_capabilities.append(TaskType.TYPE_LOOP)
    if fleet_config['task_capabilities']['delivery']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Delivery tasks")
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if fleet_config['task_capabilities']['clean']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Clean tasks")
        task_capabilities.append(TaskType.TYPE_CLEAN)

    # Callable for validating requests that this fleet can accommodate
    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    # Transforms
    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
    robot_coordinates = config_yaml['reference_coordinates']['robot']
    transforms = {
        'rmf_to_robot': nudged.estimate(rmf_coordinates, robot_coordinates),
        'robot_to_rmf': nudged.estimate(robot_coordinates, rmf_coordinates)}
    transforms['orientation_offset'] = \
        transforms['rmf_to_robot'].get_rotation()
    mse = nudged.estimate_error(transforms['rmf_to_robot'],
                                rmf_coordinates,
                                robot_coordinates)
    print(f"Coordinate transformation error: {mse}")
    print("RMF to Robot transform:")
    print(f"    rotation:{transforms['rmf_to_robot'].get_rotation()}")
    print(f"    scale:{transforms['rmf_to_robot'].get_scale()}")
    print(f"    trans:{transforms['rmf_to_robot'].get_translation()}")
    print("Robot to RMF transform:")
    print(f"    rotation:{transforms['robot_to_rmf'].get_rotation()}")
    print(f"    scale:{transforms['robot_to_rmf'].get_scale()}")
    print(f"    trans:{transforms['robot_to_rmf'].get_translation()}")

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

    # Initialize robot API for this fleet
    node.get_logger().info(
        "Creating RobotAPI for namespace [%s]" %
        fleet_config['fleet_manager']['prefix']
    )
    api = RobotAPI(
        fleet_config['fleet_manager']['prefix'],
        fleet_config['fleet_manager']['user'],
        fleet_config['fleet_manager']['password'],
        node,
        fleet_config['fleet_manager'].get('api', {}))
    node.get_logger().info(
        f"RobotAPI connected={api.connected}; beginning robot registration thread"
    )

    # Initialize robots for this fleet

    missing_robots = config_yaml['robots']
    fleet_state_publish_enabled = False

    def _add_fleet_robots():
        nonlocal fleet_state_publish_enabled
        robots = {}
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                node.get_logger().info(f"Checking live pose for robot: {robot_name}")
                position = api.position(robot_name)
                if position is None:
                    node.get_logger().warn(
                        f"No pose received yet for robot [{robot_name}]; retrying"
                    )
                    continue
                if len(position) > 2:
                    rmf_position = _robot_pose_to_rmf(position, transforms)
                    node.get_logger().info(
                        f"Live robot pose [{robot_name}] "
                        f"robot_frame=({position[0]:.3f}, {position[1]:.3f}, "
                        f"{position[2]:.2f} deg) "
                        f"rmf_frame=({rmf_position[0]:.3f}, {rmf_position[1]:.3f}, "
                        f"{rmf_position[2]:.2f} rad)"
                    )
                    node.get_logger().info(f"Initializing robot: {robot_name}")
                    robots_config = config_yaml['robots'][robot_name]
                    rmf_config = robots_config['rmf_config']
                    robot_config = robots_config['robot_config']
                    initial_waypoint = rmf_config['start']['waypoint']
                    initial_orientation = rmf_config['start']['orientation']

                    starts = []
                    time_now = adapter.now()

                    use_configured_start = (
                        initial_waypoint is not None and initial_orientation is not None
                    )

                    if use_configured_start:
                        waypoint = nav_graph.find_waypoint(initial_waypoint)
                        if waypoint is None:
                            node.get_logger().warn(
                                f"Configured initial waypoint [{initial_waypoint}] "
                                f"does not exist for robot [{robot_name}]. "
                                "Falling back to the live pose."
                            )
                            use_configured_start = False
                        else:
                            distance_to_waypoint = math.hypot(
                                rmf_position[0] - waypoint.location[0],
                                rmf_position[1] - waypoint.location[1],
                            )
                            if distance_to_waypoint > 1.0:
                                node.get_logger().warn(
                                    f"Robot [{robot_name}] is {distance_to_waypoint:.2f} m "
                                    f"away from configured start waypoint "
                                    f"[{initial_waypoint}]. Using the live pose instead."
                                )
                                use_configured_start = False

                    if use_configured_start:
                        node.get_logger().info(
                            f"Using provided initial waypoint "
                            f"[{initial_waypoint}] "
                            f"and orientation [{initial_orientation:.2f}] to "
                            f"initialize starts for robot [{robot_name}]")
                        initial_waypoint_index = nav_graph.find_waypoint(
                            initial_waypoint).index
                        starts = [plan.Start(time_now,
                                             initial_waypoint_index,
                                             initial_orientation)]
                    else:
                        max_merge_waypoint_distance = float(
                            rmf_config["start"].get("max_merge_waypoint_distance", 0.1)
                        )
                        max_merge_lane_distance = float(
                            rmf_config["start"].get("max_merge_lane_distance", 1.0)
                        )
                        node.get_logger().info(
                            f"Running compute_plan_starts for robot: {robot_name} "
                            f"(waypoint_merge={max_merge_waypoint_distance:.2f} m, "
                            f"lane_merge={max_merge_lane_distance:.2f} m)"
                        )
                        starts = plan.compute_plan_starts(
                            nav_graph,
                            rmf_config['start']['map_name'],
                            rmf_position,
                            time_now,
                            max_merge_waypoint_distance=max_merge_waypoint_distance,
                            max_merge_lane_distance=max_merge_lane_distance,
                        )

                        if starts is None or len(starts) == 0:
                            nearest_waypoint, nearest_distance = _nearest_graph_waypoint(
                                nav_graph, rmf_position
                            )
                            if nearest_waypoint is not None:
                                node.get_logger().warn(
                                    f"Unable to merge live pose for robot [{robot_name}] "
                                    f"into the nav graph. Falling back to nearest "
                                    f"waypoint [{nearest_waypoint.waypoint_name}] "
                                    f"({nearest_distance:.2f} m away)."
                                )
                                starts = [
                                    plan.Start(
                                        time_now,
                                        nearest_waypoint.index,
                                        rmf_position[2],
                                    )
                                ]

                    if starts is None or len(starts) == 0:
                        node.get_logger().error(
                            f"Unable to determine StartSet for {robot_name}")
                        continue

                    robot = RobotCommandHandle(
                        name=robot_name,
                        fleet_name=fleet_name,
                        config=robot_config,
                        node=node,
                        graph=nav_graph,
                        vehicle_traits=vehicle_traits,
                        transforms=transforms,
                        map_name=rmf_config['start']['map_name'],
                        start=starts[0],
                        position=rmf_position,
                        charger_waypoint=rmf_config['charger']['waypoint'],
                        update_frequency=rmf_config.get(
                            'robot_state_update_frequency', 1),
                        adapter=adapter,
                        api=api)

                    if robot.initialized:
                        robots[robot_name] = robot
                        # Add robot to fleet
                        fleet_handle.add_robot(robot,
                                               robot_name,
                                               profile,
                                               [starts[0]],
                                               partial(_updater_inserter,
                                                       robot))
                        node.get_logger().info(
                            f"Successfully added new robot: {robot_name}")
                        # 最初の robot 登録が終わってから fleet state publish を
                        # 有効化すると、起動直後の segfault を避けやすい。
                        if (
                            not fleet_state_publish_enabled
                            and fleet_state_publish_period is not None
                        ):
                            fleet_handle.fleet_state_topic_publish_period(
                                timedelta(seconds=fleet_state_publish_period)
                            )
                            fleet_state_publish_enabled = True
                            node.get_logger().info(
                                "Enabled fleet state topic publishing at "
                                f"{fleet_state_publish_period:.2f} s"
                            )

                    else:
                        node.get_logger().error(
                            f"Failed to initialize robot: {robot_name}")

                    del missing_robots[robot_name]

                else:
                    pass
                    node.get_logger().debug(
                        f"{robot_name} not found, trying again...")
        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()
    return adapter


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                    help="URI of the api server to transmit state and task information.")
    parser.add_argument("--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    if args.server_uri == "":
        server_uri = None
    else:
        server_uri = args.server_uri

    adapter = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time,
        server_uri)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
