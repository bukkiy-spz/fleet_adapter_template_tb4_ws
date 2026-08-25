#!/usr/bin/env python3

import math
import yaml

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from tb4_fleet_adapter import nudged_compat as nudged


class TransformedNavGraphVisualizer(Node):

    def __init__(self):
        super().__init__('transformed_navgraph_visualizer')

        self.declare_parameter(
            'config_file',
            '/home/ibuki/research/rmf_ws/fleet_adapter_template_tb4_ws/'
            'src/tb4_fleet_adapter/config_tb4_sim.yaml'
        )

        self.declare_parameter(
            'nav_graph_file',
            '/home/ibuki/research/rmf_ws/fleet_adapter_template_tb4_ws/'
            'src/tb4_rmf_demo/maps/rmf/tb4_20260612/nav_graphs/0.yaml'
        )

        self.declare_parameter('frame_id', 'map')

        config_file = self.get_parameter('config_file').value
        nav_graph_file = self.get_parameter('nav_graph_file').value
        self.frame_id = self.get_parameter('frame_id').value

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        with open(nav_graph_file, 'r') as f:
            graph = yaml.safe_load(f)

        rmf_coords = config['reference_coordinates']['rmf']
        robot_coords = config['reference_coordinates']['robot']

        self.transform = nudged.estimate(rmf_coords, robot_coords)

        self.get_logger().info(
            f'RMF -> Nav2 transform: '
            f'rotation={self.transform.get_rotation():.6f}, '
            f'scale={self.transform.get_scale():.6f}, '
            f'translation={self.transform.get_translation()}'
        )

        self.vertices = graph['levels']['L1']['vertices']
        self.lanes = graph['levels']['L1']['lanes']

        self.publisher = self.create_publisher(
            MarkerArray,
            '/tb4/transformed_nav_graph_markers',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_markers)

    def transform_point(self, x, y):
        tx, ty = self.transform.transform([x, y])
        return float(tx), float(ty)

    def publish_markers(self):
        markers = MarkerArray()

        stamp = self.get_clock().now().to_msg()

        # lanes
        lane_marker = Marker()
        lane_marker.header.frame_id = self.frame_id
        lane_marker.header.stamp = stamp
        lane_marker.ns = 'rmf_transformed_lanes'
        lane_marker.id = 0
        lane_marker.type = Marker.LINE_LIST
        lane_marker.action = Marker.ADD
        lane_marker.scale.x = 0.05

        lane_marker.color.r = 0.0
        lane_marker.color.g = 1.0
        lane_marker.color.b = 0.0
        lane_marker.color.a = 0.9

        for lane in self.lanes:
            start_idx = lane[0]
            end_idx = lane[1]

            start = self.vertices[start_idx]
            end = self.vertices[end_idx]

            sx, sy = self.transform_point(start[0], start[1])
            ex, ey = self.transform_point(end[0], end[1])

            p1 = Point()
            p1.x = sx
            p1.y = sy
            p1.z = 0.05

            p2 = Point()
            p2.x = ex
            p2.y = ey
            p2.z = 0.05

            lane_marker.points.append(p1)
            lane_marker.points.append(p2)

        markers.markers.append(lane_marker)

        marker_id = 1

        for idx, vertex in enumerate(self.vertices):
            x_rmf = vertex[0]
            y_rmf = vertex[1]

            properties = vertex[2] if len(vertex) > 2 else {}
            name = properties.get('name', '')

            x, y = self.transform_point(x_rmf, y_rmf)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'rmf_transformed_waypoints'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.10
            sphere.pose.orientation.w = 1.0

            if properties.get('is_charger', False):
                sphere.scale.x = 0.30
                sphere.scale.y = 0.30
                sphere.scale.z = 0.15
            else:
                sphere.scale.x = 0.20
                sphere.scale.y = 0.20
                sphere.scale.z = 0.10

            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 1.0

            markers.markers.append(sphere)

            if name:
                label = Marker()
                label.header.frame_id = self.frame_id
                label.header.stamp = stamp
                label.ns = 'rmf_transformed_labels'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD

                label.pose.position.x = x
                label.pose.position.y = y
                label.pose.position.z = 0.35
                label.pose.orientation.w = 1.0

                label.scale.z = 0.25

                label.color.r = 1.0
                label.color.g = 0.5
                label.color.b = 0.0
                label.color.a = 1.0

                label.text = name

                markers.markers.append(label)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)

    node = TransformedNavGraphVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
