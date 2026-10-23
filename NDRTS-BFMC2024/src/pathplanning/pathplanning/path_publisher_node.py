#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import networkx as nx
import pathlib
from ament_index_python.packages import get_package_share_directory
import os


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/waypoints', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)

        graphml_name = self.declare_parameter(
            'graphml_name', 'traseu.graphml').get_parameter_value().string_value
        graphml_path = os.path.join(
            get_package_share_directory('pathplanning'),
            'maps',  # or 'config' if you moved it
            graphml_name
        )

        self.points, self.edges = self.load_graph(graphml_path)
        self.get_logger().info(
            f"Loaded {len(self.points)} points and {len(self.edges)} edges from {graphml_path}")

    def load_graph(self, graphml_file):
        G = nx.read_graphml(graphml_file, node_type=str)
        points = [(float(d["x"]), -float(d["y"]))
                  for _, d in G.nodes(data=True)]
        id2idx = {n: i for i, (n, _) in enumerate(G.nodes(data=True))}
        edges = [(id2idx[u], id2idx[v]) for u, v in G.edges()]
        return points, edges

    def publish_path(self):
        path_msg = Path()
        header_prefix = "map|" + ";".join(f"{a},{b}" for a, b in self.edges)
        path_msg.header = Header(frame_id=header_prefix)
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
