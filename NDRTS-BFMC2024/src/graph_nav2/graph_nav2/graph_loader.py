#!/usr/bin/env python3
"""
Servește cereri de traseu (listă de PoseStamped) pentru o listă
de noduri waypoint_ids în graf.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from graph_nav2_interfaces.srv import Path       # ← custom srv
import networkx as nx


class GraphLoader(Node):
    def __init__(self):
        super().__init__("graph_loader")

        graph_file = self.declare_parameter(
            "graphml_file", "map.graphml").value
        self.G = nx.read_graphml(graph_file)
        self.get_logger().info(
            f"Loaded graph with {self.G.number_of_nodes()} nodes")

        self.srv = self.create_service(Path, "get_path", self.handle_path)

    def handle_path(self, req, resp):
        ids = req.waypoint_ids
        if len(ids) < 2:
            self.get_logger().error("Need at least 2 waypoint_ids")
            return resp

        first = True
        for i in range(len(ids) - 1):
            s, g = ids[i], ids[i+1]
            try:
                segment = nx.shortest_path(self.G, s, g, weight="weight")
            except nx.NetworkXNoPath:
                self.get_logger().error(f"No path {s}→{g}")
                return resp

            # append each node in segment
            for idx, node_id in enumerate(segment):
                # avoid duplicating the join node
                if not first and idx == 0:
                    continue
                first = False

                x = float(self.G.nodes[node_id]["x"])
                y = float(self.G.nodes[node_id]["y"])
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                resp.poses.append(ps)

        return resp


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GraphLoader())
    rclpy.shutdown()
