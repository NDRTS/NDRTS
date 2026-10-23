#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import networkx as nx
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Publishers
        self.speed_pub = self.create_publisher(Int32, '/speed', 10)
        self.angle_pub = self.create_publisher(Int32, '/direction_lane', 10)
        self.stop_nav_pub = self.create_publisher(
            Int32, '/stop_lanekeeping_cmd', 10)
        self.stop_nav_pub.publish(Int32(data=1))

        # Vehicle parameters
        self.vehicle_wheelbase = 0.26

        graphml_name = self.declare_parameter(
            'graphml_name', 'traseu.graphml').get_parameter_value().string_value
        self.graphml_file = os.path.join(
            get_package_share_directory('pathplanning'),
            'maps',  # or 'config' if you moved it there
            graphml_name
        )

        self.start_node = '112'
        self.end_node = '120'
        self.waypoints = []  # You can add intermediate nodes here
        self.path = self.load_graphml_path_with_waypoints()

        # Control parameters
        self.min_speed = 0.3
        self.max_speed = 0.5
        self.min_curv = 0.0
        self.max_curv = 2.0
        self.base_look = 0.1
        self.look_gain = 0.5
        self.dt = 0.1

        # Sim state
        self.position = np.array(self.path[0])
        self.heading = 0.0

        # Timer loop
        self.create_timer(self.dt, self.loop)

    def load_graphml_path_with_waypoints(self):
        G = nx.read_graphml(self.graphml_file)
        node_seq = [self.start_node] + self.waypoints + [self.end_node]
        full_path = []
        for i in range(len(node_seq) - 1):
            seg = nx.shortest_path(
                G, source=node_seq[i], target=node_seq[i + 1])
            if i > 0:
                seg = seg[1:]
            full_path.extend(seg)
        return [(float(G.nodes[n]['x']), float(G.nodes[n]['y'])) for n in full_path]

    def loop(self):
        path = self.path
        if len(path) < 3:
            self.get_logger().error("Not enough points in path.")
            return

        ctrl = self.pure_pursuit_adaptive_control(path)
        final_point = np.array(path[-1])
        distance_to_goal = np.linalg.norm(final_point - self.position)

        self.get_logger().info(
            f"[PP] Distance to goal: {distance_to_goal:.3f} m")

        if distance_to_goal < 0.2:
            self.get_logger().info("✅ Destination reached. Stopping vehicle.")
            self.speed_pub.publish(Int32(data=0.0))
            self.angle_pub.publish(Int32(data=0.0))
            rclpy.shutdown()
            return

        speed_mps = ctrl["speed"]
        steer_deg = np.clip(ctrl["steering_angle_deg"], -25, 25)
        speed_cms = int(np.clip(speed_mps * 100, -50, 50))

        self.speed_pub.publish(Int32(data=speed_cms))
        self.angle_pub.publish(Int32(data=int(steer_deg)))

        self.heading += (speed_mps / self.vehicle_wheelbase) * \
            np.tan(np.radians(steer_deg)) * self.dt
        self.position += np.array([np.cos(self.heading),
                                  np.sin(self.heading)]) * speed_mps * self.dt

        self.get_logger().info(
            f"[SIM] Position: {self.position}, Heading: {self.heading:.2f} rad")

    def calculate_curvature(self, p1, p2, p3):
        a = np.linalg.norm(np.array(p1) - np.array(p2))
        b = np.linalg.norm(np.array(p2) - np.array(p3))
        c = np.linalg.norm(np.array(p3) - np.array(p1))
        s = (a + b + c) / 2
        area = max(s * (s - a) * (s - b) * (s - c), 0)
        area = np.sqrt(area)
        if area == 0:
            return 0.0
        radius = (a * b * c) / (4 * area)
        return 1 / radius

    def adaptive_speed(self, curvature):
        curvature = np.clip(curvature, self.min_curv, self.max_curv)
        factor = 1 - (curvature - self.min_curv) / \
            (self.max_curv - self.min_curv)
        return self.min_speed + factor * (self.max_speed - self.min_speed)

    def find_lookahead_point(self, path, lookahead_distance):
        cumulative_distance = 0.0
        closest_idx = int(
            np.argmin([np.linalg.norm(np.array(pt) - self.position) for pt in path]))
        for j in range(closest_idx, len(path) - 1):
            segment = np.linalg.norm(np.array(path[j + 1]) - np.array(path[j]))
            cumulative_distance += segment
            if cumulative_distance >= lookahead_distance:
                return path[j + 1]
        return path[-1]

    def calculate_steering_angle(self, target_point, lookahead_distance):
        dx = target_point[0] - self.position[0]
        dy = target_point[1] - self.position[1]
        local_x = np.cos(-self.heading) * dx - np.sin(-self.heading) * dy
        local_y = np.sin(-self.heading) * dx + np.cos(-self.heading) * dy
        if lookahead_distance == 0:
            return 0.0
        return np.arctan2(2 * self.vehicle_wheelbase * local_y, lookahead_distance ** 2)

    def pure_pursuit_adaptive_control(self, path_points):
        dists = [np.linalg.norm(np.array(p) - self.position)
                 for p in path_points]
        idx = min(int(np.argmin(dists)), len(path_points) - 3)
        p1, p2, p3 = path_points[idx], path_points[idx +
                                                   1], path_points[idx + 2]
        curvature = self.calculate_curvature(p1, p2, p3)
        speed = self.adaptive_speed(curvature)
        lookahead_distance = self.base_look + self.look_gain * speed
        target_point = self.find_lookahead_point(
            path_points, lookahead_distance)
        steering = self.calculate_steering_angle(
            target_point, lookahead_distance)
        return {
            'target_point': target_point,
            'curvature': curvature,
            'speed': speed,
            'steering_angle_rad': steering,
            'steering_angle_deg': np.degrees(steering),
            'lookahead_distance': lookahead_distance
        }


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
