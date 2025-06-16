#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt  # For path visualization

# --------------------------- CONFIG VEHICUL ---------------------------
vehicle_wheelbase = 0.26   # ampatament [m]

# --------------------------- FUNCȚII UTILE ---------------------------
def calculate_curvature(p1, p2, p3):
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

def adaptive_speed(curvature, min_speed, max_speed, min_curvature, max_curvature):
    curvature = np.clip(curvature, min_curvature, max_curvature)
    factor = 1 - (curvature - min_curvature) / (max_curvature - min_curvature)
    return min_speed + factor * (max_speed - min_speed)

def find_lookahead_point(position, path, lookahead_distance):
    cumulative_distance = 0.0
    closest_idx = int(np.argmin([np.linalg.norm(np.array(pt) - position) for pt in path]))
    for j in range(closest_idx, len(path) - 1):
        segment = np.linalg.norm(np.array(path[j+1]) - np.array(path[j]))
        cumulative_distance += segment
        if cumulative_distance >= lookahead_distance:
            return path[j+1]
    return path[-1]

def calculate_steering_angle(position, heading, target_point, lookahead_distance, wheelbase):
    dx = target_point[0] - position[0]
    dy = target_point[1] - position[1]
    local_x = np.cos(-heading) * dx - np.sin(-heading) * dy
    local_y = np.sin(-heading) * dx + np.cos(-heading) * dy
    if lookahead_distance == 0:
        return 0.0
    return np.arctan2(2 * wheelbase * local_y, lookahead_distance**2)

def pure_pursuit_adaptive_control(position, heading, path_points,
                                  min_speed, max_speed,
                                  min_curvature, max_curvature,
                                  base_lookahead, lookahead_gain):
    dists = [np.linalg.norm(np.array(p) - position) for p in path_points]
    idx = min(int(np.argmin(dists)), len(path_points) - 3)
    p1, p2, p3 = path_points[idx], path_points[idx+1], path_points[idx+2]
    curvature = calculate_curvature(p1, p2, p3)
    speed = adaptive_speed(curvature, min_speed, max_speed, min_curvature, max_curvature)
    lookahead_distance = base_lookahead + lookahead_gain * speed
    target_point = find_lookahead_point(position, path_points, lookahead_distance)
    steering = calculate_steering_angle(position, heading, target_point, lookahead_distance, vehicle_wheelbase)
    return {
        'target_point':       target_point,
        'curvature':          curvature,
        'speed':              speed,
        'steering_angle_rad': steering,
        'steering_angle_deg': np.degrees(steering),
        'lookahead_distance': lookahead_distance
    }

# --------------------------- ÎNCĂRCARE TRASÉU ---------------------------
def load_graphml_path_with_waypoints(graphml_file, start_node, end_node, waypoints=[]):
    G = nx.read_graphml(graphml_file)
    node_seq = [start_node] + waypoints + [end_node]
    full_path = []
    for i in range(len(node_seq) - 1):
        seg = nx.shortest_path(G, source=node_seq[i], target=node_seq[i+1])
        if i > 0:
            seg = seg[1:]
        full_path.extend(seg)
    return [(float(G.nodes[n]['x']), float(G.nodes[n]['y'])) for n in full_path]

# --------------------------- MAIN ROS NODE ---------------------------
def main():
    rospy.init_node("pure_pursuit_node")

    # Publishers
    speed_pub = rospy.Publisher("/speed", Float32, queue_size=10)
    angle_pub = rospy.Publisher("/direction_lane", Float32, queue_size=10)
    stop_nav_pub = rospy.Publisher('/stop_lanekeeping_cmd', Int32, queue_size=10)
    stop_nav_pub.publish(Int32(data=1))

    # === Config path ===
    graphml_file = "/home/jetson/Desktop/NDRTS/NDRTS-BFMC2024/src/waypoints/scripts/traseu.graphml"
    start_node = "112"
    end_node = "120"
    # waypoints = ["47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57"]  # example: ["5", "9"]
    waypoints = []  # example: ["5", "9"]

    path = load_graphml_path_with_waypoints(graphml_file, start_node, end_node, waypoints)

    # === Pure Pursuit parameters ===
    min_speed = 0.3     # m/s
    max_speed = 0.5     # m/s
    min_curv = 0.0
    max_curv = 2.0
    base_look = 0.1
    look_gain = 0.5
    dt = 0.1            # seconds

    position = np.array(path[0])
    heading = 0.0
    rate = rospy.Rate(1.0 / dt)

    while not rospy.is_shutdown():
        ctrl = pure_pursuit_adaptive_control(position, heading, path,
                                             min_speed, max_speed,
                                             min_curv, max_curv,
                                             base_look, look_gain)

        final_point = np.array(path[-1])
        distance_to_goal = np.linalg.norm(final_point - position)

        rospy.loginfo(f"[PP] Distance to goal: {distance_to_goal:.3f} m")

        if distance_to_goal < 0.2:
            rospy.loginfo("✅ Destination reached. Stopping vehicle.")
            speed_pub.publish(Float32(data=0.0))
            angle_pub.publish(Float32(data=0.0))
            break

        speed_mps = ctrl["speed"]
        steer_deg = ctrl["steering_angle_deg"]
        steer_deg = np.clip(steer_deg, -25, 25)

        speed_cms = int(speed_mps * 100)
        speed_cms = int(np.clip(speed_cms, -50, 50))

        speed_pub.publish(Float32(data=speed_cms))
        angle_pub.publish(Float32(data=int(steer_deg)))

        # Update position (simulate motion)
        heading += (speed_mps / vehicle_wheelbase) * np.tan(np.radians(steer_deg)) * dt
        position += np.array([np.cos(heading), np.sin(heading)]) * speed_mps * dt

        rospy.loginfo(f"[SIM] Position: {position}, Heading: {heading:.2f} rad")

        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
