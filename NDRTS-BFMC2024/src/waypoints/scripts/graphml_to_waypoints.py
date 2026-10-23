#!/usr/bin/env python3
"""
Load a GraphML, keep the node order, flip Y once (ROS is +Y up, SVG +Y down),
and publish:

  * nav_msgs/Path  poses             ->  map points
  * header.frame_id = "map|a,b;c,d…" ->  edge list (index pairs)

The header trick avoids a custom message while letting the bridge recover
the exact edge sequence for drawing.
"""
import rospy, pathlib, networkx as nx
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# --------------------------------------------------------------------------- #
# helper                                                                      #
# --------------------------------------------------------------------------- #
def load_graph(graphml_file):
    G = nx.read_graphml(graphml_file, node_type=str)

    # points – flip Y once
    points = [(float(d["x"]), -float(d["y"])) for _, d in G.nodes(data=True)]

    # map node-id -> index
    id2idx = {n: i for i, (n, _) in enumerate(G.nodes(data=True))}

    # edges as index pairs, in file order
    edges = [(id2idx[u], id2idx[v]) for u, v in G.edges()]

    return points, edges

# --------------------------------------------------------------------------- #
def main():
    rospy.init_node("graphml_waypoints_publisher")
    pub = rospy.Publisher("/waypoints", Path, queue_size=1, latch=True)

    graphml = rospy.get_param("~graphml_file",
                              str(pathlib.Path(__file__).with_suffix(".graphml")))
    points, edges = load_graph(graphml)
    rospy.loginfo("Loaded %d points, %d edges from %s",
                  len(points), len(edges), graphml)
    if not points:
        rospy.logerr("No points – aborting")
        return

    # build a Path message
    path = Path()
    header_prefix = "map|" + ";".join(f"{a},{b}" for a, b in edges)
    path.header = Header(frame_id=header_prefix)

    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    rate = rospy.Rate(1)                      # 1 Hz
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        pub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    main()
