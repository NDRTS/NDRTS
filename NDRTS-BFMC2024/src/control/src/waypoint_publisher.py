#!/usr/bin/env python3
import rospy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseArray, Pose
import os

def load_graphml(file_path):
    waypoints = []
    tree = ET.parse(file_path)
    root = tree.getroot()
    ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}

    for node in root.findall(".//graphml:node", ns):
        x = float(node.find(".//graphml:data[@key='d0']", ns).text)
        y = float(node.find(".//graphml:data[@key='d1']", ns).text)
        waypoints.append((x, y))
    return waypoints

def main():
    rospy.init_node('waypoint_publisher', anonymous=True)
    pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    file_path = rospy.get_param("~graphml_path", "/home/jetson/ ")
    waypoints = load_graphml(file_path)

    msg = PoseArray()
    msg.header.frame_id = "map"

    for x, y in waypoints:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0
        msg.poses.append(pose)

    rospy.loginfo(f"📡 Loaded {len(msg.poses)} waypoints from GraphML file.")

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
