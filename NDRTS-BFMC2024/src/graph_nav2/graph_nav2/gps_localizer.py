#!/usr/bin/env python3
"""
Converteşte NavSatFix (lat/lon) → metri în cadrul 'map' cu ajutorul UTM.
"""
import rclpy, pyproj
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class GPSLocalizer(Node):
    def __init__(self):
        super().__init__('gps_localizer')
        self.utm_zone = self.declare_parameter('utm_zone', 33).value
        self.proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')

        self.create_subscription(NavSatFix, '/gps_data', self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/gps/pose', 10)

    def cb(self, msg):
        x, y = self.proj(msg.longitude, msg.latitude)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GPSLocalizer())
    rclpy.shutdown()
