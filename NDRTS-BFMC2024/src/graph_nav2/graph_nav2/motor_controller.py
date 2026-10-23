#!/usr/bin/env python3
"""
Control simplu viteză + direcție pe baza unghiului către waypoint-ul curent.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from graph_nav2_interfaces.srv import Path               # ← our custom service
from std_msgs.msg import Int16


def clamp(val, lo, hi):
    return int(max(lo, min(hi, val)))


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # parameters
        self.lookahead = self.declare_parameter("lookahead", 1.0).value
        self.k_steer = self.declare_parameter("k_steer", 1.0).value
        self.k_speed = self.declare_parameter("k_speed", 20.0).value

        self.pose = None
        self.path = []                      # list[PoseStamped]

        self.create_subscription(PoseStamped, "/gps/pose", self.pose_cb, 10)

        # subscribe to responses of GraphLoader
        self.create_service(Path, "get_path", self.store_path)  # optional

        self.pub_steer = self.create_publisher(Int16, "/direction_lane", 10)
        self.pub_speed = self.create_publisher(Int16, "/speed", 10)

        self.create_timer(0.05, self.loop)

    # ------------- callbacks -------------------------------------------------
    def pose_cb(self, msg):
        self.pose = msg

    def store_path(self, req, resp):
        # this node is **not** meant to answer; we only want the response
        return resp  # ignored

    # ------------- main control loop ----------------------------------------
    def loop(self):
        if not (self.pose and self.path):
            return

        tgt = self.path[0].pose.position
        dx = tgt.x - self.pose.pose.position.x
        dy = tgt.y - self.pose.pose.position.y
        dist = math.hypot(dx, dy)

        if dist < self.lookahead:
            self.path.pop(0)
            return

        heading_error = math.degrees(math.atan2(dy, dx))
        steer = clamp(self.k_steer * heading_error, -250, 250)
        speed = clamp(self.k_speed * math.cos(math.radians(heading_error)),
                      -500, 500)

        self.pub_steer.publish(Int16(data=steer))
        self.pub_speed.publish(Int16(data=speed))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotorController())
    rclpy.shutdown()
