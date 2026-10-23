#!/usr/bin/env python3

import time
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from control.signs import TrafficSignManeuvers
import re

LABEL_MAP = {
    'crosswalksign':  'Crosswalksign',
    'stopsign':       'Stopsign',
    'parkingsign':    'Parkingsign',
    'round-aboutsign': 'Round-aboutsign',
    'prioritysign':   'Prioritysign',
}

DEDUP_PERIOD = 0.30


def normalise(raw: str) -> str:
    token = re.sub(r'[^A-Za-z]', '', raw).lower()
    return LABEL_MAP.get(token, raw.strip())


class TrafficSignDetector:
    def __init__(self, maneuvers: TrafficSignManeuvers):
        self.votes = {}
        self.cooldown_until = {}
        self.threshold = 3
        self.time_window = 1.0
        self.remove_timeout = 10.0
        self.cooldown_secs = {
            "Stopsign": 20.0,
            "Parkingsign": 40.0,
            "Round-aboutsign": 15.0,
            "Crosswalksign": 15.0,
            "Prioritysign": 15.0
        }
        self.maneuvers = maneuvers

    def update_detection(self, label: str, now: float):
        if label in self.cooldown_until and now < self.cooldown_until[label]:
            return

        if label not in self.votes:
            self.votes[label] = {'count': 0, 'last': now}
        self.votes[label]['count'] += 1
        self.votes[label]['last'] = now

        if self.votes[label]['count'] >= self.threshold:
            self.maneuvers.get_logger().info(f"✅ Confirmed detection: {label}")
            self.maneuvers.handle_maneuver(label)
            self.cooldown_until[label] = now + \
                self.cooldown_secs.get(label, 5.0)
            self.votes.pop(label, None)

    def remove_old_detections(self, now: float):
        self.votes = {k: v for k, v in self.votes.items(
        ) if now - v['last'] <= self.remove_timeout}


class ControlNode(Node):
    def __init__(self):
        super().__init__('sign_publisher')
        self.dist_front = 9999.0
        self.line = 0
        self.last_label = None
        self.last_stamp = 0.0

        self.maneuvers = TrafficSignManeuvers(self)
        self.detector = TrafficSignDetector(self.maneuvers)

        self.create_subscription(
            String, '/detected_class', self.sign_callback, 10)
        self.create_subscription(
            Int32, '/distance_front', self.dist_front_callback, 10)
        self.create_subscription(Int32, '/line', self.line_callback, 10)

        self.timer = self.create_timer(0.1, self.cleanup_votes)

    def dist_front_callback(self, msg):
        self.dist_front = msg.data
        self.maneuvers.dist_front = msg.data

    def line_callback(self, msg):
        self.line = msg.data
        self.maneuvers.line = msg.data

    def sign_callback(self, msg):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        raw = msg.data
        label = normalise(raw)

        if label == self.last_label and (now - self.last_stamp) < DEDUP_PERIOD:
            return

        self.last_label, self.last_stamp = label, now
        self.get_logger().debug(f"🔤 Raw '{raw}' → Norm '{label}' (fwd)")
        self.detector.update_detection(label, now)

    def cleanup_votes(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        self.detector.remove_old_detections(now)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.maneuvers.send_resume_signal()
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
