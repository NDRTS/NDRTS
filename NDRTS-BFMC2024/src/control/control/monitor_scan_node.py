#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math
import numpy as np

# Control parameters
TARGET_LEFT = 50.0
TARGET_RIGHT = 40.0
MIN_VALID_CM = 20.0
MAX_VALID_CM = 90.0
K = 1.0
MAX_STEER_DEG = 17.0
MAX_STEER_STEP = 4.0


class LidarMonitor(Node):
    def __init__(self):
        super().__init__('monitor_scan_node')

        self.latest_scan = None
        self.latest_averages = {}
        self.last_correction = 0.0
        self.lane_keeping_stopped = None

        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.direction_pub = self.create_publisher(
            Int32, '/direction_lane', 10)
        self.stop_lk_pub = self.create_publisher(
            Int32, '/stop_lanekeeping_cmd', 10)
        self.front_distance_pub = self.create_publisher(
            Int32, '/distance_front', 10)

        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

        self.get_logger().info("✅ monitor_scan_node started.")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is not None:
            self.process_scan(self.latest_scan)

    def process_scan(self, scan):
        desired_angles_deg = [0, 90, 180, -90]
        window_rad = math.radians(10)
        num_readings = len(scan.ranges)
        angles = np.array(
            [scan.angle_min + i * scan.angle_increment for i in range(num_readings)])

        averages = {}
        for deg in desired_angles_deg:
            rad = math.radians(deg)
            indices = np.where((angles >= (rad - window_rad))
                               & (angles <= (rad + window_rad)))[0]
            valid_readings = [
                scan.ranges[i] for i in indices
                if not math.isinf(scan.ranges[i]) and not math.isnan(scan.ranges[i])
            ]
            if valid_readings:
                avg_cm = sum(valid_readings) / len(valid_readings) * 100.0
                averages[deg] = avg_cm
            else:
                averages[deg] = None

        self.latest_averages = averages.copy()

        if averages.get(180) is not None:
            self.front_distance_pub.publish(
                Int32(data=int(round(averages[180]))))

        if averages.get(90) is not None and averages.get(-90) is not None:
            right_cm = averages[90]
            left_cm = averages[-90]
            total = left_cm + right_cm
            print(
                f"Left: {left_cm:.2f} cm, Right: {right_cm:.2f} cm, Total: {total:.2f} cm")

            if total < 100:
                if self.lane_keeping_stopped is not True:
                    self.stop_lk_pub.publish(Int32(data=1))
                    self.lane_keeping_stopped = True
                self.compute_and_send_steering(left_cm, right_cm)
            else:
                if self.lane_keeping_stopped is not False:
                    self.stop_lk_pub.publish(Int32(data=0))
                    self.lane_keeping_stopped = False

    def compute_and_send_steering(self, left_cm, right_cm):
        error_left = left_cm - TARGET_LEFT
        error_right = right_cm - TARGET_RIGHT
        new_correction = K * (error_right - error_left)
        new_correction = max(
            min(new_correction, MAX_STEER_DEG), -MAX_STEER_DEG)

        delta = new_correction - self.last_correction
        if abs(delta) > MAX_STEER_STEP:
            correction = self.last_correction + \
                math.copysign(MAX_STEER_STEP, delta)
        else:
            correction = new_correction

        self.last_correction = correction
        correction = int(round(correction))
        # self.direction_pub.publish(Int32(data=int(correction)))
        if correction is not None:
            self.direction_pub.publish(Int32(data=int(correction)))
        else:
            self.get_logger().warn("🚫 correction is None – not publishing.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
