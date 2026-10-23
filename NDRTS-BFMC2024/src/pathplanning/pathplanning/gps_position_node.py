#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import serial
import time


class GPSPositionNode(Node):
    def __init__(self):
        super().__init__('gps_position_node')
        self.publisher = self.create_publisher(
            PointStamped, '/car_position', 5)

        self.port = self.declare_parameter(
            "port", "/dev/ttyACM1").get_parameter_value().string_value
        self.baud = self.declare_parameter(
            "baud", 115200).get_parameter_value().integer_value
        self.ser = self.open_serial()
        self.timer = self.create_timer(0.1, self.read_serial)

    def send_lec(self):
        self.ser.write(b"\r\r")
        time.sleep(0.1)
        self.ser.write(b"lec\r")
        time.sleep(0.1)

    def open_serial(self):
        while rclpy.ok():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1)
                self.get_logger().info(f"Serial open on {self.port}")
                self.send_lec()
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Cannot open {self.port} ({e}). Retrying in 2s")
                time.sleep(2)

    def parse_line(self, line: str):
        if "POS" not in line:
            return None
        parts = line.strip().split(",")
        try:
            i = parts.index("POS")
            x, y = float(parts[i + 1]), float(parts[i + 2])
            return x, -y
        except (ValueError, IndexError):
            return None

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors="ignore")
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial error: {e}. Reopening port")
            self.ser.close()
            self.ser = self.open_serial()
            return

        xy = self.parse_line(line)
        if xy is None:
            return

        x, y = xy
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = x
        msg.point.y = y
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
