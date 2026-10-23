#!/usr/bin/env python3

import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from control.engines.messageconverter import MessageConverter


class EngineSerialBridge(Node):
    def __init__(self):
        super().__init__('engine_serial_bridge')

        # Load parameters (with defaults)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter(
            "serial_port").get_parameter_value().string_value
        baud = self.get_parameter(
            "baudrate").get_parameter_value().integer_value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(port, baud)
            self.get_logger().info(
                f"✅ Connected to serial port: {port} @ {baud} bps")
        except serial.SerialException as e:
            self.get_logger().fatal(f"❌ Failed to open serial port: {e}")
            raise

        # Initialize message converter
        self.msg_converter = MessageConverter()

        kl_cmd = self.msg_converter.get_command("kl", mode=30)
        self._send_serial_command(kl_cmd)
        time.sleep(0.1)

        # Create ROS 2 subscriptions
        self.create_subscription(
            Int32, "/direction_lane", self.on_steering_command, 10)
        self.create_subscription(Int32, "/speed", self.on_speed_command, 10)

    def on_steering_command(self, msg: Int32):
        """Handle new steering command and send it via serial."""
        angle_deg = int(round(msg.data))                # 🔹 int, 3-digit
        cmd = self.msg_converter.get_command("steer",   # 🔹 new key
                                             steerAngle=angle_deg)
        self._send_serial_command(cmd)

    def on_speed_command(self, msg: Int32):
        """Handle new speed command and send it via serial."""
        speed_cms = int(round(msg.data))                # 🔹 int, 3-digit
        cmd = self.msg_converter.get_command("speed",   # 🔹 new key
                                             speed=speed_cms)
        self._send_serial_command(cmd)

    def _create_steering_command(self, angle: int) -> str:
        """Generate formatted steering command string."""
        return self.msg_converter.get_command("steer", steerAngle=int(angle))

    def _create_speed_command(self, speed: int) -> str:
        """Generate formatted speed command string."""
        return self.msg_converter.get_command("speed", speed=int(speed))

    def _send_serial_command(self, command: str):
        self.get_logger().info(f"→ Serial: {command.strip()}")
        try:
            self.serial_conn.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send serial command: {e}")


def main():
    rclpy.init()
    node = EngineSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
