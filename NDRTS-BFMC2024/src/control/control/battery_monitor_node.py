#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import serial
from control.engines.messageconverter import MessageConverter


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # === Serial Setup ===
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.mc = MessageConverter()

        # === ROS Publishers ===
        self.battery_pub = self.create_publisher(Float32, '/battery', 10)
        self.instant_pub = self.create_publisher(Int32, '/battery_instant', 10)
        self.power_pub = self.create_publisher(Float32, '/battery_power', 10)
        self.percent_pub = self.create_publisher(
            Float32, '/battery_percent', 10)

        # === Enable readings ===
        self.send_serial_command("battery", 1)
        self.send_serial_command("instant", 1)
        self.send_serial_command("batteryCapacity", 11700)

        # === State
        self.battery_volts = 0.0
        self.instant_current_mA = 0

        # === Timer to read data
        self.create_timer(0.1, self.read_serial)

    def send_serial_command(self, command: str, value: int):
        cmd = self.mc.get_command(command, activate=value)
        self.get_logger().info(f"→ Sending: {cmd.strip()}")
        self.ser.write(cmd.encode())

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()

            if line.startswith("@battery:"):
                value_str = line.split("@battery:")[1].split(";")[0]
                battery_mV = int(value_str)
                self.battery_volts = battery_mV / 1000.0

                self.get_logger().info(
                    f"Battery level: {self.battery_volts:.3f} V")
                self.battery_pub.publish(Float32(data=self.battery_volts))
                self.publish_battery_percent()

            elif line.startswith("@instant:"):
                value_str = line.split("@instant:")[1].split(";")[0]
                self.instant_current_mA = int(value_str)

                self.get_logger().info(
                    f"Instant consumption: {self.instant_current_mA} mA")
                self.instant_pub.publish(Int32(data=self.instant_current_mA))
                self.publish_power()

        except Exception as e:
            self.get_logger().warn(f"Failed to parse line: {line} → {e}")

    def publish_power(self):
        # P = U × I (with I in A)
        power = self.battery_volts * (self.instant_current_mA / 1000.0)
        self.get_logger().info(f"Power: {power:.2f} W")
        self.power_pub.publish(Float32(data=power))

    def publish_battery_percent(self):
        # Assume: 8.4V = 100%, 7.0V = 0% for 2S LiPo
        percent = (self.battery_volts - 7.0) / (8.4 - 7.0) * 100
        percent = max(0.0, min(percent, 100.0))
        self.get_logger().info(f"Battery percent: {percent:.1f}%")
        self.percent_pub.publish(Float32(data=percent))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔌 Battery monitor stopped by user.")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
