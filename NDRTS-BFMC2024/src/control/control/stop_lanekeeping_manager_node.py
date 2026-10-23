#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class StopLaneKeepingManager(Node):
    def __init__(self):
        super().__init__('stop_lanekeeping_manager_node')

        # Publisher to forward the stop command
        self.publisher_ = self.create_publisher(Int32, '/stop_lanekeeping', 10)

        # Subscriber to listen for the command
        self.subscription = self.create_subscription(
            Int32,
            '/stop_lanekeeping_cmd',
            self.callback,
            10
        )

        self.get_logger().info("✅ stop_lanekeeping_manager_node is up and running.")

    def callback(self, msg):
        self.get_logger().info(f"🔁 Forwarding stop command: {msg.data}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopLaneKeepingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
