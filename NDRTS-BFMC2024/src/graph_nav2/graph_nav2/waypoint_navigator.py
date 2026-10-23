#!/usr/bin/env python3
"""
Trimite un şir de way-points către acțiunea Nav2 `follow_waypoints`,
bazat pe un vector de waypoint_ids.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from graph_nav2_interfaces.srv import Path


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator_fw")
        self._ac = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._cli = self.create_client(Path, "get_path")

    def send_path(self, waypoint_ids):
        self._cli.wait_for_service()
        req = Path.Request()
        req.waypoint_ids = waypoint_ids
        resp = self._cli.call(req)

        if not resp.poses:
            self.get_logger().error("Received empty path!")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = resp.poses

        self.get_logger().info("Waiting for follow_waypoints action server…")
        self._ac.wait_for_server()
        send_goal = self._ac.send_goal_async(goal,
                                             feedback_callback=self._fb)
        send_goal.add_done_callback(self._result_cb)

    def _fb(self, fb_msg):
        self.get_logger().info(
            f"WP current index: {fb_msg.feedback.current_waypoint}")

    def _result_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED")
            return
        self.get_logger().info("Goal ACCEPTED")
        goal_handle.get_result_async().add_done_callback(self._finished)

    def _finished(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(
                f"Missed {len(result.missed_waypoints)} waypoints")
        else:
            self.get_logger().info("✔ Path completed.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    # ← here you can list as many intermediate IDs as you like:
    node.send_path(["1", "2", "6", "8", "10"])
    rclpy.spin(node)


if __name__ == "__main__":
    main()
