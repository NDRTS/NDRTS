# graph_nav2/traffic_communication_node.py
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Port of BFMC “processTrafficCommunication” to ROS 2.
# – Keeps the Thread-based TCP/UDP client exactly as it was.
# – Replaces WorkerProcess with an rclpy Node and a background thread.
# – Publishes inbound server messages on a ROS 2 topic
#   and subscribes to vehicle state topics for outbound messages.

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped

from threading import Thread, Event
from queue import Queue
import time
import os

# ------- BFMC helpers (now inside this package) ----------------------------
from .TrafficCommunication.useful.sharedMem import sharedMem
from .TrafficCommunication.threads.threadTrafficCommunication import (
    threadTrafficCommunication,
)

# relative import so it works regardless of package name
from .TrafficCommunication.utils.messages.allMessages import Location
from .TrafficCommunication.utils.messages.messageHandlerSender import (
    messageHandlerSender,
)


class TrafficCommNode(Node):
    """
    rclpy wrapper around BFMC threadTrafficCommunication.
    """

    def __init__(self):
        super().__init__('traffic_comm')

        # ───────── Parameters (declare + get) ─────────
        self.declare_parameter('device_id',   3)
        self.declare_parameter('frequency',   1.0)
        self.declare_parameter(
            'pem_path',
            os.path.join(
                os.path.dirname(__file__),
                'TrafficCommunication', 'useful', 'publickey_server_test.pem'
            )
        )

        self.device_id = int(self.get_parameter('device_id').value)
        self.frequency = float(self.get_parameter('frequency').value)
        self.pem_path = self.get_parameter('pem_path').value

        # ───────── Shared memory (same object the BFMC thread expects) ──────
        self._shmem = sharedMem()

        # ───────── ROS 2 I/O ────────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.sub_pose = self.create_subscription(
            PoseStamped,   'vehicle_pose',    self._pose_cb, qos)
        self.sub_vel = self.create_subscription(
            TwistStamped,  'vehicle_speed',   self._vel_cb,  qos)

        self.pub_inbound = self.create_publisher(
            String, 'traffic_server/inbound', qos)

        # ───────── Start BFMC thread ────────────────────────────────────────
        queue_map = {
            "Critical": Queue(),
            "Warning":  Queue(),
            "General":  Queue(),
            "Config":   Queue(),
        }

        self._bfmc_thread = threadTrafficCommunication(
            self._shmem,
            queue_map,
            self.device_id,
            self.frequency,
            self.pem_path
        )
        self._stop_evt = Event()
        self._bfmc_thread.start()

        # Forward every message that arrives on BFMC “General” queue
        self._queue_reader = Thread(target=self._drain_queue,
                                    args=(queue_map["General"],),
                                    daemon=True)
        self._queue_reader.start()

        self.get_logger().info(
            f"TrafficCommNode ready - device_id={self.device_id}, "
            f"freq={self.frequency} Hz"
        )

    # ═════════════════ ROS callbacks ════════════════════════════════════════
    def _pose_cb(self, msg: PoseStamped):
        self._shmem.insert("devicePos",
                           [msg.pose.position.x, msg.pose.position.y])
        # For completeness you can also write orientation / yaw here.

    def _vel_cb(self, msg: TwistStamped):
        self._shmem.insert("deviceSpeed",
                           [msg.twist.linear.x])

    # ═════════════════ queue-reader thread ══════════════════════════════════
    def _drain_queue(self, q: Queue):
        while not self._stop_evt.is_set():
            try:
                item = q.get(timeout=0.2)   # blocks briefly
            except Exception:
                continue
            self.pub_inbound.publish(String(data=str(item)))

    # ═════════════════ clean shutdown ══════════════════════════════════════
    def destroy_node(self):
        self._stop_evt.set()
        try:
            self._bfmc_thread.stop()
            self._bfmc_thread.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
