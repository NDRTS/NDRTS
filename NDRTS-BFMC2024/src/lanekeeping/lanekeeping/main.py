#!/usr/bin/env python3

import base64
import cv2
import logging
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge

from lanekeeping.src.LaneKeeping.lanekeeping import LaneKeeping
from lanekeeping.src.LaneDetection.detect import LaneDetection


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.bridge = CvBridge()
        self.camera = "455"
        self.real_world_example = True
        self.skipped_frames = 10
        self.frames_used = 0
        self.time_sum = 0
        self.stop_lanekeeping_value = 0

        self.direction_pub = self.create_publisher(
            Int32, "/direction_lane", 10)

        self.lk = LaneKeeping(
            640, 480,
            logging.getLogger('Root logger'),
            self.camera,
            direction_publisher=self.direction_pub
        )

        self.ld = LaneDetection(640, 480, self.camera, self.lk)

        if self.real_world_example:
            self.ld.square_pulses_min_height = 80
            self.ld.square_pulses_pix_dif = 10
            self.ld.square_pulses_min_height_dif = 20
            self.ld.square_pulses_allowed_peaks_width_error = 15

        self.publisher_img = self.create_publisher(
            Image, '/lane_detection_viz', 10)
        self.publisher_b64 = self.create_publisher(
            String, '/lane_detection_viz_base64', 10)

        self.subscription_img = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription_stop = self.create_subscription(
            Int32, '/stop_lanekeeping', self.stop_callback, 10)

        self.get_logger().info("🚘 LaneDetectionNode started")

    def stop_callback(self, msg: Int32):
        self.stop_lanekeeping_value = msg.data

    def image_callback(self, msg: Image):
        if self.stop_lanekeeping_value == 1:
            return

        src = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.real_world_example or self.frames_used % self.skipped_frames == 0:
            self.frames_used += 1
            start = time.time()

            results = self.ld.lanes_detection(src.copy())
            angle, annotated = self.lk.lane_keeping(results)

            elapsed = time.time() - start
            self.time_sum += elapsed

            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            self.publisher_img.publish(img_msg)

            # Publish base64 string
            _, jpeg = cv2.imencode('.jpg', annotated)
            base64_str = base64.b64encode(jpeg).decode("utf-8")
            self.publisher_b64.publish(String(data=base64_str))

    def print_average_time(self):
        if self.frames_used > 0:
            avg = self.time_sum / self.frames_used
            self.get_logger().info(f"🕒 Average processing time: {avg:.4f}s")


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_average_time()
    finally:
        node.destroy_node()
        rclpy.shutdown()
