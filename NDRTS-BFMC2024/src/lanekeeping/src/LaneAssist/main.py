#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import time
import logging
from src.LaneKeeping.lanekeeping import LaneKeeping
from src.LaneDetection.detect import LaneDetection

stop_lanekeeping_value = 0
lane_viz_pub = None  # We'll initialize in main

# Define these variables globally so callbacks can access them:
real_world_example = True
skipped_frames = 10
frames_used = 0
time_sum = 0

def stop_lanekeeping_callback(msg):
    global stop_lanekeeping_value
    stop_lanekeeping_value = msg.data

def image_callback(msg):
    global bridge, lk, ld, real_world_example, skipped_frames, frames_used, time_sum
    global stop_lanekeeping_value, lane_viz_pub

    if stop_lanekeeping_value == 1:
        return  # Stop processing if stop_lanekeeping_value is 1

    # Convert ROS -> OpenCV
    src = bridge.imgmsg_to_cv2(msg, "bgr8")

    if real_world_example or frames_used % skipped_frames == 0:
        frames_used += 1
        start = time.time()

        ld_frame = src.copy()
        # Lane detection
        results = ld.lanes_detection(ld_frame)

        # Lane keeping
        angle, annotated_frame = lk.lane_keeping(results)

        end = time.time()
        elapsed = end - start
        time_sum += elapsed

        # Publish the annotated image
        annotated_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        annotated_msg.header.stamp = rospy.Time.now()
        annotated_msg.header.frame_id = "camera_frame"
        lane_viz_pub.publish(annotated_msg)

        # Optionally show in OpenCV (commented out)
        # cv2.imshow('lk', annotated_frame)
        # cv2.waitKey(1)

def main():
    global lane_viz_pub, bridge, lk, ld
    rospy.init_node('lane_detection_node')

    bridge = CvBridge()
    lane_viz_pub = rospy.Publisher("/lane_detection_viz", Image, queue_size=1)

    camera = "455"
    lk = LaneKeeping(640, 480, logging.getLogger('Root logger'), camera)
    ld = LaneDetection(640, 480, camera, lk)
    if real_world_example:
        ld.square_pulses_min_height = 80
        ld.square_pulses_pix_dif = 10
        ld.square_pulses_min_height_dif = 20
        ld.square_pulses_allowed_peaks_width_error = 15

    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.Subscriber("/stop_lanekeeping", Int32, stop_lanekeeping_callback)
    rospy.spin()

    if frames_used > 0:
        print(f"\nNDRTS ~ Info: Average time: {time_sum / frames_used}s\n\n")

if __name__ == "__main__":
    main()
