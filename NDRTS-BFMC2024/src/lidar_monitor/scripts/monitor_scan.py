#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np

latest_scan = None  
latest_averages = {}

# Control parameters (in centimeters)
TARGET_DISTANCE_CM = 30.0  # desired distance on each side
MIN_VALID_CM = 30.0        # lower bound (too close)
MAX_VALID_CM = 80.0        # upper bound (too far)
K = 0.6                    # proportional gain

# Maximum steering angle in degrees
MAX_STEER_DEG = 20.0

# Smoothing parameters
MAX_STEER_STEP = 1.0       # maximum change (in degrees) allowed per update
last_correction = 0.0      # global variable to store the last steering command

def send_data(steer):
    """Publishes the steering correction.
    Positive correction means steer right, negative means steer left.
    """
    pub = rospy.Publisher('/direction_lane', Float32, queue_size=10)
    rospy.loginfo("Sending steering correction: %d°", steer)
    pub.publish(steer)

def scan_callback(scan):
    global latest_scan
    latest_scan = scan

def process_scan(scan, publish=True):
    global last_correction

    # Desired angles: front (0°), left (90°), back (180°), right (-90°)
    desired_angles_deg = [0, 90, 180, -90]
    window_rad = math.radians(10)  # ±10° window

    num_readings = len(scan.ranges)
    angles = np.array([scan.angle_min + i * scan.angle_increment for i in range(num_readings)])
    
    averages = {}
    for d_deg in desired_angles_deg:
        d_rad = math.radians(d_deg)
        indices = np.where((angles >= (d_rad - window_rad)) & (angles <= (d_rad + window_rad)))[0]
        if len(indices) > 0:
            valid_readings = [scan.ranges[i] for i in indices 
                              if not math.isinf(scan.ranges[i]) and not math.isnan(scan.ranges[i])]
            if valid_readings:
                avg_distance = sum(valid_readings) / len(valid_readings)
                avg_distance_cm = avg_distance * 100  # convert from meters to centimeters
                averages[d_deg] = avg_distance_cm
            else:
                averages[d_deg] = None
        else:
            averages[d_deg] = None

    global latest_averages
    latest_averages = averages.copy()

    # For steering, we care about the left (90°) and right (-90°) measurements.
    if averages.get(90) is not None and averages.get(-90) is not None:
        right_cm = averages[90]
        left_cm = averages[-90]
        rospy.loginfo("Left: %.2f cm, Right: %.2f cm", left_cm, right_cm)
        error_left = left_cm - TARGET_DISTANCE_CM
        error_right = right_cm - TARGET_DISTANCE_CM
        new_correction = K * (error_right - error_left)
        new_correction = max(min(new_correction, MAX_STEER_DEG), -MAX_STEER_DEG)
        delta = new_correction - last_correction
        if abs(delta) > MAX_STEER_STEP:
            correction = last_correction + math.copysign(MAX_STEER_STEP, delta)
        else:
            correction = new_correction
        last_correction = correction
        
        # Round correction to the nearest integer
        correction = int(round(correction))
        
        if publish:
            send_data(correction)
        rospy.loginfo("Computed correction: %d° (error_left: %.2f, error_right: %.2f)",
                      correction, error_left, error_right)
    else:
        rospy.loginfo("Side measurements (left/right) not available.")
    
    return averages

def lidar_monitor():
    rospy.init_node('lidar_monitor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.loginfo("Lidar monitor started.")
    
    rate = rospy.Rate(50)  # update at 50 Hz (adjust as needed)
    while not rospy.is_shutdown():
        if latest_scan is not None:
            process_scan(latest_scan, publish=True)
        rate.sleep()

if __name__ == '__main__':
    try:
        lidar_monitor()
    except rospy.ROSInterruptException:
        pass
