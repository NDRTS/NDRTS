#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32
import math
import numpy as np
import threading
import sys, termios, tty, os

latest_scan = None  
latest_averages = {}

# Control parameters (in centimeters)
TARGET_LEFT = 40.0   # desired distance on the left side
TARGET_RIGHT = 25.0  # desired distance on the right side
MIN_VALID_CM = 30.0  # lower bound (too close)
MAX_VALID_CM = 80.0  # upper bound (too far)
K = 1                # proportional gain

# Maximum steering angle in degrees
MAX_STEER_DEG = 20.0

# Smoothing parameters
MAX_STEER_STEP = 1.0       # maximum change (in degrees) allowed per update
last_correction = 0.0      # global variable to store the last steering command

# Global publisher for stopping lane keeping; will be initialized in lidar_monitor()
stop_lanekeeping_pub = None

# Global flag to avoid continuous publishing of stop/resume commands
lane_keeping_stopped = None  # None indicates not yet set

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
    global last_correction, lane_keeping_stopped

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
        # In your configuration, right reading is at 90° and left at -90°.
        right_cm = averages[90]
        left_cm = averages[-90]
        rospy.loginfo("Left: %.2f cm, Right: %.2f cm", left_cm, right_cm)
        
        total = left_cm + right_cm
        
        # Check the condition: if total is less than 100 cm, we want to stop lane keeping.
        if total < 100:
            # Publish stop command only if not already stopped.
            if lane_keeping_stopped is not True:
                stop_lanekeeping_pub.publish(1)
                rospy.loginfo("Published stop command (1) because total (%.2f cm) < 100 cm", total)
                lane_keeping_stopped = True
            # Process steering even if lane-keeping is stopped? (Your logic might skip corrections here.)
            # For now, we continue with corrections even if stopping is triggered.
            error_left = left_cm - TARGET_LEFT    # positive if left is too far
            error_right = right_cm - TARGET_RIGHT  # positive if right is too far
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
            # Only publish a resume command if previously stopped.
            if lane_keeping_stopped is not False:
                stop_lanekeeping_pub.publish(0)
                rospy.loginfo("Published resume command (0) because total (%.2f cm) >= 100 cm", total)
                lane_keeping_stopped = False
            rospy.loginfo("Total distance (%.2f cm) >= 100 cm. No steering command sent.", total)
    else:
        rospy.loginfo("Side measurements (left/right) not available.")
    
    return averages

def lidar_monitor():
    global stop_lanekeeping_pub
    rospy.init_node('lidar_monitor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    # Initialize the stop lane-keeping publisher after node initialization.
    stop_lanekeeping_pub = rospy.Publisher('/stop_lanekeeping', Int32, queue_size=10)
    rospy.loginfo("Lidar monitor started.")
    
    rate = rospy.Rate(50)  # update at 50 Hz
    while not rospy.is_shutdown():
        if latest_scan is not None:
            process_scan(latest_scan, publish=True)
        rate.sleep()

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def exit_listener():
    """Listens for a 'q' key press and then exits the program."""
    while True:
        key = get_key()
        if key == 'q':
            print("Exiting script.")
            os._exit(0)

if __name__ == '__main__':
    exit_thread = threading.Thread(target=exit_listener)
    exit_thread.daemon = True
    exit_thread.start()
    
    try:
        lidar_monitor()
    except rospy.ROSInterruptException:
        pass
