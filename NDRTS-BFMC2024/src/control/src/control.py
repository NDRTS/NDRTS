#!/usr/bin/env python3
import signal
import time
import rospy
from std_msgs.msg import String, Float32, Int32
from signs import *  # your maneuver helpers

# ---------------------------------------------------------------------------
# Globals from subscribed topics
# ---------------------------------------------------------------------------
dist_front = 9999.0
line = 0

def dist_front_callback(msg: Float32):
    global dist_front
    dist_front = msg.data

def line_callback(msg: Int32):
    global line
    line = msg.data

# ---------------------------------------------------------------------------
# Publisher for stopping lane keeping
# ---------------------------------------------------------------------------
stop_nav_pub = rospy.Publisher('/stop_lanekeeping_cmd', Int32, queue_size=10)

def send_stop_signal():
    rospy.loginfo("🚨 Sending STOP signal")
    stop_nav_pub.publish(Int32(data=1))

def send_resume_signal():
    rospy.loginfo("▶️ Sending RESUME signal")
    stop_nav_pub.publish(Int32(data=0))

# ---------------------------------------------------------------------------
# TrafficSignDetector class with cooldown logic
# ---------------------------------------------------------------------------
class TrafficSignDetector:
    def __init__(self):
        self.votes = {}  # cleaned_label -> {count, last}
        self.cooldown_until = {}  # cleaned_label -> time

        self.threshold = 3
        self.time_window = 1.0
        self.remove_timeout = 10.0

        self.cooldown_secs = {
            "Stopsign": 20.0,
            "Parkingsign": 20.0,
            "Round-aboutsign": 5.0,
            "Crosswalksign": 3.0,
            "Prioritysign": 3.0
        }

    def update_detection(self, raw_label: str):
        now = rospy.get_time()
        # label = raw_label.strip().replace(",", "")
        label = raw_label

        if label in self.cooldown_until and now < self.cooldown_until[label]:
            return

        if label not in self.votes:
            self.votes[label] = {'count': 0, 'last': now}
        self.votes[label]['count'] += 1
        self.votes[label]['last'] = now

        if self.votes[label]['count'] >= self.threshold:
            rospy.loginfo(f"✅ Confirmed detection: {label}")
            self._handle_maneuver(label)
            self.cooldown_until[label] = now + self.cooldown_secs.get(label, 5.0)
            self.votes.pop(label, None)

    def remove_old_detections(self):
        now = rospy.get_time()
        for label in list(self.votes):
            if now - self.votes[label]['last'] > self.remove_timeout:
                self.votes.pop(label)

    def _handle_maneuver(self, label: str):
        if label == "Stopsign":
            STOP_SIGN()
            time.sleep(1.5)

        elif label == "Crosswalksign":
            start = rospy.get_time()
            while rospy.get_time() - start < 10:
                if dist_front <= 30:
                    CROSSWALK_WITH_PEDESTRIAN()
                else:
                    CROSSWALK_WITHOUT_PEDESTRIAN()
                rospy.sleep(0.25)

        elif label == "Prioritysign":
            PRIORITY_SIGN()

        elif label == "Round-aboutsign" and line == 1:
            send_stop_signal()
            ROUNDABOUTSIGN()
            send_resume_signal()

        elif label == "Parkingsign":
            send_stop_signal()
            time.sleep(1.5)
            PARALLEL_PARK_LEFT()

        else:
            rospy.loginfo(f"⚠️ No handler defined for: {label}")

# ---------------------------------------------------------------------------
# ROS callbacks and main loop
# ---------------------------------------------------------------------------
detector = TrafficSignDetector()

def sign_callback(msg: String):
    raw = msg.data
    cleaned = raw.strip().replace(",", "")
    rospy.loginfo(f"🔤 Raw label: '{raw}' → Cleaned: '{cleaned}'")
    detector.update_detection(cleaned)

def signal_handler(sig, frame):
    send_resume_signal()
    rospy.signal_shutdown("Ctrl-C pressed")

def main():
    rospy.init_node('sign_publisher', anonymous=True)

    rospy.Subscriber('/detected_class', String, sign_callback)
    rospy.Subscriber('/distance_front', Float32, dist_front_callback)
    rospy.Subscriber('/line', Int32, line_callback)

    signal.signal(signal.SIGINT, signal_handler)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        detector.remove_old_detections()
        rate.sleep()

if __name__ == '__main__':
    main()
