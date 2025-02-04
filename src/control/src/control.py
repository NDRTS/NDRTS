#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32, Int32
from signs import *
import signal

# Definire publisher global pentru oprirea menținerii benzii
stop_lanekeeping_publisher = None

# Callback pentru actualizarea distanței frontale
def dist_front_callback(data):
    global dist_front
    dist_front = data.data

# Callback pentru actualizarea distanței laterale
def dist_lateral_callback(data):
    global dist_lateral
    dist_lateral = data.data

# Callback pentru actualizarea liniei detectate
def line_callback(data):
    global line
    line = data.data

# Inițializare publisher pentru oprirea navigației
stop_nav_pub = rospy.Publisher('/stop_nav', Int32, queue_size=10)

# Funcție pentru trimiterea semnalului de oprire
def send_stop_signal():
    rospy.loginfo("Sending STOP signal")
    stop_msg = Int32()
    stop_msg.data = 1
    stop_nav_pub.publish(stop_msg)

# Funcție pentru trimiterea semnalului de reluare
def send_resume_signal():
    rospy.loginfo("Sending RESUME signal")
    resume_msg = Int32()
    resume_msg.data = 0
    stop_nav_pub.publish(resume_msg)

# Clasă pentru detectarea semnelor de circulație
class TrafficSignDetector:
    def __init__(self):
        self.detection_counts = {}
        self.threshold = 2  # Prag pentru detectare sigură
        self.time_window = 5  # Fereastră de timp pentru detecții
        self.remove_timeout = 15  # Timeout pentru eliminarea detecțiilor vechi

    # Funcție pentru actualizarea detecțiilor
    def update_detection(self, label):
        current_time = rospy.get_time()
        global stop_lanekeeping_publisher
        if label in self.detection_counts:
            self.detection_counts[label]['count'] += 1
            self.detection_counts[label]['last_detection_time'] = current_time
            # Acțiuni pentru diferite semne de circulație
            if self.detection_counts[label]['count'] == self.threshold:
                if label == "Stopsign":
                    send_stop_signal()
                    STOP_SIGN()
                    time.sleep(1.5)
                    send_resume_signal()
                if label == "Crosswalksign":
                    cross = True
                    start_time = rospy.get_time()
                    while cross:
                        if dist_front <= 30:
                            CROSSWALK_WITH_PEDESTRIAN()
                        elif dist_front >= 40:
                            CROSSWALK_WITHOUT_PEDESTRIAN()
                        current_time = rospy.get_time()
                        if current_time - start_time >= 10:
                            break
                if label == "Prioritysign":
                    PRIORITY_SIGN()
                if label == "Highwayentrancesign":
                    HIGHWAY_ENTER()
                if label == "Highwayexitsign":
                    HIGHWAY_EXIT()
                elif label == "Round-aboutsign" and line==1:
                    send_stop_signal()
                    ROUNDABOUTSIGN()
                    send_resume_signal()       
                elif label == "Parkingsign":
                    print("Parkinsign detected")
                    time.sleep(5) 
                    dist_lateral_1 = dist_lateral 
                    time.sleep(5)  
                    dist_lateral_2 = dist_lateral  
                    send_stop_signal()
                    if dist_lateral_1 >= 15 and dist_lateral_2 <= 15:
                        print("First parking space is available")
                        PARKING_SPACE_1()
                    else:
                        print("First parking space is not available")
                        send_resume_signal()
                else:
                    rospy.loginfo(f"Detected {label}")
        else:
            self.detection_counts[label] = {'count': 1, 'last_detection_time': current_time}
     
    # Verifică detecțiile adevărate
    def check_for_true_detections(self):
        current_time = rospy.get_time()
        true_detections = []
        for label, info in self.detection_counts.items():
            if info['count'] >= self.threshold and current_time - info['last_detection_time'] <= self.time_window:
                true_detections.append(label)
        return true_detections

    # Elimină detecțiile vechi
    def remove_old_detections(self):
        current_time = rospy.get_time()
        labels_to_remove = []
        for label, info in self.detection_counts.items():
            if current_time - info['last_detection_time'] > self.remove_timeout:
                labels_to_remove.append(label)
        for label in labels_to_remove:
            del self.detection_counts[label]

    # Resetează contorul de detecții
    def reset_counts(self):
        self.detection_counts = {}

# Callback pentru procesarea datelor de la semnele de circulație detectate
def callback(data):
    global pedestrian
    global stai
    label = data.data
    pedestrian=data.data
    if pedestrian == "Pedestrian":
        stai = True
    else: 
        stai = False

    detector.update_detection(label)

# Handler pentru semnalele de întrerupere
def signal_handler(sig, frame):
    global stop_lanekeeping_publisher
    if stop_lanekeeping_publisher is not None:
        stop_lanekeeping_publisher.publish(0)
    rospy.signal_shutdown("Ctrl+C pressed")

# Funcția principală de inițializare a nodului ROS
def main():
    rospy.init_node('sign_publisher', anonymous=True)

if __name__ == '__main__':
    main()
    detector = TrafficSignDetector()
    rospy.Subscriber('/detected_class', String, callback)
    rospy.Subscriber('/distance_front', Float32, dist_front_callback)
    rospy.Subscriber('/distance_lateral', Float32, dist_lateral_callback)
    rospy.Subscriber('/line', Int32, line_callback)
    stop_lanekeeping_publisher = rospy.Publisher('/stop_nav', Int32, queue_size=10)
    signal.signal(signal.SIGINT, signal_handler)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        detector.remove_old_detections()
        rate.sleep()

    rospy.spin()
