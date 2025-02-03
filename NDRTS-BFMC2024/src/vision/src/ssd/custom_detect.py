import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
import jetson_inference
import jetson_utils
import numpy as np
import time

# Inițializează nodul ROS
rospy.init_node('object_detection_publisher')

# Publisher pentru clasa detectată
class_name_pub = rospy.Publisher('detected_class', String, queue_size=10)

# Publisher pentru frame rate
frame_rate_pub = rospy.Publisher('frame_rate', Float32, queue_size=10)

# Inițializează bridge-ul OpenCV
bridge = CvBridge()

# Citește etichetele claselor
class_labels = []
with open("/home/jetson/Desktop/NDRTS/src/vision/src/ssd/models/voc-model-labels.txt", "r") as f:
    class_labels = [line.strip() for line in f.readlines()]

# Inițializează rețeaua de detecție
net = jetson_inference.detectNet(argv=["--model=/home/jetson/Desktop/NDRTS/src/vision/src/ssd/models/mb2-ssd-lite.onnx", 
                                       "--labels=/home/jetson/Desktop/NDRTS/src/vision/src/ssd/models/voc-model-labels.txt", 
                                       "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes"], threshold=0.5)

frame_rate = 0
last_frame_time = time.time()

def image_callback(msg):
    global net, frame_rate, last_frame_time
    try:
        # Conversia imaginii ROS în imagine OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")

        # Delimitarea regiunii de interes (ROI)
        height, width, _ = cv_image.shape
        roi_start_x = int(width * 0.7)
        roi_end_x = width - 1  
        roi_start_y = 0  
        roi_end_y = height - 1  
        roi = cv_image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]

        # Conversia imaginii OpenCV în imagine CUDA 
        cuda_img = jetson_utils.cudaFromNumpy(roi)

        # Detecție obiecte
        detections = net.Detect(cuda_img)

        # Iterare prin detecții
        for detection in detections:
            class_name = class_labels[detection.ClassID]
            if detection.Confidence >= 0.90:
                print("Class: {}, Confidence: {}".format(class_name, detection.Confidence))
                # Publicarea clasei detectate pe topicul de ROS
                class_name_pub.publish(class_name)

        # Calcularea frame rate-ului
        current_time = time.time()
        frame_rate = 1.0 / (current_time - last_frame_time)
        last_frame_time = current_time

        # Publicarea frame rate-ului pe topicul de ROS
        frame_rate_pub.publish(frame_rate)

    except Exception as e:
        print(e)

# Abonare la topicul de ROS pentru imaginea de la cameră
rospy.Subscriber("/camera/image_raw", Image, image_callback)

# Spin ROS
rospy.spin()
