#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import jetson_inference
import jetson_utils
import numpy as np

# Initialize ROS node
rospy.init_node('semafor_detection_publisher')

# Create a publisher for the class names
class_name_pub = rospy.Publisher('detected_colour', String, queue_size=10)

# Initialize OpenCV bridge
bridge = CvBridge()

# Load class labels
class_labels = []
with open("/home/jetson/Desktop/NDRTS/NDRTS-BFMC2024/src/vision/src/ssd/models/voc-model-labels.txt", "r") as f:
    class_labels = [line.strip() for line in f.readlines()]

# load the object detection model
net = jetson_inference.detectNet(argv=["--model=/home/jetson/Desktop/NDRTS/NDRTS-BFMC2024/src/vision/src/ssd/models/mb2-ssd-lite.onnx", "--labels=/home/jetson/Desktop/NDRTS/NDRTS-BFMC2024/src/vision/src/ssd/models/voc-model-labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes"], threshold=0.5)
# display = jetson_utils.videoOutput("display://0")  # 'my_video.mp4' for file

# Define a callback function to handle incoming images
def image_callback(msg):
    global net
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")

        # Define the ROI coordinates
        height, width, _ = cv_image.shape
        roi_start_x = int(width * 0.7)  # IMPORTANT TE PUP EU 80% of the width from left
        roi_end_x = width - 1  # End of the width
        roi_start_y = 0  # Top of the image
        roi_end_y = height - 1  # Bottom of the image

        # Crop the image to extract the ROI
        roi = cv_image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]
        print
        # Convert to CUDA image format
        cuda_img = jetson_utils.cudaFromNumpy(roi)

        # Perform object detection on the cropped ROI
        detections = net.Detect(cuda_img)

        for detection in detections:
            class_name = class_labels[detection.ClassID]
            if detection.Confidence >= 0.90:
                print("Class: {}, Confidence: {}".format(class_name, detection.Confidence))
                # Publish class name to ROS topic
                class_name_pub.publish(class_name)

        #Display the processed image (optional)
        # display.Render(cuda_img)
        # display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

    except Exception as e:
        print(e)

# Subscribe to the ROS topic containing the image feed
rospy.Subscriber("/camera/image_raw", Image, image_callback)

# Spin ROS
rospy.spin()
