#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_publisher(topic_name='/camera/image_raw'):
    # Initialize ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create a CvBridge instance to convert between ROS and OpenCV images
    bridge = CvBridge()
    
    # Create a ROS publisher for the image topic
    image_pub = rospy.Publisher(topic_name, Image, queue_size=30)
    
    # Use a GStreamer pipeline to access the camera hardware efficiently.
    # gst_pipeline = (
    #     "v4l2src device=/dev/video0 ! "
    #     "video/x-raw, width=640, height=480, framerate=30/1 ! "
    #     "videoconvert ! "
    #     "video/x-raw, format=BGR ! "
    #     "appsink"
    # )

    gst_pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=960, height=540, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    )

    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        rospy.logerr("Camera cannot be opened.")
        return
    
    rospy.loginfo(f"Publishing images on topic: {topic_name}")
    
    rate = rospy.Rate(10)  # Publish at 10Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Error reading frame from camera.")
            break

        # --- GPU Processing (optional) ---
        # Upload the frame to GPU memory
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)
        
        # Convert the frame to grayscale using the GPU
        gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
        
        # Download the processed image back to host memory
        processed_frame = gpu_gray.download()
        # ----------------------
        
        # --- Adjust Brightness ---
        # Reduce brightness by scaling pixel values (alpha=0.9 reduces brightness by 10%)
        adjusted_frame = cv2.convertScaleAbs(processed_frame, alpha=0.9, beta=0)
        # -------------------------
        
        # Convert the adjusted frame into a ROS Image message (using mono8 encoding)
        img_msg = bridge.cv2_to_imgmsg(processed_frame, encoding='mono8')
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "camera_frame"
        image_pub.publish(img_msg)
        
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
