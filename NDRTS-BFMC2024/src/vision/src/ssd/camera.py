#!/usr/bin/env python3

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import numpy as np

# Funcția pentru publicarea imaginilor de la camera
def camera_publisher(video_source=0, topic_name='/camera/image_raw'):
    # Inițializează nodul ROS
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Creează un obiect CvBridge pentru conversia dintre OpenCV și ROS
    bridge = CvBridge()
    
    # Creează un publisher ROS pentru a publica imagini pe un topic specificat
    image_pub = rospy.Publisher(topic_name, Image, queue_size=30)
    
    # Deschide sursa video (implicit, camera implicită a sistemului)
    cap = cv2.VideoCapture(video_source)
    
    # Verifică dacă camera a fost deschisă cu succes
    if not cap.isOpened():
        rospy.logerr('Camera nu poate fi deschisă.')
        return
    
    # Informații despre topicul pe care se publică imaginile
    rospy.loginfo(f'Publicare imagini pe topicul {topic_name}.')
    
    # Setează frecvența de publicare la 10Hz
    rate = rospy.Rate(10)
    
    # Bucla principală de publicare a imaginilor
    while not rospy.is_shutdown():
        # Citește un frame de la cameră
        ret, frame = cap.read()
        
        # Verifică dacă frame-ul a fost citit cu succes
        if not ret:
            rospy.logerr('Eroare la citirea frame-ului de la camera.')
            break
        
        # Dacă frame-ul are 3 canale (RGB)
        if frame.shape[2] == 3:  
            # Convertește frame-ul din RGB în BGR (format utilizat de OpenCV)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Redimensionează frame-ul la 640x480 (dacă este necesar)
            frame = cv2.resize(frame, (640, 480))
        
        # Convertește frame-ul din format OpenCV în mesaj ROS
        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Publică mesajul pe topicul specificat
        image_pub.publish(img_msg)
        
        # Așteaptă până la următoarea iterație
        rate.sleep()
    
    # Eliberează resursele camerei
    cap.release()

# Punctul de intrare al scriptului
if __name__ == '__main__':
    try:
        # Apelează funcția de publicare a imaginilor
        camera_publisher()
    except rospy.ROSInterruptException:
        # Gestionare excepție în cazul întreruperii ROS
        pass
