#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(msg):
    pub = rospy.Publisher('/stop_lanekeeping', Int32, queue_size=10)
    pub.publish(msg.data)

def main():
    global pub
    rospy.init_node('stop_lanekeeping_manager', anonymous=True)

    pub = rospy.Publisher('/stop_lanekeeping', Int32, queue_size=10)
    rospy.Subscriber('/stop_lanekeeping_cmd', Int32, callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
