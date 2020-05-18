#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2


def talker():
    rospy.init_node("cam", anonymous=True)
    pub = rospy.Publisher("/camImg", Image, queue_size=100)
    
    rate = rospy.Rate(100)
    cam = cv2.VideoCapture(0)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, cv_image = cam.read()
        img_message = bridge.cv2_to_imgmsg(cv_image)
        pub.publish(img_message)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
