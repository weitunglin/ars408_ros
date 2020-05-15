#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import os


def talker():
    rospy.init_node("thermal", anonymous=True)
    pub = rospy.Publisher("/thermalImg", Image, queue_size=100)
    rate = rospy.Rate(100)

    cam = cv2.VideoCapture(2)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cam.set(cv2.CAP_PROP_XI_IMAGE_DATA_BIT_DEPTH, False)

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
