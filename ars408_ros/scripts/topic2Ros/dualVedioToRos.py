#! /usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2

def talker():
    rospy.init_node("dualVedioToRosV2")
    pub_rgb = rospy.Publisher("/rgbImg", Image, queue_size=1)
    pub_trm = rospy.Publisher("/thermalImg", Image, queue_size=1)
    rate = rospy.Rate(100)

    rgbcam = cv2.VideoCapture(1)
    rgbcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    thrcam = cv2.VideoCapture(2)
    thrcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        rgbret, rbg_image = rgbcam.read()
        rgbret, rbg_image = rgbcam.read()
        thrret, trm_image = thrcam.read()
        thrret, trm_image = thrcam.read()

        img_message = bridge.cv2_to_imgmsg(rbg_image)
        pub_rgb.publish(img_message)
        img_message = bridge.cv2_to_imgmsg(trm_image)
        pub_trm.publish(img_message)
    
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
