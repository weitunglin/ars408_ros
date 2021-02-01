#! /usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2
import yaml, os

def talker():
    rospy.init_node("dualVedioToRos")
    frameRate = 20

    pub_rgb1 = rospy.Publisher("/rgbImg1", Image, queue_size=1)
    pub_rgb2 = rospy.Publisher("/rgbImg2", Image, queue_size=1)
    pub_rgb3 = rospy.Publisher("/rgbImg3", Image, queue_size=1)
    pub_rgb4 = rospy.Publisher("/rgbImg4", Image, queue_size=1)
    pub_rgb5 = rospy.Publisher("/rgbImg5", Image, queue_size=1)
    pub_rgb6 = rospy.Publisher("/rgbImg6", Image, queue_size=1)
    rate = rospy.Rate(frameRate)

    # rgbcam1 = cv2.VideoCapture(1)
    rgbcam2 = cv2.VideoCapture(2)
    # rgbcam3 = cv2.VideoCapture(3)
    rgbcam4 = cv2.VideoCapture(4)
    # rgbcam5 = cv2.VideoCapture(5)
    rgbcam6 = cv2.VideoCapture(7)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        # rgbret1, rgb_image1 = rgbcam1.read()
        rgbret2, rgb_image2 = rgbcam2.read()
        # rgbret3, rgb_image3 = rgbcam3.read()
        rgbret4, rgb_image4 = rgbcam4.read()
        # rgbret5, rgb_image5 = rgbcam5.read()
        rgbret6, rgb_image6 = rgbcam6.read()

        # pub_rgb2.publish(bridge.cv2_to_imgmsg(rgb_image1))
        pub_rgb2.publish(bridge.cv2_to_imgmsg(rgb_image2))
        # pub_rgb2.publish(bridge.cv2_to_imgmsg(rgb_image3))
        pub_rgb4.publish(bridge.cv2_to_imgmsg(rgb_image4))
        # pub_rgb2.publish(bridge.cv2_to_imgmsg(rgb_image5))
        pub_rgb6.publish(bridge.cv2_to_imgmsg(rgb_image6))

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
