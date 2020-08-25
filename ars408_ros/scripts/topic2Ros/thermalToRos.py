#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

import cv2


def talker():
    rospy.init_node("thermal")
    pub = rospy.Publisher("/thermalImg", Image, queue_size=1)
    rate = rospy.Rate(20)

    cam = cv2.VideoCapture(2)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    # cam.set(cv2.CAP_PROP_XI_IMAGE_DATA_BIT_DEPTH, False)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, cv_image = cam.read()
        ret, cv_image = cam.read()
        img_message = bridge.cv2_to_imgmsg(cv_image)
        pub.publish(img_message)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
