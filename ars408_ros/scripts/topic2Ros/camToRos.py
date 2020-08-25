#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

import cv2


def talker():
    rospy.init_node("cam2Ros")
    camIndex = rospy.get_param("~camIndex", default=0)
    topicName = rospy.get_param("~topicName", default="/rgbImg")

    pub = rospy.Publisher(topicName, Image, queue_size=1)
    rate = rospy.Rate(20)

    cam = cv2.VideoCapture(camIndex)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
