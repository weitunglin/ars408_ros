#! /usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2
import yaml, os

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

def talker():
    rospy.init_node("dualVedioToRos")
    rgbIndex = config['rgbIndex']
    thrIndex = config['thrIndex']
    topic_RGB = config['topic_RGB']
    topic_TRM = config['topic_TRM']
    frameRate = config['frameRate']

    pub_rgb = rospy.Publisher(topic_RGB, Image, queue_size=1)
    pub_thr = rospy.Publisher(topic_TRM, Image, queue_size=1)
    rate = rospy.Rate(frameRate)

    rgbcam = cv2.VideoCapture(rgbIndex)
    rgbcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    thrcam = cv2.VideoCapture(thrIndex)
    thrcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        rgbret, rgb_image = rgbcam.read()
        thrret, trm_image = thrcam.read()

        img_message = bridge.cv2_to_imgmsg(rgb_image)
        pub_rgb.publish(img_message)
        img_message = bridge.cv2_to_imgmsg(trm_image)
        pub_thr.publish(img_message)
    
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
