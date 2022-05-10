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
    rospy.init_node("multiRGB")
    rgbIndex = rospy.get_param("~rgbIndex", default=0)
    topic_RGB = config['topic_RGB']
    size = config['size_RGB_720p']
    frameRate = config['frameRate']
    codec = cv2.VideoWriter_fourcc(*'MJPG')

    w, h = size
    pub_rgb = rospy.Publisher(topic_RGB, Image, queue_size=1)
    rate = rospy.Rate(frameRate)
    rgbcam = cv2.VideoCapture(rgbIndex)
    # rgbcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    rgbcam.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    rgbcam.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    rgbcam.set(cv2.CAP_PROP_FOURCC, codec)
    rgbcam.set(cv2.CAP_PROP_FPS, frameRate)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        rgbret, rgb_image = rgbcam.read()
        pub_rgb.publish(bridge.cv2_to_imgmsg(rgb_image))
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass