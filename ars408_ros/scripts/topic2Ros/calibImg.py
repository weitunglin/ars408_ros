#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2
import numpy as np
import yaml, os

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']
topic_RGB = config['topic_RGB']
topic_RGB_Calib = config['topic_RGB_Calib']
size_RGB = config['size_RGB']
# size_RGB = config['size_RGB_720p']

cmatrix = np.array(config['K']).reshape(3,3)
dmatrix = np.array(config['D']).reshape(1,5)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cmatrix, dmatrix, size_RGB, 1, size_RGB)

global pub_RGB

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img = cv2.undistort(img, cmatrix, dmatrix, None, newcameramtx) 
    x, y, w, h = roi
    img = img[y:y+h, x:x+w]
    img = cv2.resize(img, size_RGB, cv2.INTER_CUBIC)
    pub_RGB.publish(bridge.cv2_to_imgmsg(img))

def listener():
    global pub_RGB
    rospy.init_node("calibImg")
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    pub_RGB = rospy.Publisher(topic_RGB_Calib, Image, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
