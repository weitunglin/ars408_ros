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
oldCamera = config['oldCamera']

topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']

# Fisheye
size_RGB = config['size_RGB_Calib_output']
size_TRM = config['size_TRM_output']

# old camera
if oldCamera:
    topic_RGB = config['topic_RGB']
    size_RGB = (640,480)
    size_TRM = (640,512)

global nowImg_RGB
global nowImg_TRM

alpha = 0.6
beta = 1-alpha
gamma = 0


def find_homography():
    # Find 4 corners of RGB and TRM
    pts_RGB = np.array(config['pts_RGB'])
    pts_TRM = np.array(config['pts_TRM'])

    homo, status = cv2.findHomography(pts_TRM, pts_RGB)
    return homo

def get_dual(RGBImg, TRMImg, homography):
    # 熱像形變黏貼
    img_out = cv2.warpPerspective(TRMImg, homography, (RGBImg.shape[1], RGBImg.shape[0]))  # Thermal will be scale to RGB size (640*512 to 640*480)
    img_Jcolor = cv2.applyColorMap(img_out, cv2.COLORMAP_JET)
    # 熱像疊合上 RGB
    img_Fusion = cv2.addWeighted(RGBImg, alpha, img_Jcolor, beta, gamma)
    if oldCamera:
        ROI = config['ROI']
        img_Fusion = img_Fusion[ROI[0][0]:ROI[0][1], ROI[1][0]:ROI[1][1]]

    return img_Fusion

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB
    nowImg_RGB = img.copy()

def callback_TRMImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_TRM
    nowImg_TRM = img.copy()

def listener():
    rospy.init_node("dualFusion")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)
    pub_dual = rospy.Publisher(topic_Dual, Image, queue_size=1)
    bridge = CvBridge()

    # 校正資訊
    h = find_homography()
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue

        RGBImg = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
        TRMImg = cv2.resize(nowImg_TRM, size_TRM, cv2.INTER_CUBIC)
        dualImg = get_dual(RGBImg, TRMImg, h)
        if oldCamera:
            dualImg = cv2.resize(dualImg, (800,600), cv2.INTER_CUBIC)
        img_message = bridge.cv2_to_imgmsg(dualImg)
        pub_dual.publish(img_message)
        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
