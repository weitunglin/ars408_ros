#! /usr/bin/env python3
# coding=utf-8
import rospy

from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

import cv2
import numpy as np


frameRate = 20

topic_RGB = "/rgbImg"
topic_TRM = "/thermalImg"

size_RGB = (800, 600)
size_TRM = (640, 512)

global nowImg_RGB
global nowImg_TRM

alpha = 0.7
beta = 1-alpha
gamma = 0


def find_homography():
    # RGB Four corners
    pts_RGB = np.array([[149, 96], [239, 373], [418, 107], [404, 336], [403, 374], [526, 437], [702, 345]])
    # Thermal Four corners
    pts_TRM = np.array([[ 80, 52], [165, 334], [352,  57], [337, 293], [337, 331] ,[460, 396], [634, 302]])

    homo, status = cv2.findHomography(pts_TRM, pts_RGB)
    return homo

def get_dual(RGBImg, TRMImg, homography):
    # 熱像形變黏貼
    img = np.zeros((600, 800, 3), np.uint8)
    img_out = cv2.warpPerspective(TRMImg, homography, (RGBImg.shape[1], RGBImg.shape[0]))
    img[48:538, 82:704] = img_out[48:538, 82:704]
    img_Jcolor = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    # 熱像疊合上 RGB
    img_Fusion = cv2.addWeighted(RGBImg, alpha, img_Jcolor, beta, gamma)

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
    pub_dual = rospy.Publisher("/dualImg", Image, queue_size=1)
    bridge = CvBridge()

    # 校正資訊
    h = find_homography()
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue

        RGBImg = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
        TRMImg = cv2.resize(nowImg_TRM, size_TRM, cv2.INTER_CUBIC)
        dualImg = get_dual(RGBImg, TRMImg, h)
        img_message = bridge.cv2_to_imgmsg(dualImg)
        pub_dual.publish(img_message)
        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
