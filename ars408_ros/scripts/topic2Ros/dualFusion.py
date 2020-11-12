#! /usr/bin/env python3
# coding=utf-8
import rospy

from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

import cv2
import numpy as np


frameRate = 20

topic_RGB = "/calibImg"
topic_TRM = "/thermalImg"

# Origin
size_RGB = (640, 480)
size_TRM = (640, 512)

# Fisheye
size_RGB = (800, 600)
size_TRM = (640, 512)

global nowImg_RGB
global nowImg_TRM

alpha = 0.6
beta = 1-alpha
gamma = 0


def find_homography():
    # Find 4 corners of RGB and TRM
    
    # 0724
    # pts_RGB = np.array([[149, 96], [239, 373], [418, 107], [404, 336], [403, 374], [526, 437], [702, 345]])
    # pts_TRM = np.array([[ 80, 52], [165, 334], [352,  57], [337, 293], [337, 331] ,[460, 396], [634, 302]])
    
    # 0826
    # pts_RGB = np.array([[96, 107], [304, 205], [675, 141], [80, 358], [371, 448], [377, 374], [517, 191], [696, 442], [670, 326], [623, 134]])
    # pts_TRM = np.array([[ 31, 64], [238, 154], [618,  91], [11, 317], [303, 407], [311, 330], [455, 140], [632, 393], [604, 278], [560,  83]])
    
    # 0909
    # pts_RGB = np.array([[164, 245], [610, 187], [286, 298], [331, 183], [145, 428], [165,  66], [312, 364], [104, 386]])
    # pts_TRM = np.array([[188, 263], [555, 222], [283, 307], [323, 214], [165, 415], [186, 114], [305, 362], [138, 379]])

    # 0924 Fisheye
    # pts_RGB = np.array([[1009, 308], [993, 789], [817, 344], [1144, 676], [603, 714], [921, 479], [466, 395]])
    # pts_TRM = np.array([[ 500,  37], [488, 373], [350,  48], [ 615, 295], [159, 319], [432, 156], [ 55,  88]])

    # 1016 Fisheye
    pts_RGB = np.array([[554,250],[450, 137],[548, 482],[412, 176],[396, 300],[366, 450],[407, 234],[257, 310],[307, 374]])
    pts_TRM = np.array([[515,173],[350,  12],[509, 499],[289,  73],[270, 246],[216, 469],[278, 149],[ 19, 251],[104, 370]])

    homo, status = cv2.findHomography(pts_TRM, pts_RGB)
    return homo

def get_dual(RGBImg, TRMImg, homography):
    # 0724
    # img_out[48:538, 82:704]
    # 0826
    # img[55:548, 73:697] = img_out[55:548, 73:697]             # just make scale Img to be rectangle not  like this shape (/_\)
    # img_Fusion = img_Fusion[55:548, 73:697]

    # 熱像形變黏貼
    img = np.zeros((RGBImg.shape[1], RGBImg.shape[0], 3), np.uint8)
    img_out = cv2.warpPerspective(TRMImg, homography, (RGBImg.shape[1], RGBImg.shape[0]))  # Thermal will be scale to RGB size (640*512 to 640*480)
    img_Jcolor = cv2.applyColorMap(img_out, cv2.COLORMAP_JET)
    # 熱像疊合上 RGB
    img_Fusion = cv2.addWeighted(RGBImg, alpha, img_Jcolor, beta, gamma)
    # img_Fusion = img_Fusion

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
