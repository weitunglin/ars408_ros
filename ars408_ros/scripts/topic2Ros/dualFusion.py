#! /usr/bin/env python2
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

alpha = 0.3
beta = 1-alpha
gamma = 0

def find_homography():
    # RGB Four corners
    pts_RGB_75 = np.array([[300, 347], [347, 347], [300, 404], [347, 404]])
    pts_RGB_150 = np.array([[303, 313], [325, 313], [303, 341], [325, 341]])
    pts_RGB_75up = np.array([[310, 114], [357, 114], [310, 170], [357, 170]])
    pts_RGB_150up = np.array([[314, 198], [336, 198], [314, 225], [336, 225]])
    pts_RGB_final = np.array([[63, 38], [533, 116], [118, 462], [533, 307]])

    # Thermal Four corners
    pts_Thermal_75 = np.array([[315, 308], [365, 308], [315, 369], [365, 369]])
    pts_Thermal_150 = np.array([[322, 273], [345, 273], [322, 302], [345, 302]])
    pts_Thermal_75up = np.array([[324, 72], [373, 72], [324, 129], [373, 129]])
    pts_Thermal_150up = np.array([[333, 155], [356, 155], [333, 183], [356, 183]])
    pts_Thermal_final = np.array([[91, 8], [554, 66], [135, 424], [559, 266]])

    h_75, status75 = cv2.findHomography(pts_Thermal_75, pts_RGB_75)
    h_150, status150 = cv2.findHomography(pts_Thermal_150, pts_RGB_150)
    h_75up, status15 = cv2.findHomography(pts_Thermal_75up, pts_RGB_75up)
    h_150up, status20 = cv2.findHomography(pts_Thermal_150up, pts_RGB_150up)
    h_final, statusfinal = cv2.findHomography(pts_Thermal_final, pts_RGB_final)

    result_h = np.array((h_75, h_150, h_75up, h_150up,h_final))
    return result_h

def get_dual(RGBImg, TImg, homography):
    h75, h150, h75up, h150up, hfinal = homography[0], homography[1], homography[2], homography[3], homography[4]

    im_out75 = cv2.warpPerspective(TImg, h75, (RGBImg.shape[1], RGBImg.shape[0]))
    im_out150 = cv2.warpPerspective(TImg, h150, (RGBImg.shape[1], RGBImg.shape[0]))
    im_out75up = cv2.warpPerspective(TImg, h75up, (RGBImg.shape[1], RGBImg.shape[0]))
    im_out150up = cv2.warpPerspective(TImg, h150up, (RGBImg.shape[1], RGBImg.shape[0]))
    im_outfinal = cv2.warpPerspective(TImg, hfinal, (RGBImg.shape[1], RGBImg.shape[0]))

    img_n = np.zeros((600, 800, 3), np.uint8)
    img_one = np.zeros((600, 800, 3), np.uint8)
    img_two = np.zeros((600, 800, 3), np.uint8)
    img_final = np.zeros((600, 800, 3), np.uint8)

    img_n[379:537, 4:605] = im_out75[379:537, 4:605]
    img_one[379:537, 4:605] = im_out75[379:537, 4:605]
    img_two[379:537, 4:605] = im_out75[379:537, 4:605]

    img_n[59:379, 4:605] = im_out150[59:379, 4:605]
    img_one[219:379, 4:605] = im_out150[219:379, 4:605]
    img_two[219:379, 4:605] = im_out150[219:379, 4:605]

    img_one[59:219, 4:605] = im_out150up[59:219, 4:605]
    img_two[59:219, 4:605] = im_outfinal[59:219, 4:605]

    img_final[59:537, 4:605] = im_outfinal[59:537, 4:605]

    im_color_n = cv2.applyColorMap(img_n, cv2.COLORMAP_JET)
    im_color_one = cv2.applyColorMap(img_one, cv2.COLORMAP_JET)
    im_color_two = cv2.applyColorMap(img_two, cv2.COLORMAP_JET)
    im_color_final = cv2.applyColorMap(img_final, cv2.COLORMAP_JET)

    img_add_n = cv2.addWeighted(RGBImg, alpha, im_color_n, beta, gamma)
    img_add_one = cv2.addWeighted(RGBImg, alpha, im_color_one, beta, gamma)
    img_add_two = cv2.addWeighted(RGBImg, alpha, im_color_two, beta, gamma)
    img_add_final = cv2.addWeighted(RGBImg, alpha, im_color_final, beta, gamma)

    return img_add_two

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
    # h75, h150, h75up, h150up, hfinal = h[0], h[1], h[2], h[3], h[4]
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue

        RGBImg = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
        TImg = cv2.resize(nowImg_TRM, size_TRM, cv2.INTER_CUBIC)
        dualImg = get_dual(RGBImg, TImg, h)
        img_message = bridge.cv2_to_imgmsg(dualImg)
        pub_dual.publish(img_message)
        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
