#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2
import numpy as np

frameRate = 20
topic_RGB = "/rgbImg"
size_RGB = (640, 480)
cmatrix = np.array([443.84389,0.0,300.09341,0.0,448.89312,259.5822,0.0,0.0,1.0])
dmatrix = np.array([-0.364865, 0.111417, -0.003288, 0.001529, 0.000000])
cmatrix = cmatrix.reshape(3,3)
dmatrix = dmatrix.reshape(1,5)
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
    pub_RGB = rospy.Publisher("/calibImg", Image, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
