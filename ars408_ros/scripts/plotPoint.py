#! /usr/bin/env python2
# -*- coding: utf-8 -*-

from ars408_msg.msg import Tests, Test

import rospy
import math
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

width = 800
height = 600
limitDist = 260
camFOV = 42

x = []
y = []
pub = None


def callbackData(data):
    global x, y
    x = []
    y = []
    for i in data.tests:
        # if i.dynProp != 'stationary':
        x.append(i.x)
        y.append(i.y)

def callbackImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    dis = math.sqrt(x[it]**2 + y[it]**2)
    for it in range(min(len(x),len(y))):
        angle = math.atan2(y[it],x[it])/math.pi*180 # 算角度(度數)
        if abs(angle) < camFOV and dis < limitDist:
            plotX = int(width / 2.0 - width / 2.0 / camFOV * angle) # 以圖片中心點計算
            plotY = int(math.atan2(x[it], 3)/math.pi*180 * 300 / 90) # 將距離用actan轉成指數後擴增值域為0~300
            plotY = 300 if plotY > 300 else plotY
            plotY = 0 if plotY < 0 else plotY
            plotY = int(height-1 - plotY) # 把0~300轉成299~599
            cv2.circle(img, (plotX , plotY), 5, (0, 255, 0),-1, 4)
    
    img_message = bridge.cv2_to_imgmsg(img)
    pub.publish(img_message)
    # print(img.shape) # 800*600*3


def listener():
    global pub
    rospy.init_node("plotPoint", anonymous=False)
    sub1 = rospy.Subscriber("/testRects", Tests, callbackData)
    sub2 = rospy.Subscriber("/rgbImg", Image, callbackImg)
    pub = rospy.Publisher("/camDraw", Image, queue_size=100)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
