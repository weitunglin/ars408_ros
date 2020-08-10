#! /usr/bin/env python2
# -*- coding: utf-8 -*-

from ars408_msg.msg import RadarPoints, RadarPoint

import rospy
import math
from sensor_msgs.msg import Image

import cv2
from cv_bridge.core import CvBridge

# 內部參數
img_width = 800
img_height = 600
fov_width = 42.3
fov_height = 54.8

# 外部參數
img2Radar_x = 180        # 影像到雷達的距離 (cm)
img2Radar_y = 10         # 影像到雷達的距離 (cm)


global radarState, pub1

class RadarState():
    def __init__(self):
        self.radarPoints = []
        self.speed = 0
        self.zaxis = 0

    def toString(self):
        s = "{0}, {1}\r\n".format(
            self.speed, 
            self.zaxis
        )
        
        for i in self.radarPoints:
            s += "{0:3d}, {1}, {2:>9.3f}, {3:>9.3f}, {4:>9.3f}, {5:>9.3f}, {6:>9.3f}, {7:>9.3f}, {8:>9.3f}\r\n".format(
                i.id,
                i.dynProp,
                i.distX,
                i.distY,
                i.vrelX,
                i.vrelY,
                i.rcs,
                i.width,
                i.height
            )
        return s

def drawRadar2Img(img, radarState):
    for i in radarState.radarPoints:
        dist = math.sqrt(i.distX**2 + i.distY**2)                   # 算距離 (m)
        angle = math.atan2(i.distY, i.distX) / math.pi*180          # 算角度 (度數)
        vRel = math.sqrt(i.vrelX**2 + i.vrelY**2)                   # 算速度 (m/s)

        if abs(angle) < fov_width:
            # 算圖片 X (橫向)
            plotX = int(img_width / 2.0 - img_width / 2.0 / fov_width * angle)  # 以圖片中心點計算
            # 算圖片 Y (縱向)
            plotY = int(math.atan2(i.distX, 3) / math.pi * 180 * 300 / 90)      # 將距離用actan轉成指數後擴增值域為0~300
            plotY = 300 if plotY > 300 else plotY
            plotY = 0 if plotY < 0 else plotY
            plotY = int(img_height-1 - plotY)                                   # 把0~300轉成299~599
            # 畫圖
            cirSize = int(max(10 - dist // 10, 1))

            plotColor = (0, 255, 0)
            # TODO: 碰撞偵測
            # if i.vrelX < 0:
            #     if i.distX / (-i.vrelX) < 4.0:
            #         plotColor = (0, 0, 255)
            cv2.circle(img, (plotX , plotY), cirSize, plotColor,-1, 4)

    return img    

def callback_Data(data):
    global radarState
    radarState.radarPoints = data.rps

def callback_Img(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img_draw = drawRadar2Img(img, radarState)
    img_message = bridge.cv2_to_imgmsg(img_draw)
    pub1.publish(img_message)

def listener():
    global pub1, radarState
    rospy.init_node("plotPoint")
    sub1 = rospy.Subscriber("/radarPub", RadarPoints, callback_Data)
    sub2 = rospy.Subscriber("/rgbImg", Image, callback_Img)
    pub1 = rospy.Publisher("/drawImg", Image, queue_size=100)
    radarState = RadarState()
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
