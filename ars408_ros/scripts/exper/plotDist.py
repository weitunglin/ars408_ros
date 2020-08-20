#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from ars408_msg.msg import RadarPoints, RadarPoint
from ars408_msg.msg import Bboxes, Bbox

import math

import cv2


# 內部參數
img_width = 800
img_height = 600
fov_width = 42.3
fov_height = 54.8

# 外部參數
img2Radar_x = 210        # 影像到雷達的距離 (cm)
img2Radar_y = 10         # 影像到雷達的距離 (cm)
img2Radar_z = 180        # 影像到地板的距離 (cm)
img2ground = 3           # 影像照到地板的距離 (m)

global radarState, pub1, pub2, img_xy, myBB

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

class BoundingBox():
    def __init__(self):
        self.bboxes = []

def drawRadar2Img(img, radarState, bboxes):
    global img_xy
    img_xy = []
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

            yValue = i.distX * math.tan(fov_height / 2 / 180 * math.pi)
            yValue = max(yValue, 1)
            yValueRatio = img2ground * math.tan(fov_height / 2 / 180 * math.pi) / yValue
            yValueRatio = min(yValueRatio, 1)
            # plotY = int(img_height/2-1 + yValueRatio * 300)  

            # 畫圖
            cirSize = int(max(10 - dist // 10, 1))

            plotColor = (0, 255, 0)
            # TODO: 碰撞偵測
            # if i.vrelX < 0:
            #     if i.distX / (-i.vrelX) < 4.0:
            #         plotColor = (0, 0, 255)
            cv2.circle(img, (plotX , plotY), cirSize, plotColor,-1, 4)
            # cv2.rectangle(img, (200, 200), (400, 400), (0, 255, 0), 2)
            # cv2.putText(img, str(dist), (plotX - 5, plotY - 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)
            img_xy.append((plotX , plotY, dist))

    for i in bboxes.bboxes:
        rectColor = (0, 255, 255)
        leftTop = (i.x_min, i.y_min)
        rightBut = (i.x_max, i.y_max)
        cv2.rectangle(img, leftTop, rightBut, (0, 255, 0), 2)

    return img

def drawBbox2Img(img, bboxes):
    global img_xy
    for i in bboxes.bboxes:
        rectColor = (0, 255, 255)
        leftTop = (i.x_min, i.y_min)
        rightBut = (i.x_max, i.y_max)
        cv2.rectangle(img, leftTop, rightBut, (0, 255, 0), 2)

        minDist = 99999
        for xy in img_xy:
            if xy[0] > leftTop[0] and xy[0]< rightBut[0] and xy[1] > leftTop[1] and xy[1]< rightBut[1]:
                if xy[2] < minDist:
                    minDist = xy[2]

        showText = "Dis: Null"
        if minDist != 99999:
            showText = "Dis: {0:0.2f}".format(minDist)

        cv2.putText(img, showText, (leftTop[0] - 5, leftTop[1] - 5), cv2.FONT_HERSHEY_PLAIN, 1.5, rectColor, 1, cv2.LINE_AA)        

    return img

def callback_Data(data):
    global radarState
    radarState.radarPoints = data.rps

def callback_Bbox(data):
    global myBB
    myBB.bboxes = data.bboxes

def callback_Img(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img_draw = drawRadar2Img(img.copy(), radarState, myBB)
    img_Dist = drawBbox2Img(img.copy(), myBB)
    img_message = bridge.cv2_to_imgmsg(img_draw)
    img_message2 = bridge.cv2_to_imgmsg(img_Dist)
    pub1.publish(img_message)
    pub2.publish(img_message2)

def listener():
    global pub1, pub2, radarState, myBB
    rospy.init_node("plotPoint")
    sub1 = rospy.Subscriber("/radarPub", RadarPoints, callback_Data)
    sub2 = rospy.Subscriber("/dualImg", Image, callback_Img)
    sub3 = rospy.Subscriber("/Bbox", Bboxes, callback_Bbox)
    pub1 = rospy.Publisher("/drawImg", Image, queue_size=1)
    pub2 = rospy.Publisher("/DistImg", Image, queue_size=1)
    radarState = RadarState()
    myBB = BoundingBox()
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
