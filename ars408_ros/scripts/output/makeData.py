#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import os
import argparse

from sensor_msgs.msg import Image
from ars408_msg.msg import Tests, Test

from cv_bridge import CvBridge
import cv2

RadarPoints = []
idx = 0

class RadarPoint():
    def __init__(self, dynProp, x, y, RCS, VrelLong, VrelLat, ):
        self.dynProp = dynProp
        self.x = x
        self.y = y
        self.RCS = RCS
        self.VrelLong = VrelLong
        self.VrelLat = VrelLat
    
    def toString(self):
        return "{0}, {1}, {2}, {3}, {4}, {5}\r\n".format(self.dynProp, self.x, self.y, self.RCS, self.VrelLong, self.VrelLat)

def callbackData(data):
    global RadarPoints
    RadarPoints = []

    for i in data.tests:
        RadarPoints.append(RadarPoint(i.dynProp, i.x, i.y, i.RCS, i.VrelLong, i.VrelLat))

def callbackImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    global idx, RadarPoints
    root = os.getcwd()
    jpgPath = os.path.join(root, "{0:07}.jpg".format(idx))
    txtPath = os.path.join(root, "{0:07}.txt".format(idx))

    cv2.imwrite(jpgPath, img)
    with open(txtPath, "w") as f:
        for rp in RadarPoints:
            f.write(rp.toString())
        f.close()
    idx += 1

def listener():
    rospy.init_node("makeData")
    sub1 = rospy.Subscriber("/testRects", Tests, callbackData)
    sub2 = rospy.Subscriber("/rgbImg", Image, callbackImg)
    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Make pair img and radar point data.')
    parser.add_argument("-startIdx", default=0, type=int, help="Start index.")

    args = parser.parse_args()
    if args.startIdx:
        idx = args.startIdx

    try:
        listener()
    except rospy.ROSInternalException:
        pass
