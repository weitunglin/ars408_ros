#! /usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ars408_msg.msg import Tests, Test
from cv_bridge import CvBridge
import cv2
import os
import argparse
import time

frameRate = 20

topic_RGB = "/rgbImg"
topic_TRM = "/thermalImg"

size_RGB = (800, 600)
size_TRM = (640, 512)

global nowImg_RGB
global nowImg_TRM
RadarPoints = []

root = os.getcwd()
jpgRoot_RGB = os.path.join(root, "IMG_RGB_"+str(time.time()))
jpgRoot_TRM = os.path.join(root, "IMG_TRM_"+str(time.time()))
txtRoot_Data = os.path.join(root, "RadarP_"+str(time.time()))


class RadarPoint():
    def __init__(self, dynProp, x, y, RCS, VrelLong, VrelLat):
        self.dynProp = dynProp
        self.x = x
        self.y = y
        self.RCS = RCS
        self.VrelLong = VrelLong
        self.VrelLat = VrelLat
    
    def toString(self):
        return "{0}, {1}, {2}, {3}, {4}, {5}\r\n".format(self.dynProp, self.x, self.y, self.RCS, self.VrelLong, self.VrelLat)

def callback_Data(data):
    global RadarPoints
    RadarPoints = []

    for i in data.tests:
        RadarPoints.append(RadarPoint(i.dynProp, i.x, i.y, i.RCS, i.VrelLong, i.VrelLat))

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
    rospy.init_node("toVideo")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)
    sub_Data = rospy.Subscriber("/testRects", Tests, callback_Data)

    i = 0
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue

        txtPath = os.path.join(txtRoot_Data, "{0:07}.txt".format(i))
        cv2.imwrite(os.path.join(jpgRoot_RGB, "{0:07}.jpg".format(i)), nowImg_RGB)
        cv2.imwrite(os.path.join(jpgRoot_TRM, "{0:07}.jpg".format(i)), nowImg_TRM)
        
        with open(txtPath, "w") as f:
            for rp in RadarPoints:
                f.write(rp.toString())
            f.close()
        i += 1

        rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Export avi and jpg')
    parser.add_argument("-r", default=os.getcwd(), help="Root.")
    parser.add_argument("-o", help="Output prefix name.")

    args = parser.parse_args()
    root = os.path.expanduser(args.r)
    try:
        os.makedirs(root)
    except Exception:
        pass

    if args.o:
        jpgRoot_RGB = os.path.join(root, "IMG_RGB_"+args.o)
        jpgRoot_TRM = os.path.join(root, "IMG_TRM_"+args.o)
        txtRoot_Data = os.path.join(root, "RadarP_"+args.o)

    try:
        os.makedirs(jpgRoot_RGB)
        os.makedirs(jpgRoot_TRM)
        os.makedirs(txtRoot_Data)
    except Exception:
        pass

    try:
        listener()
    except rospy.ROSInternalException:
        pass
