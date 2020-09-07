#! /usr/bin/env python2
# coding=utf-8
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from ars408_msg.msg import RadarPoints, RadarPoint

import os
import argparse
import time

import cv2


frameRate = 20

topic_RGB = "/rgbImg"
topic_TRM = "/thermalImg"

size_RGB = (640, 480)
size_TRM = (640, 512)

global nowImg_RGB
global nowImg_TRM
global radarState

root = os.getcwd()
jpgRoot_RGB = os.path.join(root, "IMG_RGB_"+str(time.time()))
jpgRoot_TRM = os.path.join(root, "IMG_TRM_"+str(time.time()))
txtRoot_Data = os.path.join(root, "RadarP_"+str(time.time()))


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

def callback_Data(data):
    global radarState
    radarState.radarPoints = data.rps

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

def callback_speed(data):
    radarState.speed = data.data

def callback_zaxis(data):
    radarState.zaxis = data.data

def listener():
    global radarState
    rospy.init_node("toData")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)
    sub_Data = rospy.Subscriber("/radarPub", RadarPoints, callback_Data)
    sub_speed = rospy.Subscriber("/speed", Float32, callback_speed, queue_size=1)
    sub_zaxis = rospy.Subscriber("/zaxis", Float32, callback_zaxis, queue_size=1)
    radarState = RadarState()

    i = -10
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue

        if (i >= 0):
            txtPath = os.path.join(txtRoot_Data, "{0:07}.txt".format(i))
            cv2.imwrite(os.path.join(jpgRoot_RGB, "{0:07}.jpg".format(i)), nowImg_RGB)
            cv2.imwrite(os.path.join(jpgRoot_TRM, "{0:07}.jpg".format(i)), nowImg_TRM)

            with open(txtPath, "w") as f:
                f.write(radarState.toString())
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
