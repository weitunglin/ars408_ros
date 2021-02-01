#! /usr/bin/env python3
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
import yaml

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']

topic_RGB1 = "/rgbImg2"
topic_RGB2 = "/rgbImg4"
topic_RGB3 = "/rgbImg6"

size_RGB = config['size_RGB_Calib']

global nowImg_RGB1
global nowImg_RGB2
global nowImg_RGB3

root = os.getcwd()
aviPath_RGB1 = os.path.join(root, "RGB1_"+str(time.time())+".avi" )
aviPath_RGB2 = os.path.join(root, "RGB2_"+str(time.time())+".avi" )
aviPath_RGB3 = os.path.join(root, "RGB3_"+str(time.time())+".avi" )
savePic = False
jpgRoot_RGB1 = os.path.join(root, "IMG_RGB1_"+str(time.time()))
jpgRoot_RGB2 = os.path.join(root, "IMG_RGB2_"+str(time.time()))
jpgRoot_RGB3 = os.path.join(root, "IMG_RGB3_"+str(time.time()))
i_RGB1 = 0
i_RGB2 = 0
i_RGB3 = 0


def callback_RGBImg1(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB1
    nowImg_RGB1 = img.copy()

def callback_RGBImg2(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB2
    nowImg_RGB2 = img.copy()

def callback_RGBImg3(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB3
    nowImg_RGB3 = img.copy()


def listener():
    rospy.init_node("toVideo")
    rate = rospy.Rate(frameRate)
    sub_RGB1 = rospy.Subscriber(topic_RGB1, Image, callback_RGBImg1, queue_size=1)
    sub_RGB2 = rospy.Subscriber(topic_RGB2, Image, callback_RGBImg2, queue_size=1)
    sub_RGB3 = rospy.Subscriber(topic_RGB3, Image, callback_RGBImg3, queue_size=1)

    global out_RGB1, out_RGB2, out_RGB3
    out_RGB1 = cv2.VideoWriter(aviPath_RGB1, cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size_RGB, True)
    out_RGB2 = cv2.VideoWriter(aviPath_RGB2, cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size_RGB, True)
    out_RGB3 = cv2.VideoWriter(aviPath_RGB3, cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size_RGB, True)

    i = 0
    while not rospy.is_shutdown():
        if not ("nowImg_RGB1"  in globals() and "nowImg_RGB2"  in globals() and "nowImg_RGB3"  in globals()):
            continue

        out_RGB1.write(nowImg_RGB1)
        out_RGB2.write(nowImg_RGB2)
        out_RGB3.write(nowImg_RGB3)
        if savePic:
            cv2.imwrite(os.path.join(jpgRoot_RGB1, "RGB1_{0:07}.jpg".format(i)), nowImg_RGB1)
            cv2.imwrite(os.path.join(jpgRoot_RGB2, "RGB2_{0:07}.jpg".format(i)), nowImg_RGB2)
            cv2.imwrite(os.path.join(jpgRoot_RGB3, "RGB3_{0:07}.jpg".format(i)), nowImg_RGB3)
            i += 1

        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Export avi and jpg')
    parser.add_argument("-r", default=os.getcwd(), help="Root.")
    parser.add_argument("-o", help="Output prefix name.")
    parser.add_argument("-s", "--storePic", action="store_true", help="Store pic.")

    args = parser.parse_args()
    root = os.path.expanduser(args.r)
    try:
        os.makedirs(root)
    except Exception:
        pass

    if args.o:
        aviPath_RGB1 = os.path.join(root, "RGB1_"+args.o+".avi" )
        aviPath_RGB2 = os.path.join(root, "RGB2_"+args.o+".avi" )
        aviPath_RGB3 = os.path.join(root, "RGB3_"+args.o+".avi" )

        if os.path.exists(aviPath_RGB1):
            raise FileExistsError

        if args.storePic:
            jpgRoot_RGB1 = os.path.join(root, "IMG_RGB1_"+args.o)
            jpgRoot_RGB2 = os.path.join(root, "IMG_RGB2_"+args.o)
            jpgRoot_RGB3 = os.path.join(root, "IMG_RGB3_"+args.o)

    if args.storePic:
        savePic = True
        try:
            os.makedirs(jpgRoot_RGB1)
            os.makedirs(jpgRoot_RGB2)
            os.makedirs(jpgRoot_RGB3)
        except Exception:
            pass

    try:
        listener()
    except rospy.ROSInternalException:
        pass
    except rospy.ROSTimeMovedBackwardsException:
        print("End")
