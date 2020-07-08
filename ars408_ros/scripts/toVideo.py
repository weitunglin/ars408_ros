#! /usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
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

root = os.getcwd()
aviPath_RGB = os.path.join(root, "RGB_"+str(time.time())+".avi" )
aviPath_TRM = os.path.join(root, "TRM_"+str(time.time())+".avi" )
savePic = False
jpgRoot_RGB = os.path.join(root, "IMG_RGB_"+str(time.time()))
jpgRoot_TRM = os.path.join(root, "IMG_TRM_"+str(time.time()))
i_RGB = 0
i_TRM = 0

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
    rospy.init_node("toVideo", anonymous=False)
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)

    global out_RGB, out_TRM
    out_RGB = cv2.VideoWriter(aviPath_RGB, cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size_RGB, True)
    out_TRM = cv2.VideoWriter(aviPath_TRM, cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size_TRM, True)

    i = 0
    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
            continue
    
        out_RGB.write(nowImg_RGB)
        out_TRM.write(nowImg_TRM)
        if savePic:
            cv2.imwrite(os.path.join(jpgRoot_RGB, "{0:07}.jpg".format(i)), nowImg_RGB)
            cv2.imwrite(os.path.join(jpgRoot_TRM, "{0:07}.jpg".format(i)), nowImg_TRM)
            i += 1

        rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Export avi and jpg')
    parser.add_argument("-r", default=os.getcwd(), help="Root.")
    parser.add_argument("-o", help="Output prefix name.")
    parser.add_argument("-s", "--storePic", action="store_true", help="Store pic.")

    args = parser.parse_args()
    root = os.path.expanduser(args.r)

    if args.o:
        aviPath_RGB = os.path.join(root, "RGB_"+args.o+".avi" )
        aviPath_TRM = os.path.join(root, "TRM_"+args.o+".avi" )

        if args.storePic:
            jpgRoot_RGB = os.path.join(root, "IMG_RGB_"+args.o)
            jpgRoot_TRM = os.path.join(root, "IMG_TRM_"+args.o)

    if args.storePic:
        savePic = True
        try:
            os.makedirs(jpgRoot_RGB)
            os.makedirs(jpgRoot_TRM)
        except Exception:
            pass

    try:
        listener()
    except rospy.ROSInternalException:
        pass
