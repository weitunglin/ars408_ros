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

# Decide the topic you want to output and plz care the corresponding size
# topic_RGB topic_RGB_Calib topic_TRM topic_Dual topic_yolo topic_RadarImg DistImg
outputMode = {
    'RGB':True,
    'RGB_Calib':True,
    'TRM':False,
    'Dual':False,
    'yolo':True,
    'RadarImg':True,
    'DistImg':True
}

topic = {
    'RGB':config['topic_RGB'], 
    'RGB_Calib':config['topic_RGB_Calib'],
    'TRM':config['topic_TRM'], 
    'Dual':config['topic_Dual'],
    'yolo':config['topic_yolo'], 
    'RadarImg':config['topic_RadarImg'],
    'DistImg':config['topic_DistImg']
}

size = {
    'RGB':config['size_RGB_720p'],
    'RGB_Calib':config['size_RGB_Calib'],
    'TRM':config['size_TRM'],
    'Dual':config['size_Dual'],
    'yolo':config['size_RGB_720p'],
    'RadarImg':config['size_RGB_720p'],
    'DistImg':config['size_RGB_720p']
}

global nowImg
nowImg = {
    'RGB':None,
    'RGB_Calib':None,
    'TRM':None,
    'Dual':None,
    'yolo':None,
    'RadarImg':None,
    'DistImg':None
}

# crop
# ROI = config['ROI']
# crop_x = (ROI[1][0], ROI[1][1])
# crop_y = (ROI[0][0], ROI[0][1])

root = os.getcwd()
root += '/toVideo'

aviPath = {
    'RGB': os.path.join(root, "RGB_"+str(time.time())+".avi" ),
    'RGB_Calib': os.path.join(root, "RGB_Calib"+str(time.time())+".avi" ),
    'TRM': os.path.join(root, "TRM_"+str(time.time())+".avi" ),
    'Dual': os.path.join(root, "Dual_"+str(time.time())+".avi" ),
    'yolo': os.path.join(root, "yolo_"+str(time.time())+".avi" ),
    'RadarImg': os.path.join(root, "RadarImg_"+str(time.time())+".avi" ),
    'DistImg': os.path.join(root, "DistImg_"+str(time.time())+".avi" )
}

savePic = False
jpgPath = {
    'RGB': os.path.join(root, "IMG_RGB_"+str(time.time())),
    'RGB_Calib': os.path.join(root, "IMG_RGB_Calib"+str(time.time())),
    'TRM': os.path.join(root, "IMG_TRM_"+str(time.time())),
    'Dual': os.path.join(root, "IMG_Dual_"+str(time.time())),
    'yolo': os.path.join(root, "IMG_yolo_"+str(time.time())),
    'RadarImg': os.path.join(root, "IMG_RadarImg_"+str(time.time())),
    'DistImg': os.path.join(root, "IMG_DistImg_"+str(time.time()))
}

jpgCount = {
    'RGB':0,
    'RGB_Calib':0,
    'TRM':0,
    'Dual':0,
    'yolo':0,
    'RadarImg':0,
    'DistImg':0
}


def callback_Img(data, arg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg
    nowImg[arg] = img.copy()

def listener():
    rospy.init_node("toVideo")
    rate = rospy.Rate(frameRate)
    rospy.Subscriber(topic['RGB'], Image, callback_Img, 'RGB', queue_size=1)
    rospy.Subscriber(topic['RGB_Calib'], Image, callback_Img, 'RGB_Calib', queue_size=1)
    rospy.Subscriber(topic['TRM'], Image, callback_Img, 'TRM', queue_size=1)
    rospy.Subscriber(topic['Dual'], Image, callback_Img, 'Dual', queue_size=1)
    rospy.Subscriber(topic['yolo'], Image, callback_Img, 'yolo', queue_size=1)
    rospy.Subscriber(topic['RadarImg'], Image, callback_Img, 'RadarImg', queue_size=1)
    rospy.Subscriber(topic['DistImg'], Image, callback_Img, 'DistImg', queue_size=1)

    global outputVideo
    outputVideo = {
        'RGB':None,
        'RGB_Calib':None,
        'TRM':None,
        'Dual':None,
        'yolo':None,
        'RadarImg':None,
        'DistImg':None
    }
    for key in outputMode:
        if outputMode[key]:
            outputVideo[key] = cv2.VideoWriter(aviPath[key], cv2.VideoWriter_fourcc(*'DIVX'), frameRate, size[key], True)

    while not rospy.is_shutdown():
        for key in outputMode:
            if outputMode[key] == False or type(nowImg[key]) == type(None):
                continue
            # print(key, nowImg[key].shape, size[key])
            outputVideo[key].write(nowImg[key])
            if savePic:
                cv2.imwrite(os.path.join(jpgPath[key], "{0:07}.jpg".format(jpgCount[key])), nowImg[key])
                jpgCount[key] += 1
        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Export avi and jpg')
    parser.add_argument("-r", default=os.getcwd() + '/toVideo', help="Root.")
    parser.add_argument("-o", help="Output prefix name.")
    parser.add_argument("-s", "--storePic", action="store_true", help="Store pic.")

    print("start")
    args = parser.parse_args()
    root = os.path.expanduser(args.r)
    try:
        os.makedirs(root)
    except Exception:
        pass

    if args.o:
        for key in aviPath:
            aviPath[key] = os.path.join(root,  key + "_" + args.o + ".avi" )

        if os.path.exists(aviPath['RGB']):
            raise FileExistsError

        if args.storePic:
            for key in outputMode:
                if outputMode[key]:
                    jpgPath[key] = os.path.join(root,  "IMG_" + key + "_" + args.o)

    if args.storePic:
        savePic = True
        try:
            for key in outputMode:
                if outputMode[key]:
                    os.makedirs(jpgPath[key])
        except Exception:
            pass

    try:
        listener()
    except rospy.ROSInternalException:
        pass
    except rospy.ROSTimeMovedBackwardsException:
        print("End")
