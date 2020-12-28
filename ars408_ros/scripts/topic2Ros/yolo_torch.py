#! /usr/bin/env python3
# coding=utf-8
#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#
#   Editor      : VIM
#   File name   : video_demo.py
#   Author      : YunYang1994
#   Created date: 2018-11-30 15:56:37
#   Description :
#
#================================================================

from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import Bboxes, Bbox
import rospy
import os, sys
import cv2

sys.path.append(os.path.expanduser("~") + "/Documents/yolov4_torch")
os.chdir(os.path.expanduser("~") + "/Documents/yolov4_torch")
from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet
import argparse
import yaml

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']

topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']
topic_yolo = config['topic_yolo']
topic_Bbox = config['topic_Bbox']

size_RGB = config['size_RGB_Calib']
size_TRM = config['size_TRM']
size_Dual = config['size_Dual']

ROI = config['ROI']
crop_x = (ROI[1][0], ROI[1][1])
crop_y = (ROI[0][0], ROI[0][1])

global nowImg_RGB
global nowImg_TRM
global nowImg_FUS

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    nowImg_RGB = img.copy()

def callback_TRMImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_TRM
    nowImg_TRM = img.copy()

def callback_FUSImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_FUS
    nowImg_FUS = img.copy()

def listener():
    rospy.init_node("yolo")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)
    sub_FUS = rospy.Subscriber(topic_Dual, Image, callback_FUSImg, queue_size=1)
    pub_yolo = rospy.Publisher(topic_yolo, Image, queue_size=1)
    pub_bbox = rospy.Publisher(topic_Bbox, Bboxes, queue_size=1)
    bridge = CvBridge()

    """hyper parameters"""
    use_cuda = True

    cfgfile_RGB = config['path_RGB_cfg']
    weightfile_RGB = config['path_RGB_weights']
    cfgfile_TRM = config['path_TRM_cfg']
    weightfile_TRM = config['path_TRM_weights']

    RGB = Darknet(cfgfile_RGB)
    RGB.print_network()
    RGB.load_weights(weightfile_RGB)
    print('Loading RGB weights from %s... Done!' % (weightfile_RGB))

    if use_cuda:
        RGB.cuda()
    num_classes = RGB.num_classes
    if num_classes == 20:
        namesfile = 'data/voc.names'
    elif num_classes == 80:
        namesfile = 'data/coco.names'
    else:
        namesfile = config['path_RGB_names']
    class_names = load_class_names(namesfile)


    TRM = Darknet(cfgfile_TRM)
    TRM.print_network()
    TRM.load_weights(weightfile_TRM)
    print('Loading TRM weights from %s... Done!' % (weightfile_TRM))
    if use_cuda:
        TRM.cuda()
    num_classes = TRM.num_classes
    if num_classes == 20:
        namesfile = 'data/voc.names'
    elif num_classes == 80:
        namesfile = 'data/coco.names'
    else:
        namesfile = config['path_TRM_names']
    class_names = load_class_names(namesfile)

    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals() and "nowImg_FUS"  in globals()):
            continue
        # t1 = time.time()
        img_FUS = nowImg_FUS[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
        img_FUS = cv2.resize(img_FUS , size_Dual)
        sized_RGB = cv2.resize(nowImg_RGB, (RGB.width, RGB.height))
        sized_RGB = cv2.cvtColor(sized_RGB, cv2.COLOR_BGR2RGB)
        sized_TRM = cv2.resize(nowImg_TRM , (TRM.width, TRM.height))
        sized_TRM = cv2.cvtColor(sized_TRM, cv2.COLOR_BGR2RGB)
        boxes_fusion = do_detect_ye(TRM, RGB, sized_TRM, sized_RGB, 0.25, 0.4, use_cuda)
        result_fusion = draw_bbox(img_FUS, boxes_fusion[0], class_names=class_names, show_label=True)

        # t2 = time.time()
        # print('-----------------------------------')
        # print('       FPS : %f' % (1 / (t2 - t1)))
        # print('-----------------------------------')

        """
        bboxes: [x_min, y_min, x_max, y_max, probability, cls_id] format coordinates.
        """
        BB = Bboxes()
        for index, bbox in enumerate(boxes_fusion[0]):
            tempBB = Bbox()
            tempBB.x_min = bbox[0]
            tempBB.y_min = bbox[1]
            tempBB.x_max = bbox[2]
            tempBB.y_max = bbox[3]
            tempBB.score = bbox[4]
            tempBB.objClass = class_names[bbox[6]]
            BB.bboxes.append(tempBB)

        pub_bbox.publish(BB)
        result = cv2.cvtColor(result_fusion, cv2.COLOR_RGB2BGR)
        img_message = bridge.cv2_to_imgmsg(result_fusion)
        pub_yolo.publish(img_message)
        rate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass