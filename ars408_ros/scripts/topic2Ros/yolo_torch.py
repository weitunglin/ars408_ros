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

frameRate = 20
topic_RGB = "/calibImg"
topic_TRM = "/thermalImg"
topic_FUS = "/dualImg"

size_RGB = (640, 480)
size_TRM = (640, 512)
size_FUS = (640, 480)

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
    sub_FUS = rospy.Subscriber(topic_FUS, Image, callback_FUSImg, queue_size=1)
    pub_yolo = rospy.Publisher("/yoloImg", Image, queue_size=1)
    pub_bbox = rospy.Publisher("/Bbox", Bboxes, queue_size=1)
    bridge = CvBridge()

    """hyper parameters"""
    use_cuda = True

    cfgfile_RGB='./cfg/yolo-1213_obj.cfg'
    weightfile_RGB='./weight/yolo-RGB_new.weights'
    cfgfile_TRM='./cfg/yolo-1213_obj.cfg'
    weightfile_TRM='./weight/yolo-Thermal_new.weights'

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
        namesfile = 'data/1104_obj.names'
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
        namesfile = 'data/1104_obj.names'
    class_names = load_class_names(namesfile)

    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals() and "nowImg_FUS"  in globals()):
            continue
        # t1 = time.time()
        img_FUS = nowImg_FUS[141:475, 164:550]
        img_FUS = cv2.resize(img_FUS , (640, 480))
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
        image_h, image_w, _ = img_FUS.shape
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