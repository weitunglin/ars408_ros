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

from PIL import Image as Img
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import Bboxes, Bbox
import rospy
import os, sys
import math
import cv2
import time
import numpy as np
import tensorflow as tf

# sys.path.insert(1, "/home/balin/Documents/tensorflow-yolov3")
# sys.path.append("/home/balin/Documents/tensorflow-yolov3")
sys.path.append("/home/balin/Documents/yolov3")
os.chdir("/home/balin/Documents/yolov3")
import core.utils as utils


frameRate = 20
topic_RGB = "/rgbImg"
topic_TRM = "/thermalImg"

size_RGB = (640, 480)
size_TRM = (640, 512)

global nowImg_RGB
global nowImg_TRM

class BoundingBox():
    def __init__(self):
        self.BoundingBoxes = []

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
    rospy.init_node("yolo")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg, queue_size=1)
    pub_yolo = rospy.Publisher("/yoloImg", Image, queue_size=1)
    pub_bbox = rospy.Publisher("/Bbox", Bboxes, queue_size=1)
    bridge = CvBridge()

    return_elements = ["input/input_rgb:0","input/input_lwir:0", "pred_sbbox/concat_2:0", "pred_mbbox/concat_2:0", "pred_lbbox/concat_2:0",\
                    "pred_sbbox_rgb/concat_2:0", "pred_mbbox_rgb/concat_2:0", "pred_lbbox_rgb/concat_2:0",\
                    "pred_sbbox_lwir/concat_2:0", "pred_mbbox_lwir/concat_2:0", "pred_lbbox_lwir/concat_2:0"] #定義model返回參數名稱
    pb_file         = "./RGB_graph.pb"
    num_classes     = 8
    input_size      = 416
    graph           = tf.Graph()
    return_tensors  = utils.read_pb_return_tensors(graph, pb_file, return_elements)

    with tf.Session(graph=graph) as sess:
        while not rospy.is_shutdown():
            if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
                continue
            RGBImg = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
            TImg = cv2.resize(nowImg_TRM, size_TRM, cv2.INTER_CUBIC)

            image_rgb = Img.fromarray(RGBImg)
            image_lwir  = Img.fromarray(TImg)            

            frame_size = TImg.shape[:2]
            image_rgb,image_lwir = utils.image_preporcess(np.copy(RGBImg),np.copy(TImg), [input_size, input_size])
            image_rgb = image_rgb[np.newaxis, ...]
            image_lwir = image_lwir[np.newaxis, ...]
            
            prev_time = time.time()

            pred_sbbox, pred_mbbox, pred_lbbox,pred_sbbox_rgb, pred_mbbox_rgb, pred_lbbox_rgb,pred_sbbox_lwir, pred_mbbox_lwir, pred_lbbox_lwir = sess.run(
                    [return_tensors[2], return_tensors[3], return_tensors[4],return_tensors[5], return_tensors[6], return_tensors[7],return_tensors[8], return_tensors[9], return_tensors[10]],
                        feed_dict={ return_tensors[0]: image_rgb,return_tensors[1]: image_lwir})#運行模組進行推論
            
            #sess.run後得到三組參數
            #(pred_sbbox,pred_sbbox_rgb,pred_sbbox_lwir)
            #(pred_mbbox,pred_mbbox_rgb,pred_mbbox_lwir)
            #(pred_lbbox,pred_lbbox_rgb,pred_lbbox_lwir)
            #pred_bbox同樣格式，可以選擇使用上述1~9種的輸出同時進行討論(自己選擇)
            #                             np.reshape(pred_lbbox_lwir, (-1, 5 + self.num_classes))],axis=0)
            # pred_bbox_lwir = np.concatenate([np.reshape(pred_sbbox_lwir, (-1, 5 + num_classes)),
            #                             np.reshape(pred_mbbox_lwir, (-1, 5 + num_classes)),
            #                             np.reshape(pred_lbbox_lwir, (-1, 5 + num_classes))],axis=0) 
            pred_bbox_lwir = np.concatenate([np.reshape(pred_sbbox, (-1, 5 + num_classes)),
                                        np.reshape(pred_mbbox, (-1, 5 + num_classes)),
                                        np.reshape(pred_lbbox, (-1, 5 + num_classes))],axis=0) 
            
            
            bboxes_lwir = utils.postprocess_boxes(pred_bbox_lwir, frame_size, input_size, 0.5)
            
            
            bboxes_lwir = utils.nms(bboxes_lwir, 0.45, method='nms')
            
            image = utils.draw_bbox(TImg, bboxes_lwir)

            result = np.asarray(image)
            print(bboxes_lwir)
            
            temp = np.array(bboxes_lwir)
            BB = Bboxes()
            for index, ele in enumerate(temp):
                tempBB = Bbox()
                tempBB.x_min = int(temp[index,0])
                tempBB.y_min = int(temp[index,1])
                tempBB.x_max = int(temp[index,2])
                tempBB.y_max = int(temp[index,3])
                BB.bboxes.append(tempBB)

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            img_message = bridge.cv2_to_imgmsg(result)
            pub_yolo.publish(img_message)
            pub_bbox.publish(BB)
            curr_time = time.time()
            exec_time = curr_time - prev_time
            info = "time:" + str(round(1000 * exec_time, 2)) + " ms, FPS: " + str(round((1000 / (1000 * exec_time)), 1))
            print(info)
            rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass