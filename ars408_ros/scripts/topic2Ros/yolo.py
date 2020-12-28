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

sys.path.append(os.path.expanduser("~") + "/Documents/yolov3fusion1")
os.chdir(os.path.expanduser("~") + "/Documents/yolov3fusion1")
import core.utils as utils
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

    return_elements = ["input/input_rgb:0","input/input_lwir:0", "pred_sbbox/concat_2:0", "pred_mbbox/concat_2:0", "pred_lbbox/concat_2:0"]
    pb_file         = "./yolo_fusion_0910.pb"
    num_classes     = 8
    input_size      = 416
    graph           = tf.Graph()
    return_tensors  = utils.read_pb_return_tensors(graph, pb_file, return_elements)

    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.9
    with tf.Session(graph=graph, config=config) as sess:
    # with tf.Session(graph=graph) as sess:
        while not rospy.is_shutdown():
            if not ("nowImg_RGB"  in globals() and "nowImg_TRM"  in globals()):
                continue
            frame_rgb = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
            frame_lwir = cv2.resize(nowImg_TRM, size_TRM, cv2.INTER_CUBIC)
            # frame_fusion = cv2.resize(nowImg_FUS, size_Dual, cv2.INTER_CUBIC)

            image_rgb = Img.fromarray(frame_rgb)
            image_lwir  = Img.fromarray(frame_lwir)            

            frame_size = frame_lwir.shape[:2]
            image_rgb,image_lwir = utils.image_preporcess(np.copy(frame_rgb),np.copy(frame_lwir), [input_size, input_size])
            image_rgb = image_rgb[np.newaxis, ...]
            image_lwir = image_lwir[np.newaxis, ...]
            
            prev_time = time.time()

            pred_sbbox, pred_mbbox, pred_lbbox = sess.run(
                [return_tensors[2], return_tensors[3], return_tensors[4]],
                    feed_dict={ return_tensors[0]: image_rgb,return_tensors[1]: image_lwir})
                    
            pred_bbox = np.concatenate([np.reshape(pred_sbbox, (-1, 5 + num_classes)),
                                        np.reshape(pred_mbbox, (-1, 5 + num_classes)),
                                        np.reshape(pred_lbbox, (-1, 5 + num_classes))], axis=0)
            
            bboxes = utils.postprocess_boxes(pred_bbox, frame_size, input_size, 0.5)
            bboxes = utils.nms(bboxes, 0.45, method='nms')
            image = utils.draw_bbox(frame_rgb, bboxes)
            result = np.asarray(image)

            curr_time = time.time()
            exec_time = curr_time - prev_time
            info = "time:" + str(round(1000 * exec_time, 2)) + " ms, FPS: " + str(round((1000 / (1000 * exec_time)), 1))
            print(info)

            temp = np.array(bboxes)
            BB = Bboxes()
            yoloClasses = utils.read_class_names("./data/classes/obj.names")
            for index, ele in enumerate(temp):
                tempBB = Bbox()
                tempBB.x_min = int(temp[index,0])
                tempBB.y_min = int(temp[index,1])
                tempBB.x_max = int(temp[index,2])
                tempBB.y_max = int(temp[index,3])
                tempBB.score = temp[index,4]
                tempBB.objClass = yoloClasses[int(temp[index,5])]
                # tempBB.id = int(temp[index,6])
                BB.bboxes.append(tempBB)

            pub_bbox.publish(BB)
            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            img_message = bridge.cv2_to_imgmsg(result)
            pub_yolo.publish(img_message)
            rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass