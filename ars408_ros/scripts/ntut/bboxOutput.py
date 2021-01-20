#! /usr/bin/env python3
# coding=utf-8

from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import Bboxes, Bbox
from ars408_msg.msg import RadarPoints, RadarPoint
import matplotlib.pyplot as plt
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


# topic_RGB = config['topic_RGB']
topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']
topic_yolo = config['topic_yolo']
topic_Bbox = config['topic_Bbox']
topic_Radar = config['topic_Radar']
topic_RadarImg = config['topic_RadarImg']
topic_DistImg = config['topic_DistImg']

size_RGB = config['size_RGB_Calib']
size_TRM = config['size_TRM']
size_Dual = config['size_Dual']

ROI = config['ROI']
crop_x = (ROI[1][0], ROI[1][1])
crop_y = (ROI[0][0], ROI[0][1])

# 內部參數
# size_RGB = (640 * pixelTime, 480 * pixelTime)  
img_width = config['size_RGB_Calib_output'][0]
img_height = config['size_RGB_Calib_output'][1]
pixelTime = img_width / config['size_RGB_Calib'][0]
textTime = config['textTime']
scoreScale = math.sqrt(config['size_RGB_Calib_output'][0] ** 2 + config['size_RGB_Calib_output'][1] ** 2)

cmatrix = np.array(config['K']).reshape(3,3)
dmatrix = np.array(config['D']).reshape(1,5)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cmatrix, dmatrix, size_RGB, 1, size_RGB)

global nowImg_RGB
global nowImg_TRM
global nowImg_FUS
global pub1, pub2, myPoints, count, module
count = 1
module = 20

"""hyper parameters"""
use_cuda = True
filename = "costco1"

cfgfile_RGB = config['path_RGB_cfg']
weightfile_RGB = config['path_RGB_weights']
cfgfile_TRM = config['path_TRM_cfg']
weightfile_TRM = config['path_TRM_weights']

imgfile_RGB='/home/balin/20210110北科/RGB_' + filename + '.avi'

import cv2

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


RGB_test = cv2.VideoCapture(imgfile_RGB)
T_test = cv2.VideoCapture(imgfile_RGB)
fusion_test = cv2.VideoCapture(imgfile_RGB)

count = 0
while RGB_test.isOpened():
    t1 = time.time()
    RGB_ret, RGB_frame = RGB_test.read()
    T_ret, T_frame = T_test.read()
    f_ret, f_frame = fusion_test.read() 
    if not RGB_ret or not T_ret or not f_ret:
        print('no video')
        break
    print("1",RGB_frame.shape,T_frame.shape,f_frame.shape)
    sized_RGB = cv2.resize(RGB_frame, (RGB.width, RGB.height))
    sized_RGB = cv2.cvtColor(sized_RGB, cv2.COLOR_BGR2RGB)
    sized_TRM = cv2.resize(T_frame , (TRM.width, TRM.height))
    sized_TRM = cv2.cvtColor(sized_TRM, cv2.COLOR_BGR2RGB)
    boxes_fusion = do_detect_ye(TRM, RGB, sized_TRM, sized_RGB, 0.25, 0.4, use_cuda)
    f = open(os.path.join('/home/balin/Downloads/北科/costco_output_label/', filename + "_{0:07}.txt".format(count)), 'w')
    for index, bbox in enumerate(boxes_fusion[0]):
        f.write(str(bbox[6]) + " " + "{:.6f}".format((bbox[2] + bbox[0]) / 2) + " " + "{:.6f}".format((bbox[3] + bbox[1]) / 2) + " " + "{:.6f}".format((bbox[2] - bbox[0])) + " " + "{:.6f}".format((bbox[3] - bbox[1])) + " " + "{:.6f}".format(bbox[4]) + "\n")

    result_fusion = draw_bbox(f_frame, boxes_fusion[0], class_names=class_names, show_label=True)
    print("2",sized_RGB.shape,sized_TRM.shape,result_fusion.shape)
    t2 = time.time()
    print('-----------------------------------')
    print('       max and argmax : %f' % (1 / (t2 - t1)))
    print('-----------------------------------')
    # cv2.imshow('Yolo demo', result_fusion)
    cv2.imwrite(os.path.join('/home/balin/Downloads/北科/costco_output_img/', filename + "_{0:07}.jpg".format(count)), result_fusion)
    # cv2.waitKey(1)
    count += 1