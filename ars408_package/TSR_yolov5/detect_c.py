# -*- coding: utf-8 -*-
"""
Created on Tue Dec  6 19:19:30 2022

@author: jrwan
"""

import argparse
import os
import platform
import shutil
import time
#from pathlib import Path
import pathlib
plt = platform.system()
if plt == 'Linux': pathlib.WindowsPath = pathlib.PosixPath


import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from utils.datasets import LoadStreams, LoadImages
from utils.general import ( non_max_suppression, scale_coords, xyxy2xywh, strip_optimizer)
from utils.plots import plot_one_box_TSR
from utils.torch_utils import select_device, time_synchronized

from models.models import ( Darknet, load_darknet_weights )
from utils.datasets import (letterbox )
#from utils.general import *

from PIL import Image as im
import numpy as np

# For YoloV5 classfication
import torch.nn.functional as F

from modelsYolov5.common import DetectMultiBackend
from utilsYolov5.augmentations import classify_transforms
from utilsYolov5.general import (LOGGER, Profile, check_img_size_Yolov5)



class TrafficSign:
    def __init__(self):
        self.img = None
        self.shape = None
        self.shape_conf = None
        self.color = None
        self.color_conf = None
        self.type = None
        self.type_conf = None
        self.speed = None # if type is RE
        self.light = None # if shape is TrafficLight
        self.distance = None
        

## ========================================== SVM NEW =====================================
def showimg(img):
    cv2.imshow('Image', img)
    # 按下任意鍵則關閉所有視窗
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    
## ========================================== 20220924 ===================================

def loadIcon(path, x, y):
    icon       = im.open(path)
    icon       = icon.resize((x,y))
    icon       = cv2.cvtColor(np.asarray(icon),cv2.COLOR_RGB2BGR) 
    icon_img   = im.fromarray(icon) 

    return icon_img

## LoadImage
greenlight_img = loadIcon("./svm/svm_class/greenlight.jpg", 30, 30)
yellowlight_img = loadIcon("./svm/svm_class/yellowlight.jpg", 30, 30)
redlight_img = loadIcon("./svm/svm_class/redlight.jpg", 30, 30)
upArrow_img = loadIcon("./svm/svm_class/upArrow.jpg", 30, 30)
leftArrow_img = loadIcon("./svm/svm_class/leftArrow.jpg", 30, 30)
rightArrow_img = loadIcon("./svm/svm_class/rightArrow.jpg", 30, 30)


NB3_img = loadIcon("./images/UI/NB3.jpg", 105, 105)
NC3_img = loadIcon("./images/UI/NC3.jpg", 105, 105)
ND3_img = loadIcon("./images/UI/ND3.jpg", 105, 105)
ND4_img = loadIcon("./images/UI/ND4.jpg", 105, 105)
NSL_img = loadIcon("./images/UI/NSL.jpg", 105, 105)
NA4_img = loadIcon("./images/UI/NA4.jpg", 105, 105)
speedlimitBG_img = loadIcon("./images/UI/speedlimitBG.jpg", 105, 105)
RF_img = loadIcon("./images/UI/RF.jpg", 105, 105)
SB_img = loadIcon("./images/UI/SB.jpg", 105, 105)
SC1_img = loadIcon("./images/UI/SC1.jpg", 105, 105)
bar = loadIcon("./images/UI/bar3.jpg", 1280, 60)
'''
NB3_img = loadIcon("./images/UI/NB3.jpg", 60, 60)
NC3_img = loadIcon("./images/UI/NC3.jpg", 60, 60)
ND3_img = loadIcon("./images/UI/ND3.jpg", 60, 60)
ND4_img = loadIcon("./images/UI/ND4.jpg", 60, 60)
NSL_img = loadIcon("./images/UI/NSL.jpg", 60, 60)
NA4_img = loadIcon("./images/UI/NA4.jpg", 60, 60)
speedlimitBG_img = loadIcon("./images/UI/speedlimitBG.jpg", 60, 60)
RF_img = loadIcon("./images/UI/RF.jpg", 60, 60)
SB_img = loadIcon("./images/UI/SB.jpg", 60, 60)
SC1_img = loadIcon("./images/UI/SC1.jpg", 60, 60)
bar = loadIcon("./images/UI/bar3.jpg", 1280, 60)
'''
def showIcon(img, iconImg, x, y):

    demo = im.fromarray(img)
    demo.paste(iconImg, (x ,y))
    demo_test     = cv2.cvtColor(np.asarray(demo),cv2.COLOR_RGB2BGR)
    img           = cv2.cvtColor(np.asarray(demo_test),cv2.COLOR_BGR2RGB)

    return img


def run(
        im0s,
        imgsz=(224, 224),  # inference size (height, width)
        model='',
):
    
    names = model.names

    # Run inference
    seen, dt = 0, (Profile(), Profile(), Profile())
        
    transformations = classify_transforms()
    im = transformations(im0s) ##########################
    
    with dt[0]:
        im = torch.Tensor(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

    # Inference
    with dt[1]:
        results = model(im)
        
    # Post-process
    with dt[2]:
        pred = F.softmax(results, dim=1)  # probabilities

    # Process predictions
    for i, prob in enumerate(pred):  # per image
        seen += 1
        
        # Print results
        top5i = prob.argsort(0, descending=True)[:5].tolist()  # top 5 indices

        # print(top5i)
        # Write results
        # text = '\n'.join(f'{prob[j]:.2f} {names[j]}' for j in top5i)
        # print('!!!!!!!!!!!!!!!!!!!!!!!!     ', text, '         !!!!!!!!!!!!!!!!!!!!!!!!!!')

    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    return (names[top5i[0]], prob[top5i[0]].item())


# 新增Yolo-Tiny的SpeedLimit Detection
def SpeedLimit_Detect(model, im0s, imgsz, conf_thres, iou_thres, classes, agnostic_nms, augment, device, half
, names, colors, path = None, save_img = False):
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    
    im0s = letterbox(im0s, new_shape=imgsz, auto_size=64)[0]

    # Convert
    im0s = im0s[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    im0s = np.ascontiguousarray(im0s)
    
    img = torch.from_numpy(im0s).to(device)
    
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img, augment=augment)[0]

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
    t2 = time_synchronized()
    
    SpeedLimitSign_list = []

    # Process detections
    for i, det in enumerate(pred):  # detections per image

        if det is not None and len(det):

            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0s.shape).round()

            for *xyxy, conf, cls in det:
                SpeedLimitSign_list.append((int(cls), float(conf)))

        SpeedLimitSign_list.sort( key = lambda s: s[1] )
        print(SpeedLimitSign_list)

        # Print time (inference + NMS)
        print('Speed-Done. (%.3fs)' % ( t2 - t1))

        if SpeedLimitSign_list is not None and len(SpeedLimitSign_list):
            return SpeedLimitSign_list[0][0]
        
        return None

def lightdetect(model, im0s, imgsz, conf_thres, iou_thres, classes, agnostic_nms, augment, device, half
, names, colors, path, save_img, state_light):

    # Run inference
    t0 = time.time()

    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once

    im0s = letterbox(im0s, new_shape=imgsz, auto_size=64)[0]

    # Convert
    im0s = im0s[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    im0s = np.ascontiguousarray(im0s)
    
    img = torch.from_numpy(im0s).to(device)
    
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img, augment=augment)[0]
    #showimg(im0s)

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
    t2 = time_synchronized()

    # Process detections
    for i, det in enumerate(pred):  # detections per image

        p, s, im0 = path, '', im0s

        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if det is not None and len(det):

            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += '%g %ss, ' % (n, names[int(c)])  # add to string

            # Write results
            for *xyxy, conf, cls in det:

                state_light[int(cls)] = True

                c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                print("\nc1:",c1, "c2:", c2, "\n")


        # Print time (inference + NMS)
        print('%sLight-Done. (%.3fs)' % (s, t2 - t1))
        ##########################################################################
        
        return state_light
    
## =======================================================


def load_classes(path):
    # Loads *.names file at 'path'
    with open(path, 'r') as f:
        names = f.read().split('\n')
    return list(filter(None, names))  # filter removes empty strings (such as last line)


def show_allSign( im0, allSign_list, frame=None ):
    
    Im0Xmax = im0.shape[1]
    Im0Ymax  = im0.shape[0]
    sign_count = 0
    '''
    for sign_count, sign in enumerate(allSign_list):
        if allSign_list[sign_count] == 'Circle RED NB3':
            im0 = showIcon(im0, NB3_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle RED NC3':
            im0 = showIcon(im0, NC3_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle RED ND3':
            im0 = showIcon(im0, ND3_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle RED ND4':
            im0 = showIcon(im0, ND4_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle RED NSL':
            im0 = showIcon(im0, NSL_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle RED NA4':
            im0 = showIcon(im0, NA4_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Circle BLUE RF':
            im0 = showIcon(im0, RF_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Rectangle YELLOW SB1':
            im0 = showIcon(im0, SB_img, (sign_count)*105, Im0Ymax-105)
        if allSign_list[sign_count] == 'Rectangle YELLOW SC1':
            im0 = showIcon(im0, SC1_img, (sign_count)*105, Im0Ymax-105)
    '''
    '''
    if frame != None and frame >= 0 and frame <= 320:
        im0 = showIcon(im0, SB_img, (sign_count)*60 + 350, Im0Ymax-60)
    if frame != None and frame >= 482 and frame <= 710:
        im0 = showIcon(im0, SC1_img, (sign_count)*60 + 350, Im0Ymax-60)
    '''
    #im0 = showIcon(im0, SC1_img, (sign_count)*60 + 350, Im0Ymax-60)
    return im0
            
def show_speedLimitSign( im0, speedLimitSign_list, frame=None ):
    #print(speedLimitSign_list)
    Im0Xmax = im0.shape[1]
    Im0Ymax  = im0.shape[0]
    
    for speedLimitSign_count, speedLimitSign in enumerate(speedLimitSign_list):
        print( speedLimitSign_count, speedLimitSign )
        
        
        im0 = showIcon(im0, speedlimitBG_img, Im0Xmax - (105 * (speedLimitSign_count+1)), Im0Ymax - 105)      #印出 速限 icon 
        #im0 = showIcon(im0, speedlimitBG_img, 0, 0) 
        
        speedNum = (speedLimitSign+1) * 10  # 速限detect出來的值
        print('-----------------------------',speedNum)
        #speedNum = 40

        if speedNum   :       # 假如 速限有被偵測出來(為True的時候)
        
            SP_x = Im0Xmax - (105 * (speedLimitSign_count+1))
            SP_y = Im0Ymax - 105
        
        
            if speedNum >= 100: # 針對 3位數 以上的值 
                SP_x-=10    #微調x軸位置
                
            cv2.putText(im0, "%s" %speedNum , (SP_x+31, SP_y+53) ,cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)    #印 速限"值" 在 速限icon 上
            # print('前方速限 ' + str(speedNum) + ' 公里')
            text = 'The speed limit ahead is ' + str(speedNum) + ' km/h, please slow down'
            # cv2.putText(im0, text, (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    

    '''
    # show in bar
    if len(speedLimitSign_list) != 0:
        speedNum = (speedLimitSign_list[0]+1)*10
        im0 = showIcon(im0, speedlimitBG_img, 1040, Im0Ymax - 60)  
        cv2.putText(im0, "%s" %speedNum , (1059, Im0Ymax - 32) ,cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    '''
    return im0


def show_trafficLight( im0, trafficLight_list, frame=None ):
    for trafficLight in trafficLight_list:
        if True not in trafficLight[1]:
            trafficLight_list.remove(trafficLight)
            
    
    trafficLight_list.sort( key = lambda s: s[0][1] ) # 以y軸高度做初步排序
    for trafficLight_count, trafficLight in enumerate(trafficLight_list):
        # print( trafficLight[0][1], trafficLight[1][0] )
        
        light_count = 0
        
        # 0:green  1:red 2:yellow 3:up 4:left  5:right 
        if trafficLight[1][0]:
            im0 = showIcon(im0, greenlight_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1
        elif trafficLight[1][1]:
            im0 = showIcon(im0, redlight_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1

        elif trafficLight[1][2]:
            im0 = showIcon(im0, yellowlight_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1
        
        if trafficLight[1][3] and not trafficLight[1][0] and not trafficLight[1][1] and not trafficLight[1][2]:
            im0 = showIcon(im0, upArrow_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1

        if trafficLight[1][4] and not trafficLight[1][0] and not trafficLight[1][2]:
            im0 = showIcon(im0, leftArrow_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1

        if trafficLight[1][5] and not trafficLight[1][0] and not trafficLight[1][2]:
            im0 = showIcon(im0, rightArrow_img, (light_count+1)*30, (trafficLight_count+1)*30)
            light_count +=1
    
    if len(trafficLight_list) != 0 and trafficLight_list[0][1][1]:
        pass
        # print('前方號誌 紅燈 請降低車速並停止')
        # cv2.putText(im0, 'The traffic light ahead is red, please slow down and stop', (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        # im0 = showIcon(im0, redlight_img, 70, 673)

    return im0
        

def detect(save_img=False):
    out, source, weights, view_img, save_txt, imgsz, cfg, names = \
        opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, opt.cfg, opt.names
    webcam = source == '0' or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')

    # Initialize
    device = select_device(opt.device)
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = Darknet(cfg, imgsz).cuda()

    try:
        model.load_state_dict(torch.load(weights[0], map_location=device)['model'])
        #model = attempt_load(weights, map_location=device)  # load FP32 model
        #imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    except:
        load_darknet_weights(model, weights[0])
        input("light-model error!")
        
    model.to(device).eval()


    if half:
        model.half()  # to FP16
    
    '''---------------------------'''
    #TODO : YoloV5 Load Parameter
    
    device_yolov5 = select_device('')
    half_yolov5=False
    dnn_yolov5 = False
    data_yolov5 = ''
    imgsz_yolov5 = (244, 244)
    #TODO : RECTANGLE COLOR Classfication (RC)
    
    weights_rc = './weights/RCbest100ep.pt'
    model_rc = DetectMultiBackend(weights_rc, device=device_yolov5, dnn=dnn_yolov5, data=data_yolov5, fp16=half_yolov5)
    stride, pt = model_rc.stride, model_rc.pt
    imgsz_rc = check_img_size_Yolov5(imgsz_yolov5, s=stride)  # check image size
    bs = 1
    model_rc.warmup(imgsz=(1 if pt else bs, 3, *imgsz_rc))  # warmup
    
    #TODO : CIRCLE RED TYPE Classfication (CRT)
    
    weights_crt = './weights/CRTbest100ep.pt'
    model_crt = DetectMultiBackend(weights_crt, device=device_yolov5, dnn=dnn_yolov5, data=data_yolov5, fp16=half_yolov5)
    stride, pt = model_crt.stride, model_crt.pt
    imgsz_crt = check_img_size_Yolov5(imgsz_yolov5, s=stride)  # check image size
    bs = 1
    model_crt.warmup(imgsz=(1 if pt else bs, 3, *imgsz_crt))  # warmup
    
    #TODO : CIRCLE COLOR Classfication (CC)
    
    weights_cc = './weights/CCbest100ep.pt'
    model_cc = DetectMultiBackend(weights_cc, device=device_yolov5, dnn=dnn_yolov5, data=data_yolov5, fp16=half_yolov5)
    stride, pt = model_cc.stride, model_cc.pt
    imgsz_cc = check_img_size_Yolov5(imgsz_yolov5, s=stride)  # check image size
    bs = 1
    model_cc.warmup(imgsz=(1 if pt else bs, 3, *imgsz_cc))  # warmup
    
    '''---------------------------'''
    

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz)
    else:
        save_img = True
        dataset = LoadImages(source, img_size=imgsz, auto_size=64)
        #print("\ndataset:",dataset)

    # Get names and colors
    names = load_classes(names)
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
    
    
    # Load SpeedLimitLight
    ## =================== SpeedLimit Model Load =================== ##
    if opt.speedLimit_cnn:
        weights_SpeedLimit, imgsz_SpeedLimit, cfg_SpeedLimit, names_SpeedLimit = \
        opt.weights_SpeedLimit, opt.img_size_SpeedLimit, opt.cfg_SpeedLimit, opt.names_SpeedLimit

        # Load model
        model_SpeedLimit = Darknet(cfg_SpeedLimit, imgsz_SpeedLimit).cuda()

        try:
            model_SpeedLimit.load_state_dict(torch.load(weights_SpeedLimit[0], map_location=device)['model'])
        except:
            load_darknet_weights(model_SpeedLimit, weights_SpeedLimit[0])
            input("SpeedLimit-model error!")
        model_SpeedLimit.to(device).eval()


        if half:
            model_SpeedLimit.half()  # to FP16

        # Get names and colors
        names_SpeedLimit = load_classes(names_SpeedLimit)

    ## =================== SpeedLimit Model Load END =================== ===================
    
    ## =================== Light Model Load =================== ##

    if opt.light_cnn:
        weights_light, imgsz_light, cfg_light, names_light = \
        opt.weights_light, opt.img_size_light, opt.cfg_light, opt.names_light

        # Load model
        model_light = Darknet(cfg_light, imgsz_light).cuda()

        try:
            model_light.load_state_dict(torch.load(weights_light[0], map_location=device)['model'])
        except:
            load_darknet_weights(model_light, weights_light[0])
            input("light-model error!")
        model_light.to(device).eval()


        if half:
            model_light.half()  # to FP16

        # Get names and colors
        names_light = load_classes(names_light)

    ## ===================   Light Model Load END =================== ===================

    # Run inference
    t0 = time.time()
    
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    for path, img, im0s, vid_cap in dataset:
        #print("------------------------------------------------",dataset.frame)
        
        img = torch.from_numpy(img).to(device)
        
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = model(img, augment=opt.augment)[0]
        #showimg(im0s)

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        # t2 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            change_label = True
            
            trafficLight_list = [] # To collect all TrafficLight detect results, Finally show on the image.
            speedLimitSign_list = []
            allSign_list = []

            if webcam:  # batch_size >= 1
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
            else:
                p, s, im0 = path, '', im0s
                
            save_path = str(pathlib.Path(out) / pathlib.Path(p).name)
            txt_path = str(pathlib.Path(out) / pathlib.Path(p).stem) + ('_%g' % dataset.frame if dataset.mode == 'video' else '')
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None and len(det):

                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # Write results
                for *xyxy, conf, cls in det:
                    label_to_show = ''
                    ## TSR recognition Algo START

                    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                    print("\nc1:",c1, "c2:", c2, "\n")

                    if int(cls)== 0:
                        
                        c1, c2 = (int(xyxy[0])-10, int(xyxy[1])), (int(xyxy[2])+10, int(xyxy[3]))
                        if c1[0] <0 :
                            c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2])+10, int(xyxy[3]))
                        print("\nCHANGEc1:",c1, "c2:", c2, "\n")

                    anchor_img = im0[c1[1]:c2[1], c1[0]:c2[0]]

                    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))

                    #anchor_height = (xyxy[3]-xyxy[1])
                    #anchor_width = (xyxy[2]-xyxy[0])
                    
                    '''-------------------------------------------------------'''

                    # print( '-------------- ', color[0], ', ', color[1], ' -----------------' )
                    
                    ## 開始第二階段SVM
                    if int(cls)== 0 : # 紅綠燈 TrafficLight
                        print( 'TrafficLight' )
                        
                    elif int(cls)== 1: # 八角形 Octagon
                        print( 'Octagon' )
                        
                    elif int(cls)== 2: # 圓形 Circle
                        print( 'Circle' )
                        color = run( im0s = anchor_img, imgsz=(224, 224),  model = model_cc )
                        if color[0] == 'RED':
                            s_type = run( im0s = anchor_img, imgsz=(224, 224),  model = model_crt )
                            
                            if s_type[0] == 'RE':
                                speedLimitSign = SpeedLimit_Detect(model_SpeedLimit, anchor_img, imgsz_SpeedLimit, opt.conf_thres_SpeedLimit, 
                                                          opt.iou_thres_SpeedLimit, opt.classes_SpeedLimit, opt.agnostic_nms_SpeedLimit, opt.augment, 
                                                          device, half, names_SpeedLimit, 0, path, save_img)
                                
                                if speedLimitSign is not None:
                                    speedLimitSign_list.append(speedLimitSign)
                                else:
                                    continue
                        if color[0] == 'BLUE':
                            pass
                    elif int(cls)== 3: # 長方形 Rectangle
                        print( 'Rectangle' )
                        color = run( im0s = anchor_img, imgsz=(224, 224),  model = model_rc )
                        if color[0] == 'YELLOW':
                            pass
                    elif int(cls)== 4: # 三角形 Triangle
                        print( 'Triangle' )
                        
                    elif int(cls)== 5: # 菱形 Diamond
                        print( 'Diamond' )
                        
                    elif int(cls)== 6: # 梅花 Plum
                        print( 'Plum' )
                        
                    elif int(cls)== 7: # 圓角三角形 Circle-Triangle
                        print('Circle-Triangle')
                        
                    ## TSR recognition Algo END

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    if save_img or view_img:  # Add bbox to image
                        # label = '%s %.2f' % (names[int(cls)], conf)
                        label = '%s' % (names[int(cls)])
                        if change_label:
                            label = label_to_show
                        #plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                        plot_one_box_TSR(c1, c2, im0, label=label, color=colors[int(cls)], line_thickness=1,classNo=int(cls))


            # Print time (inference + NMS)
            # print('%sDone. (%.3fs)' % (s, t2 - t1))
            # show result on image
            # im0 = showIcon(im0, bar, 0, im0.shape[0]-60)
            im0 = show_trafficLight(im0, trafficLight_list, dataset.frame)
            im0 = show_speedLimitSign(im0, speedLimitSign_list, dataset.frame)
            im0 = show_allSign( im0, allSign_list, dataset.frame)
            
            # Stream results
            if view_img:
                cv2.imshow(p, im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'images':
                    cv2.imwrite(save_path, im0)
                else:
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer

                        fourcc = 'mp4v'  # output video codec
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
                    vid_writer.write(im0)
               
            t3 = time_synchronized()
            print('%sDone. (%.3fs)' % (s, t3 - t1))
    if save_txt or save_img:
        print('Results saved to %s' % pathlib.Path(out))
        if platform == 'darwin' and not opt.update:  # MacOS
            os.system('open ' + save_path)

    print('Done. (%.3fs)' % (time.time() - t0))

class TSR():
    def __init__(self, opt):
        weights, self.view_img, imgsz, cfg, names = \
           opt.weights, opt.view_img, opt.img_size, opt.cfg, opt.names

        # Initialize
        self.device = select_device(opt.device)
        half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = Darknet(cfg, imgsz).cuda()

        try:
            self.model.load_state_dict(torch.load(weights[0], map_location=self.device)['model'])
            #model = attempt_load(weights, map_location=device)  # load FP32 model
            #imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
        except:
            load_darknet_weights(self.model, weights[0])
            input("light-model error!")
        
        self.model.to(self.device).eval()


        if half:
            self.model.half()  # to FP16
    
        '''---------------------------'''
        #TODO : YoloV5 Load Parameter
    
        self.device_yolov5 = select_device('')
        self.half_yolov5 = True
        self.dnn_yolov5 = False
        self.data_yolov5 = ''
        self.imgsz_yolov5 = (244, 244)
        #TODO : RECTANGLE COLOR Classfication (RC)
    
        weights_rc = './weights/RCbest100ep.pt'
        self.model_rc = DetectMultiBackend(weights_rc, device=self.device_yolov5, dnn=self.dnn_yolov5, data=self.data_yolov5, fp16=self.half_yolov5)
        stride, pt = self.model_rc.stride, self.model_rc.pt
        self.imgsz_rc = check_img_size_Yolov5(self.imgsz_yolov5, s=stride)  # check image size
        bs = 1
        self.model_rc.warmup(imgsz=(1 if pt else bs, 3, *self.imgsz_rc))  # warmup
    
        #TODO : CIRCLE RED TYPE Classfication (CRT)
    
        weights_crt = './weights/CRTbest100ep.pt'
        self.model_crt = DetectMultiBackend(weights_crt, device=self.device_yolov5, dnn=self.dnn_yolov5, data=self.data_yolov5, fp16=self.half_yolov5)
        stride, pt = self.model_crt.stride, self.model_crt.pt
        self.imgsz_crt = check_img_size_Yolov5(self.imgsz_yolov5, s=stride)  # check image size
        bs = 1
        self.model_crt.warmup(imgsz=(1 if pt else bs, 3, *self.imgsz_crt))  # warmup
    
        #TODO : CIRCLE COLOR Classfication (CC)
    
        weights_cc = './weights/CCbest100ep.pt'
        self.model_cc = DetectMultiBackend(weights_cc, device=self.device_yolov5, dnn=self.dnn_yolov5, data=self.data_yolov5, fp16=self.half_yolov5)
        stride, pt = self.model_cc.stride, self.model_cc.pt
        self.imgsz_cc = check_img_size_Yolov5(self.imgsz_yolov5, s=stride)  # check image size
        bs = 1
        self.model_cc.warmup(imgsz=(1 if pt else bs, 3, *self.imgsz_cc))  # warmup

        # Get names and colors
        self.names = load_classes(names)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
    
        # Load SpeedLimitLight
        ## =================== SpeedLimit Model Load =================== ##
        if opt.speedLimit_cnn:
            self.weights_SpeedLimit, self.imgsz_SpeedLimit, self.cfg_SpeedLimit, self.names_SpeedLimit = \
            opt.weights_SpeedLimit, opt.img_size_SpeedLimit, opt.cfg_SpeedLimit, opt.names_SpeedLimit

            # Load model
            self.model_SpeedLimit = Darknet(self.cfg_SpeedLimit, self.imgsz_SpeedLimit).cuda()

            try:
                self.model_SpeedLimit.load_state_dict(torch.load(self.weights_SpeedLimit[0], map_location=self.device)['model'])
            except:
                load_darknet_weights(self.model_SpeedLimit, self.weights_SpeedLimit[0])
                input("SpeedLimit-model error!")
            self.model_SpeedLimit.to(self.device).eval()


            if half:
                self.model_SpeedLimit.half()  # to FP16

            # Get names and colors
            self.names_SpeedLimit = load_classes(self.names_SpeedLimit)

        ## =================== SpeedLimit Model Load END =================== ===================
    
        ## =================== Light Model Load =================== ##

        if opt.light_cnn:
            self.weights_light, self.imgsz_light, self.cfg_light, self.names_light = \
            opt.weights_light, opt.img_size_light, opt.cfg_light, opt.names_light

            # Load model
            self.model_light = Darknet(self.cfg_light, self.imgsz_light).cuda()

            try:
                self.model_light.load_state_dict(torch.load(self.weights_light[0], map_location=self.device)['model'])
            except:
                load_darknet_weights(self.model_light, self.weights_light[0])
                input("light-model error!")
            self.model_light.to(self.device).eval()


            if half:
                self.model_light.half()  # to FP16

            # Get names and colors
            self.names_light = load_classes(self.names_light)
        
        self.p = "TSR"
        self.view_img = True
        self.opt = opt
    ## ===================   Light Model Load END =================== ===================

    def __call__(self, img0: np.ndarray):
        # Run inference 
        # showimg(img0)

        # Padded resize
        img = letterbox(img0, new_shape=self.opt.img_size, auto_size=self.opt.auto_size)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half_yolov5 else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img)[0]
        print(pred)

        # Apply NMS
        pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres, classes=None, agnostic=self.opt.agnostic_nms)
        # t2 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            change_label = True
            
            trafficLight_list = [] # To collect all TrafficLight detect results, Finally show on the image.
            speedLimitSign_list = []
            allSign_list = []
                
            gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None and len(det):

                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, self.names[int(c)])  # add to string

                # Write results
                for *xyxy, conf, cls in det:
                    label_to_show = ''
                    ## TSR recognition Algo START

                    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                    print("\nc1:",c1, "c2:", c2, "\n")

                    if int(cls)== 0:
                        
                        c1, c2 = (int(xyxy[0])-10, int(xyxy[1])), (int(xyxy[2])+10, int(xyxy[3]))
                        if c1[0] <0 :
                            c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2])+10, int(xyxy[3]))
                        print("\nCHANGEc1:",c1, "c2:", c2, "\n")

                    anchor_img = img0[c1[1]:c2[1], c1[0]:c2[0]]

                    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))

                    #anchor_height = (xyxy[3]-xyxy[1])
                    #anchor_width = (xyxy[2]-xyxy[0])
                    
                    '''-------------------------------------------------------'''

                    # print( '-------------- ', color[0], ', ', color[1], ' -----------------' )
                    
                    ## 開始第二階段SVM
                    if int(cls)== 0 : # 紅綠燈 TrafficLight
                        print( 'TrafficLight' )
                        
                    elif int(cls)== 1: # 八角形 Octagon
                        print( 'Octagon' )
                        
                    elif int(cls)== 2: # 圓形 Circle
                        print( 'Circle' )
                        color = run( im0s = anchor_img, imgsz=(224, 224),  model = self.model_cc )
                        if color[0] == 'RED':
                            s_type = run( im0s = anchor_img, imgsz=(224, 224),  model = self.model_crt )
                            
                            if s_type[0] == 'RE':
                                speedLimitSign = SpeedLimit_Detect(self.model_SpeedLimit, anchor_img, self.imgsz_SpeedLimit, self.opt.conf_thres_SpeedLimit, 
                                                        self.opt.iou_thres_SpeedLimit, None, self.opt.agnostic_nms_SpeedLimit, self.opt.augment, 
                                                        self.device, self.half_yolov5, self.names_SpeedLimit, 0)
                                
                                if speedLimitSign is not None:
                                    speedLimitSign_list.append(speedLimitSign)
                                else:
                                    continue
                        if color[0] == 'BLUE':
                            pass
                    elif int(cls)== 3: # 長方形 Rectangle
                        print( 'Rectangle' )
                        color = run( im0s = anchor_img, imgsz=(224, 224),  model = self.model_rc )
                        if color[0] == 'YELLOW':
                            pass
                    elif int(cls)== 4: # 三角形 Triangle
                        print( 'Triangle' )
                        
                    elif int(cls)== 5: # 菱形 Diamond
                        print( 'Diamond' )
                        
                    elif int(cls)== 6: # 梅花 Plum
                        print( 'Plum' )
                        
                    elif int(cls)== 7: # 圓角三角形 Circle-Triangle
                        print('Circle-Triangle')
                        
                    ## TSR recognition Algo END

                    # if save_txt:  # Write to file
                    #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #     with open(txt_path + '.txt', 'a') as f:
                    #         f.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    if self.view_img:  # Add bbox to image
                        # label = '%s %.2f' % (names[int(cls)], conf)
                        label = '%s' % (self.names[int(cls)])
                        if change_label:
                            label = label_to_show
                        #plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                        plot_one_box_TSR(c1, c2, img0, label=label, color=self.colors[int(cls)], line_thickness=1,classNo=int(cls))


            # Print time (inference + NMS)
            # print('%sDone. (%.3fs)' % (s, t2 - t1))
            # show result on image
            # im0 = showIcon(im0, bar, 0, im0.shape[0]-60)
            img0 = show_trafficLight(img0, trafficLight_list)
            img0 = show_speedLimitSign(img0, speedLimitSign_list)
            img0 = show_allSign( img0, allSign_list)
            
            # Stream results
            if self.view_img:
                cv2.imshow(self.p, img0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            t3 = time_synchronized()
            print('Done. (%.3fs)' % (t3 - t1))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weights/best.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--cfg', type=str, default='cfg/8_TrafficSignShapes.cfg', help='*.cfg path')
    parser.add_argument('--names', type=str, default='data/8_TrafficSignShapes.names', help='*.cfg path')
    
    # ========================= TrafficLight Option ============================== #
    parser.add_argument('--light-cnn', action='store_true', help='display results')
    parser.add_argument('--weights-light', nargs='+', type=str, default='./weights/6_TrafficLight-tiny_220913.pt', help='model.pt path(s)')
    parser.add_argument('--img-size-light', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--cfg-light', type=str, default='cfg/6_TrafficLight-tiny.cfg', help='*.cfg path')
    parser.add_argument('--names-light', type=str, default='data/6_TrafficLight.names', help='*.cfg path')

    parser.add_argument('--conf-thres-light', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres-light', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--classes-light', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms-light', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment-light', action='store_true', help='augmented inference')
    
    # ========================= SpeedLimit Option ============================== #
    parser.add_argument('--speedLimit-cnn', action='store_true', help='display results')
    parser.add_argument('--weights-SpeedLimit', nargs='+', type=str, default='./weights/SpeedLimit.pt', help='model.pt path(s)')
    parser.add_argument('--img-size-SpeedLimit', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--cfg-SpeedLimit', type=str, default='cfg/SpeedLimit.cfg', help='*.cfg path')
    parser.add_argument('--names-SpeedLimit', type=str, default='data/SpeedLimit.names', help='*.cfg path')

    parser.add_argument('--conf-thres-SpeedLimit', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres-SpeedLimit', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--classes-SpeedLimit', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms-SpeedLimit', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment-SpeedLimit', action='store_true', help='augmented inference')
    

    opt = parser.parse_args()
    print(opt)


    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()



