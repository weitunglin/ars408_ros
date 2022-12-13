import argparse
import os
import platform
import shutil
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from utils.google_utils import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, strip_optimizer)
from utils.plots import plot_one_box_TSR
from utils.torch_utils import select_device, load_classifier, time_synchronized

from models.models import *
from utils.datasets import *
from utils.general import *



## NEW
import joblib
from skimage.transform import resize
from PIL import Image as im
import numpy as np
##


## ========================================== SVM NEW =====================================
def showimg(img):
    cv2.imshow('Image', img)
    # 按下任意鍵則關閉所有視窗
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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
            
        #showimg(im0)
        #showimg(im0s)

        
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


                if save_img or view_img:  # Add bbox to image
                    label = '%s %.2f' % (names[int(cls)], conf)
                    #plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                    #plot_one_box_TSR(c1, c2, im0, label=label, color=colors[int(cls)], line_thickness=1,classNo=int(cls))

                    


        # Print time (inference + NMS)
        print('%sLight-Done. (%.3fs)' % (s, t2 - t1))

        return state_light
    



def loadIcon(path, x, y):
    icon       = im.open(path)
    icon       = icon.resize((x,y))
    icon       = cv2.cvtColor(np.asarray(icon),cv2.COLOR_RGB2BGR) 
    icon_img   = im.fromarray(icon) 

    return icon_img

def showIcon(img, iconImg, x, y):
    demo = im.fromarray(img)
    demo.paste(iconImg, (x ,y))
    demo_test     = cv2.cvtColor(np.asarray(demo),cv2.COLOR_RGB2BGR)
    img           = cv2.cvtColor(np.asarray(demo_test),cv2.COLOR_BGR2RGB)

    return img

    
    

# Classification rectangle 
# 分辨第二階段 長方形之"顏色"
def svm_rectangle_color(test_pic, clf):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    print(test_pic.shape)
    # img = imread(test_pic)
    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/2nd-rectangle_color/2nd-rectangle_color.pkl')

    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict

def svm_rectangle_blue(test_pic):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    clf = joblib.load('./svm/2nd-rectangle_color/3rd-svm_rectangle_blue/3rd-svm_rectangle_blue.pkl')

    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict

def svm_rectangle_green(test_pic):
    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    clf = joblib.load('./svm/2nd-rectangle_color/3rd-svm_rectangle_green/3rd-svm_rectangle_green.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict

def svm_rectangle_yellow(test_pic,clf):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/2nd-rectangle_color/3rd-svm_rectangle_yellow/3rd-svm_rectangle_yellow.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict

# Classification circle
# 分辨第二階段 圓形之"顏色"

def svm_circle_color(test_pic,clf):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/2nd-circle_color/2nd-circle_color.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    #print('svm_color:',predict)
    return predict



def svm_circle_red(test_pic,clf):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/2nd-circle_color/3rd-svm_circle_red/3rd-svm_circle_red.pkl')
    #clf = joblib.load('./svm/svm_circle_red.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict

def svm_circle_blue(test_pic, clf):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/2nd-circle_color/3rd-svm_circle_blue/3rd-svm_circle_blue.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict




# Classification triangle
# 分辨第三階段 三角形之"種類"
def svm_triangle(test_pic):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)

    #clf = joblib.load('./svm/3rd-svm_triangle/svm_triangle.pkl')
    
    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict


# 分辨第三階段 紅綠燈之 "種類"
def SVM_light(test_pic):

    images      = []
    flat_data   = []
    dimension   = (64, 64)

    # img = imread(test_pic)
    img_resized = resize(test_pic, dimension, anti_aliasing=True, mode='reflect')
    flat_data.append(img_resized.flatten()) 
    images.append(img_resized)


    #clf = joblib.load('./svm/3rd-svm_trafficlight/3rd-svm_trafficlight.pkl')


    # predict
    predict = clf.predict(flat_data)
    print('svm:',predict)
    return predict


## =======================================================


def load_classes(path):
    # Loads *.names file at 'path'
    with open(path, 'r') as f:
        names = f.read().split('\n')
    return list(filter(None, names))  # filter removes empty strings (such as last line)

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


    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model'])  # load weights
        modelc.to(device).eval()

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
    
    

    ## =================== Light Model Load =================== ##

    if opt.light_cnn:
        weights_light, imgsz_light, cfg_light, names_light = \
        opt.weights_light, opt.img_size_light, opt.cfg_light, opt.names_light

        # Initialize
        
        #half = device.type != 'cpu'  # half precision only supported on CUDA

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
        colors_light = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names_light))]

    #trafficlightBG_img = loadIcon("./svm/svm_class/trafficlight.jpg", 180, 30)
    greenlight_img = loadIcon("./svm/svm_class/greenlight.jpg", 30, 30)
    yellowlight_img = loadIcon("./svm/svm_class/yellowlight.jpg", 30, 30)
    redlight_img = loadIcon("./svm/svm_class/redlight.jpg", 30, 30)
    upArrow_img = loadIcon("./svm/svm_class/upArrow.jpg", 30, 30)
    leftArrow_img = loadIcon("./svm/svm_class/leftArrow.jpg", 30, 30)
    rightArrow_img = loadIcon("./svm/svm_class/rightArrow.jpg", 30, 30)



    speedLimit_img = loadIcon("./svm/svm_class/speedlimitBG.jpg", 105, 105)



    ## ===================   Light Model Load END =================== ===================



    ## SVM LOAD

    c_clf = joblib.load('./svm/2nd-circle_color/2nd-circle_color.pkl')
    cR_clf = joblib.load('./svm/2nd-circle_color/3rd-svm_circle_red/3rd-svm_circle_red.pkl')
    r_clf = joblib.load('./svm/2nd-rectangle_color/2nd-rectangle_color.pkl')

    rY_clf = joblib.load('./svm/2nd-rectangle_color/3rd-svm_rectangle_yellow/3rd-svm_rectangle_yellow.pkl')
    cB_clf = joblib.load('./svm/2nd-circle_color/3rd-svm_circle_blue/3rd-svm_circle_blue.pkl')



    ##

    # Run inference
    t0 = time.time()
    
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once


    width   = img.shape[1]
    height  = img.shape[0]

    for path, img, im0s, vid_cap in dataset:
        #print("\n",img)
        
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
        t2 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)
            input("\nclassify")

        # Process detections
        for i, det in enumerate(pred):  # detections per image

            tflight_count = 1
            RE_count = 1

            if webcam:  # batch_size >= 1
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
            else:
                p, s, im0 = path, '', im0s
                
            #showimg(im0)
            #showimg(im0s)

            Im0Xmax = im0.shape[1]
            Im0Ymax  = im0.shape[0]

            save_path = str(Path(out) / Path(p).name)
            txt_path = str(Path(out) / Path(p).stem) + ('_%g' % dataset.frame if dataset.mode == 'video' else '')
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

                    light_count = 1

                    sign_count = 1

                    #showimg(anchor_img)

                    light_height = (xyxy[3]-xyxy[1])
                    light_width = (xyxy[2]-xyxy[0])



                    ## 開始第二階段SVM
                    # if int(cls)== 0 and (light_height<25):
                    #     continue
                    

                    if int(cls)== 0 :

                        print("\ntrafficlight")

                        if light_height < 25 :
                            continue
                        elif (light_height >= light_width) and light_width < 30 :
                            continue
                        
                        print("tfli:",tflight_count)



                        if opt.light_cnn:

                            ##                 0:green  1:red 2:yellow 3:up 4:left  5:right 
                            state_light = np.array([False, False, False, False, False, False]) 


                            state_light = lightdetect(model_light, anchor_img, imgsz_light, opt.conf_thres_light, opt.iou_thres_light,
                            opt.classes_light, opt.agnostic_nms_light, opt.augment, device, half, names_light,
                            colors_light, path, save_img, state_light)

                            #im0 = showIcon(im0, trafficlightBG_img, 0, tflight_count*30)
                            

                            if state_light[0]:

                                im0 = showIcon(im0, greenlight_img, light_count*30, tflight_count*30)
                                light_count +=1
                            
                            if state_light[1]:

                                im0 = showIcon(im0, redlight_img, light_count*30, tflight_count*30)
                                light_count +=1

                            if state_light[2]:

                                im0 = showIcon(im0, yellowlight_img, light_count*30, tflight_count*30)
                                light_count +=1

                            if state_light[3]:

                                im0 = showIcon(im0, upArrow_img, light_count*30, tflight_count*30)
                                light_count +=1

                            if state_light[4]:

                                im0 = showIcon(im0, leftArrow_img, light_count*30, tflight_count*30)
                                light_count +=1

                            if state_light[5]:

                                im0 = showIcon(im0, rightArrow_img, light_count*30, tflight_count*30)
                                light_count +=1



                                
                                
                                
                        



                    elif int(cls)== 1:
                        print("\nOctagon")
                        continue

                    elif int(cls)== 2:
                        print("\ncircle")
                        #continue
                        svm_color     = int(svm_circle_color(anchor_img, c_clf))

                        if svm_color  == 0 :
                            print("S2:blue")

                            circle = int(svm_circle_blue(anchor_img, cB_clf))

                            if circle == 0:
                                print("FB2")

                            elif circle == 1:
                                print("FD4")

                            elif circle == 2:
                                print("RF")

                        elif svm_color  == 1 :
                            print("S2:Red")

                            circle = int(svm_circle_red(anchor_img,cR_clf))

                            if circle == 0:
                                print("NA4")
                                
                            elif circle == 1:
                                print("NB3")
                                
                            elif circle == 2:
                                print("NC3")

                                
                            elif circle == 3:
                                print("ND3")
                                
                            elif circle == 4:
                                print("no-entry")
                                
                            elif circle == 5:                           
                                print("RE")



                                
                                im0 = showIcon(im0, speedLimit_img, Im0Xmax - (105 * sign_count), Im0Ymax - 105)      #印出 速限 icon 
                                

                                speedNum = 90  # 速限detect出來的值


                                if speedNum   :       # 假如 速限有被偵測出來(為True的時候)
                                
                                    SP_x = Im0Xmax - (105 * sign_count)
                                    SP_y = Im0Ymax - 105
                                
                                
                                    if speedNum >= 100: # 針對 3位數 以上的值 
                                        SP_x-=10    #微調x軸位置
                                        
                                    cv2.putText(im0, "%s" %speedNum , (SP_x+31, SP_y+53) ,cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)    #印 速限"值" 在 速限icon 上


                                sign_count += 1

                                    

                        
                    elif int(cls)== 3:
                        print("\nrectangle")
                        continue
                        

                    #     svm_color     = int(svm_rectangle_color(anchor_img, r_clf))

                    #     if svm_color == 0:
                    #         print("blue")

                    #         # rectangle    = int(svm_rectangle_blue(anchor_img))
                    #         # if rectangle == 0:
                    #         #     print("FB3")

                    #         # if rectangle == 1:
                    #         #     print("FC3")

                    #         # if rectangle == 2:
                    #         #     print("IB11")

                    #         # if rectangle == 3:
                    #         #     print("ID7")

                    #     elif svm_color == 1:
                    #         print("green")

                    #         # rectangle  = int(svm_rectangle_green(anchor_img))
                    #         # if rectangle == 0:
                    #         #     print("IC6")

                    #         # if rectangle == 1:
                    #         #     print("ID6")

                    #         # if rectangle == 2:
                    #         #     print("IF5")

                    #     if svm_color == 2:
                    #         print("yellow") 

                    #         # rectangle  = int(svm_rectangle_yellow(anchor_img, rY_clf))
                    #         # if rectangle == 0:
                    #         #     print("SB1")
                    #         #     #img     = cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, bbox_thick//2)

                    #         # if rectangle == 1:
                    #         #     print("SC1")
                    #         #     #img     = cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, bbox_thick//2)

                    #         # if rectangle == 2:
                    #         #     print("SC4")
                        
                    elif int(cls)== 4:
                        print("\ntriangle")
                        continue
                        
                    elif int(cls)== 5:
                        print("\ndiamond")
                        continue
                        
                    elif int(cls)== 6:
                        print("\nplum")
                        continue
                    
                    elif int(cls)== 7:
                        print("\ncircle-triangle")
                        continue
                        
                    


                    ## TSR recognition Algo END


                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    if save_img or view_img:  # Add bbox to image
                        label = '%s %.2f' % (names[int(cls)], conf)
                        #plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                        plot_one_box_TSR(c1, c2, im0, label=label, color=colors[int(cls)], line_thickness=1,classNo=int(cls))


            # Print time (inference + NMS)
            print('%sDone. (%.3fs)' % (s, t2 - t1))

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

    if save_txt or save_img:
        print('Results saved to %s' % Path(out))
        if platform == 'darwin' and not opt.update:  # MacOS
            os.system('open ' + save_path)

    print('Done. (%.3fs)' % (time.time() - t0))


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

     
    

    opt = parser.parse_args()
    print(opt)


    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()



