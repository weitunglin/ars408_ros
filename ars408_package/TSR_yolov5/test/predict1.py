# YOLOv5 ğŸš€ by Ultralytics, GPL-3.0 license
"""
Run YOLOv5 classification inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python classify/predict.py --weights yolov5s-cls.pt --source 0                               # webcam
                                                                   img.jpg                         # image
                                                                   vid.mp4                         # video
                                                                   screen                          # screenshot
                                                                   path/                           # directory
                                                                   'path/*.jpg'                    # glob
                                                                   'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                                   'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python classify/predict.py --weights yolov5s-cls.pt                 # PyTorch
                                           yolov5s-cls.torchscript        # TorchScript
                                           yolov5s-cls.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                           yolov5s-cls_openvino_model     # OpenVINO
                                           yolov5s-cls.engine             # TensorRT
                                           yolov5s-cls.mlmodel            # CoreML (macOS-only)
                                           yolov5s-cls_saved_model        # TensorFlow SavedModel
                                           yolov5s-cls.pb                 # TensorFlow GraphDef
                                           yolov5s-cls.tflite             # TensorFlow Lite
                                           yolov5s-cls_edgetpu.tflite     # TensorFlow Edge TPU
                                           yolov5s-cls_paddle_model       # PaddlePaddle
"""

import argparse # 無衝突
import os # 無衝突
import sys # 無衝突
from pathlib import Path # 無衝突

import torch # 無衝突
import torch.nn.functional as F
import cv2

from models.common import DetectMultiBackend
from utils.augmentations import classify_transforms
from utils.general import (LOGGER, Profile, check_img_size, check_requirements, print_args)
from utils.torch_utils import select_device, smart_inference_mode

def run(
        im0s,
        imgsz=(224, 224),  # inference size (height, width)
        model='',
):
    
    stride, names, pt = model.stride, model.names, model.pt

    # Run inference
    seen, dt = 0, (Profile(), Profile(), Profile())
        
    transformations = classify_transforms()
    im = transformations(im0s) ##########################
    
    print('1----------------------    ', im, '     ----------------------1')
    print('2----------------------    ', im0s, '     ----------------------2')

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

        print(top5i)

        # Write results
        text = '\n'.join(f'{prob[j]:.2f} {names[j]}' for j in top5i)

        print('!!!!!!!!!!!!!!!!!!!!!!!!     ', text, '         !!!!!!!!!!!!!!!!!!!!!!!!!!')

    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)

def main():
    check_requirements(exclude=('tensorboard', 'thop'))
    # Load model
    device = select_device('')
    weights = './weights/Rbest100ep.pt'
    half=False
    dnn = False
    data = ''
    imgsz = (244, 244)
    
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, pt = model.stride, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    bs = 1
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    
    im0s = cv2.imread('data/images/001.png')
    run( im0s = im0s, imgsz=(224, 224),  model = model )

if __name__ == "__main__":
    main()
