from detect_c import TSR
from easydict import EasyDict as edict
import numpy as np
import torch

opt = opt = edict(
            weights = ["weights/8_TrafficShape_221009.pt"],
            view_img = True,
            img_size = 640,
            auto_size = 32,
            conf_thres = 0.4,
            iou_thres = 0.5,
            agnostic_nms = False,
            device = "0",
            cfg = "cfg/8_TrafficSignShapes.cfg",
            names = "data/8_TrafficSignShapes.names",
            light_cnn = True,
            weights_light = ["weights/6_TrafficLight-tiny_221026N.pt"],
            img_size_light = 640,
            cfg_light = "cfg/6_TrafficLight-tiny.cfg",
            names_light = "data/6_TrafficLight.names",
            agnostic_nms_light = False,
            conf_thres_light = 0.4,
            iou_thres_light = 0.5,
            speedLimit_cnn = True,
            weights_SpeedLimit = ["weights/SpeedLimit221005.pt"],
            img_size_SpeedLimit = 640,
            cfg_SpeedLimit = "cfg/SpeedLimit.cfg",
            names_SpeedLimit = "data/SpeedLimit.names",
            agnostic_nms_SpeedLimit = False,
            conf_thres_SpeedLimit = 0.4,
            iou_thres_SpeedLimit = 0.5,
        )

model = TSR(opt)

"""
import cv2
x = cv2.imread("/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_package/TSR_yolov5/images/speedLimit/10.jpg")
x = x[:, :, ::-1].transpose(2, 0, 1)
x = np.ascontiguousarray(x)
"""

x = np.random.rand(640, 640, 3)

print(model(x))
