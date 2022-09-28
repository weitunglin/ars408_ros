#! /usr/bin/env python3
# coding=utf-8
import threading
import os
import sys
from collections import defaultdict
from functools import partial
from typing import List

import rospy
import torch
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4")

from PyTorch_YOLOv4.utils.general import non_max_suppression
from PyTorch_YOLOv4.models.models import Darknet, load_darknet_weights
from PyTorch_YOLOv4.detect import load_classes
from ars408_msg.msg import Bboxes, Bbox
from config.config import default_config, rgb_config


class YOLO():
    def __init__(self):
        self.bridge = CvBridge()
        self.setup_model()

    def setup_model(self):
        self.model = Darknet(rgb_config.model["cfg"], rgb_config.model["image_size"])
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        if default_config.use_cuda:
            self.model.cuda()
        
        try:
            ckpt = torch.load(rgb_config.model["weights"], map_location=self.device)  # load checkpoint
            ckpt['model'] = {k: v for k, v in ckpt['model'].items() if self.model.state_dict()[k].numel() == v.numel()}
            self.model.load_state_dict(ckpt['model'], strict=False)
        except:
            load_darknet_weights(self.model, rgb_config.model["weights"])

        self.model.to(self.device).eval()
        if default_config.use_yolo_half:
            self.model.half()
        
        self.class_names = load_classes(rgb_config.model["names"])

    def convert_to_torch(self, image):
        img = image.copy()
        img = cv2.resize(image, rgb_config.model["image_size"])
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = torch.from_numpy(img).to(self.device, dtype=torch.float16).div(255.0)
        img = torch.permute(img, (2, 0, 1))
        img = torch.unsqueeze(img, 0)
        return img
    
    def draw_yolo_image(self, img: cv2.Mat, bounding_boxes: List[Bbox]) -> cv2.Mat:
        """
        TODO
        move to utils function
        """
        for i in bounding_boxes:
            cv2.putText(img, i.objClass, (int(i.x_min), int(i.y_min) - 7), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 4)
            cv2.rectangle(img, (int(i.x_min), int(i.y_min)), (int(i.x_max), int(i.y_max)), color=(255, 0, 0), thickness=4)
        return img

    def inference(self, rgb_images, rgb_names):
        """
        inference batch of images
        rgb_images: List[Image]
        rgb_names: List[str]

        `rgb_images` and `rgb_names` must have same length, and match the indices.
        """
        bounding_boxes_array = []
        if len(rgb_images):
            imgs = torch.vstack(tuple([self.convert_to_torch(image) for image in rgb_images])).to(self.device, dtype=torch.float16)
            with torch.no_grad():
                preds = self.model(imgs)[0]

                preds = non_max_suppression(preds, rgb_config.model["conf_thres"], rgb_config.model["iou_thres"])
                
                for rgb_name, pred in zip(rgb_names, preds):
                    bounding_boxes = Bboxes()
                    pred = pred.cpu().numpy()
                    for *xyxy, conf, cls in pred:
                        bounding_boxes.bboxes.append(
                            Bbox(
                                x_min=xyxy[0] / rgb_config.model["image_size"][0] * rgb_config[rgb_name].size[0],
                                y_min=xyxy[1] / rgb_config.model["image_size"][1] * rgb_config[rgb_name].size[1],
                                x_max=xyxy[2] / rgb_config.model["image_size"][0] * rgb_config[rgb_name].size[0],
                                y_max=xyxy[3] / rgb_config.model["image_size"][1] * rgb_config[rgb_name].size[1],
                                score=conf,
                                objClassNum=int(cls),
                                objClass=self.class_names[int(cls)],
                            )
                        )
                    bounding_boxes_array.append(bounding_boxes.bboxes)

        return bounding_boxes_array


# def main():
#     rospy.init_node("Single RGB PyTorch YOLO")    

#     inference_instance = YOLO()
#     rospy.spin()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInternalException:
#         pass
