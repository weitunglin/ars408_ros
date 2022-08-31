#! /usr/bin/env python3
# coding=utf-8
import threading
import os
import sys
from collections import defaultdict
from functools import partial

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
from PyTorch_YOLOv4.models.models import Darknet
from PyTorch_YOLOv4.detect import load_classes
from ars408_msg.msg import Bboxes, Bbox
from config import default_config, rgb_config


class YOLO():
    def __init__(self):
        self.sub_rgb = defaultdict()
        self.pub_bounding_boxes = defaultdict()
        self.pub_yolo_images = defaultdict()
        self.rgbs = defaultdict()

        self.bridge = CvBridge()
        self.setup_model()

        self.rgb_names = rgb_config.model["rgb"]
        for rgb_name in self.rgb_names:
            self.sub_rgb[rgb_name] = message_filters.Subscriber("/rgb/" + rgb_name + "/calib_image", Image)
            self.pub_bounding_boxes[rgb_name] = rospy.Publisher("/rgb/" + rgb_name + "/bounding_boxes", Bboxes, queue_size=1)

            if default_config.use_yolo_image:
                self.pub_yolo_images[rgb_name] = rospy.Publisher("/rgb/" + rgb_name + "/yolo_image", Image, queue_size=1)
        
        self.synchronizer = message_filters.TimeSynchronizer([self.sub_rgb[i] for i in self.rgb_names], 1)
        self.synchronizer.registerCallback(self.ts_callback)
    
    def setup_model(self):
        self.model = Darknet(rgb_config.model["cfg"], rgb_config.model["image_size"])
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        rospy.loginfo("using " + str(self.device) + " device")
        if default_config.use_cuda:
            self.model.cuda()
        
        try:
            self.model.load_state_dict(torch.load(rgb_config.model["weights"], map_location=self.device)["model"])
        except:
            rospy.logerr("cannot load state dict")
            exit(-1)

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
    
    def draw_yolo_image(self, rgb_name: str, img: cv2.Mat, bounding_boxes: Bboxes) -> None:
        for i in bounding_boxes.bboxes:
            cv2.putText(img, i.objClass, (int(i.x_min), int(i.y_min) - 10), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 5)
            cv2.rectangle(img, (int(i.x_min), int(i.y_min)), (int(i.x_max), int(i.y_max)), color=(255, 0, 0), thickness=5)
        self.pub_yolo_images[rgb_name].publish(self.bridge.cv2_to_imgmsg(img))

    def ts_callback(self, *images):
        for image, rgb_name in zip(images, self.rgb_names):
            self.rgbs[rgb_name] = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        rgbs = self.rgbs.keys()
        if len(rgbs):
            rgbs_copy = self.rgbs.copy()
            imgs = torch.vstack(tuple([self.convert_to_torch(rgbs_copy[i]) for i in rgbs])).to(self.device, dtype=torch.float16)
            bounding_boxes_array = defaultdict()
            with torch.no_grad():
                preds = self.model(imgs)[0]

                preds = non_max_suppression(preds, rgb_config.model["conf_thres"], rgb_config.model["iou_thres"])
                
                for rgb_name, pred in zip(rgbs, preds):
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
                    self.pub_bounding_boxes[rgb_name].publish(bounding_boxes)
                    bounding_boxes_array[rgb_name] = bounding_boxes
            if default_config.use_yolo_image:
                for rgb_name in rgbs:
                    self.draw_yolo_image(rgb_name, rgbs_copy[rgb_name], bounding_boxes_array[rgb_name])


def main():
    rospy.init_node("Single RGB PyTorch YOLO")    

    inference_instance = YOLO()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
