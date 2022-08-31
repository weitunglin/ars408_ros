#! /usr/bin/env python3
# coding=utf-8
from email import message
import threading
import os
import sys
from collections import defaultdict
from functools import partial

import rospy
import torch
import cv2
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package")
sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/NVS")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/NVS")
from NVS.models.experimental import attempt_load
from NVS.utils.datasets import LoadStreams, LoadImages
from NVS.utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
        scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box, xywh2xyxy
from NVS.utils.plots import colors, plot_one_box
from NVS.utils.torch_utils import select_device, load_classifier, time_synchronized

from ars408_msg.msg import Bboxes, Bbox
from config import default_config, rgb_config


class DUAL_YOLO():
    def __init__(self):
        self.sub_rgb = defaultdict()
        self.sub_trm = defaultdict()

        self.pub_bounding_boxes = defaultdict()
        self.pub_yolo_images = defaultdict()

        self.rgbs = defaultdict()

        self.bridge = CvBridge()
        self.setup_model()

        self.rgb_names = rgb_config.model["dual_vision"]

        #---sub---#
        #front_center
        self.sub_rgb[self.rgb_names[0]] = message_filters.Subscriber("/rgb/" + self.rgb_names[0] + "/calib_image", Image)
        #thermal
        self.sub_rgb[self.rgb_names[1]] = message_filters.Subscriber("/rgb/" + self.rgb_names[1] + "/original_image", Image)
        self.synchronizer = message_filters.TimeSynchronizer([self.sub_rgb[self.rgb_names[0]], self.sub_rgb[self.rgb_names[1]]], 1)
        self.synchronizer.registerCallback(self.callback)
        
        #---pub---#
        #bbox
        self.pub_bounding_boxes[self.rgb_names[0]] = rospy.Publisher("/rgb/" + self.rgb_names[0] + "/bounding_boxes", Bboxes, queue_size=1)
        #yolo
        if default_config.use_yolo_image:
                self.pub_yolo_images[self.rgb_names[0]] = rospy.Publisher("/rgb/" + self.rgb_names[0] + "/yolo_image", Image, queue_size=1)

    
    def setup_model(self):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = attempt_load(rgb_config.model["dual_weights"], map_location=self.device)
        if default_config.use_cuda:
            self.model.cuda()

        self.model.to(self.device).eval()
        if default_config.use_yolo_half:
            self.model.half()
        
        self.class_names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names

    def convert_to_torch(self, image):
        img = image.copy()
        img = cv2.resize(image, rgb_config.model["dual_image_size"])
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = torch.from_numpy(img).to(self.device, dtype=torch.float16).div(255.0)
        img = torch.permute(img, (2, 0, 1))
        img = torch.unsqueeze(img, 0)
        return img
    
    def draw_yolo_image(self, rgb_name, bounding_boxes):
        img = self.rgbs[rgb_name].copy()
        # img = cv2.resize(img, rgb_config.model["image_size"])
        for i in bounding_boxes.bboxes:
            cv2.putText(img, i.objClass, (int(i.x_min), int(i.y_min) - 10), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 5)
            cv2.rectangle(img, (int(i.x_min), int(i.y_min)), (int(i.x_max), int(i.y_max)), color=(255, 0, 0), thickness=5)
        self.pub_yolo_images[rgb_name].publish(self.bridge.cv2_to_imgmsg(img))

    def callback(self, rgb_image, thermal_image):
        self.rgbs["front_center"] = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="passthrough")
        self.rgbs["thermal"] = self.bridge.imgmsg_to_cv2(thermal_image, desired_encoding="passthrough")
        rgbs = self.rgbs.keys()
        if len(rgbs) >= 2:
            with torch.no_grad():
                img = self.convert_to_torch(self.rgbs[self.rgb_names[0]]).to(self.device, dtype=torch.float16)
                img2 = self.convert_to_torch(self.rgbs[self.rgb_names[1]]).to(self.device, dtype=torch.float16)
                preds = self.model(img, img2)[0]

                preds = non_max_suppression(preds, rgb_config.model["dual_conf_thres"], rgb_config.model["dual_iou_thres"])
                
                rgb_name = self.rgb_names[0]
                for pred in preds:
                    bounding_boxes = Bboxes()
                    pred = pred.cpu().numpy()
                    for *xyxy, conf, cls in pred:
                        bounding_boxes.bboxes.append(
                            Bbox(
                                x_min=xyxy[0] / rgb_config.model["dual_image_size"][0] * rgb_config[rgb_name].size[0],
                                y_min=xyxy[1] / rgb_config.model["dual_image_size"][1] * rgb_config[rgb_name].size[1],
                                x_max=xyxy[2] / rgb_config.model["dual_image_size"][0] * rgb_config[rgb_name].size[0],
                                y_max=xyxy[3] / rgb_config.model["dual_image_size"][1] * rgb_config[rgb_name].size[1],
                                score=conf,
                                objClassNum=int(cls),
                                objClass=self.class_names[int(cls)],
                            )
                        )
                    self.pub_bounding_boxes[rgb_name].publish(bounding_boxes)
                    if default_config.use_yolo_image:
                        self.draw_yolo_image(rgb_name, bounding_boxes)


def main():
    rospy.init_node("RGB Thermal PyTorch YOLO")    

    inference_instance = DUAL_YOLO()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
