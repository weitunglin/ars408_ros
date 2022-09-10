#! /usr/bin/env python3
# coding=utf-8
import os
import sys
from collections import defaultdict
from typing import List
from functools import partial

import rospy
import torch
import cv2
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/LTA")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/LTA")

from lib.config import cfg
from lib.models import get_net
from lib.utils_resa.config import Config
from lib.runner.demo_runner import DemoString

from config import default_config, rgb_config

class LaneTrace():
    def __init__(self):
        self.sub_rgb = defaultdict()

        self.pub_yolo_images = defaultdict()

        self.rgbs = defaultdict()

        self.bridge = CvBridge()
        self.setup_model()

        self.rgb_names = ['front_center']

        #---sub---#
        #front_center
        self.sub_rgb[self.rgb_names[0]] = message_filters.Subscriber("/rgb/" + self.rgb_names[0] + "/calib_image", Image)
        self.synchronizer = message_filters.TimeSynchronizer([self.sub_rgb[self.rgb_names[0]]],1)
        self.synchronizer.registerCallback(self.callback)

        #---pub---#

        #yolo
        if default_config.use_yolo_image:
                self.pub_yolo_images[self.rgb_names[0]] = rospy.Publisher("/rgb/" + self.rgb_names[0] + "/lta_image", Image, queue_size=1)


    def setup_model(self):
        
        # load model
        self.model = get_net(cfg)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        # load weight
        self.checkpoint = torch.load(cfg.MODEL.PRETRAINED, map_location= self.device)
        self.model.load_state_dict(self.checkpoint['net'],strict = False)

        if default_config.use_cuda:
            self.model.cuda()
        self.model.to(self.device).eval()

        # Load Config file--------------------------------
        self.cfg_resa = Config.fromfile('./lib/config/resa_tusimple.py')
        self.cfg_resa.gpus = 1
        self.cfg_resa.load_from = cfg.MODEL.PRETRAINED
        self.cfg_resa.finetune_from = None
        self.cfg_resa.view = True
        self.cfg_resa.work_dirs = 'work_dirs/TuSimple'


    def callback(self, image):
        rgb_name = self.rgb_names[0]
        self.rgbs[rgb_name] = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        img = self.rgbs[self.rgb_names[0]]

        #initial data
        DemoRunner = DemoString(None ,self.model,self.checkpoint,self.device,self.cfg_resa,'ros',img= img)
        #run demo
        img = DemoRunner.run()

        self.pub_yolo_images[rgb_name].publish(self.bridge.cv2_to_imgmsg(img))


def main():
    rospy.init_node("Lane Trace")    

    lane_tarce = LaneTrace()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass