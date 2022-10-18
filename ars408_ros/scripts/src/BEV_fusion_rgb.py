#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from config.config import rgb_config, CameraType

import os

class RGBBEVFusion():
    def __init__(self):
        # self.rgb_name = rgb_name
        self.config = rgb_config.names
        self.bridge = CvBridge()

        """
        setup synchronizer for all rgb and radar pair.
        """
        self.sub = list()
        for rgb_name in self.config:
            if rgb_config[rgb_name].camera_type == CameraType.RGB:
                name = rgb_name
                # add rgb subscriber
                self.sub.append(message_filters.Subscriber(f"/rgb/{name}/BEV_image", Image))
                # add radar subscriber
        
        # synchronizer
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(self.sub, queue_size=6, slop=10)
        self.synchronizer.registerCallback(self.callback)

        self.pub_calib_rgb = rospy.Publisher("BEV_fusion_image", Image, queue_size=1)

    def callback(self, *msgs):
        rgb_image_array: list[cv2.Mat] = [] # array of `cv2.Mat` for yolo inference

        # preprocess msgs
        for i in range(len(self.config) - 1):
            '''
            0: front_left
            1: front_center
            2: front_right
            3: rear_right
            4: rear_center
            5: rear_left
            '''
            rgb_image_array.append(self.bridge.imgmsg_to_cv2(msgs[i]))
        
        # Crop to fusion
        rgb_image_array[0] = rgb_image_array[0][0:1000,0:712]
        rgb_image_array[2] = rgb_image_array[2][0:1000,0:712]

        right = cv2.vconcat([rgb_image_array[2],rgb_image_array[3]])
        left = cv2.vconcat([rgb_image_array[0], rgb_image_array[5]])

        fusion_result = cv2.copyMakeBorder(left,0,0,0,500,cv2.BORDER_CONSTANT,value=[0,0,0])
        fusion_result = cv2.hconcat([fusion_result,right])

        top = cv2.copyMakeBorder(rgb_image_array[1],0,0,322,322,cv2.BORDER_CONSTANT,value=[0,0,0])
        bottom = cv2.copyMakeBorder(rgb_image_array[4],0,0,322,322,cv2.BORDER_CONSTANT,value=[0,0,0])
 
        fusion_result = cv2.vconcat([top,fusion_result])
        fusion_result = cv2.vconcat([fusion_result,bottom])
        msg = self.bridge.cv2_to_imgmsg(fusion_result)
        # msg.header.stamp = rospy.Time.now()
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_calib_rgb.publish(msg)
    


if __name__ == "__main__":
    try:
        rospy.init_node("RGB BEV Fusion")    
        rgb_to_ros = RGBBEVFusion()
        rospy.spin()

    except rospy.ROSInternalException:
        pass
