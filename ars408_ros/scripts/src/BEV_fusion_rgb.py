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

        # camera_matrix, valid_roi = cv2.getOptimalNewCameraMatrix(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, 1, self.config.size)
        # self.camera_matrix = camera_matrix
        # self.valid_roi = valid_roi

        # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, self.config.R, balance=1)
        # self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.R, new_K, self.config.size, 5)
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

        rot_front_left = np.rot90(rgb_image_array[0], 2)
        # rot_front_center = np.rot90(rgb_image_array[1], 0)
        rot_front_right = np.rot90(rgb_image_array[2], 2)
        # rot_rear_right = np.rot90(rgb_image_array[3], 0)
        rot_rear_center = np.rot90(rgb_image_array[4], 2)
        # rot_rear_left = np.rot90(rgb_image_array[5], 0)
        # rot_rearCenter = np.rot90(rgb_image_array[4], 2)

        image_result = cv2.vconcat([rgb_image_array[1], rot_rear_center])

        msg = self.bridge.cv2_to_imgmsg(image_result)
        # msg.header.stamp = rospy.Time.now()
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_calib_rgb.publish(msg)
    


if __name__ == "__main__":
    try:
        # if len(sys.argv) < 2:
        #     rospy.logerr("error getting rgb_name. got {}".format(",".join(sys.argv)))
        #     exit(-1)
        # else:
        #     rgb_name = sys.argv[1]

        rospy.init_node("RGB BEV Fusion")

        
        rgb_to_ros = RGBBEVFusion()
        rospy.spin()

    except rospy.ROSInternalException:
        pass
