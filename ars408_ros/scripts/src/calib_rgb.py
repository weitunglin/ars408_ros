#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy
import cv2
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from config.config import rgb_config

import os

class RGBCalib():
    def __init__(self, rgb_name):
        self.rgb_name = rgb_name
        self.config = rgb_config[self.rgb_name]
        self.bridge = CvBridge()

        # camera_matrix, valid_roi = cv2.getOptimalNewCameraMatrix(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, 1, self.config.size)
        # self.camera_matrix = camera_matrix
        # self.valid_roi = valid_roi

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, self.config.R, balance=1)
        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.R, new_K, self.config.size, 5)

        self.sub_rgb = rospy.Subscriber("original_image", Image, self.callback, queue_size=1)
        self.pub_calib_rgb = rospy.Publisher("calib_image", Image, queue_size=1)

    def callback(self, msg):
        # img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        """
        fisheye calibration with bigger valid roi remaining
        """
        # img = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)
        # off_y = 85
        # img = img[off_y:img.shape[0]-off_y , :]
        # img = cv2.resize(img, self.config.size, cv2.INTER_CUBIC)

        # msg = self.bridge.cv2_to_imgmsg(img)
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_calib_rgb.publish(msg)

if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            rospy.logerr("error getting rgb_name. got {}".format(",".join(sys.argv)))
            exit(-1)
        else:
            rgb_name = sys.argv[1]

        rospy.init_node("RGB Calib")

        
        rgb_to_ros = RGBCalib(rgb_name)
        rospy.spin()

    except rospy.ROSInternalException:
        pass
