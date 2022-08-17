#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from ars408_ros import rgb_config


class RGBCalib():
    def __init__(self, rgb_name):
        self.rgb_name = rgb_name
        self.config = rgb_config[self.rgb_name]

        self.sub_rgb = rospy.Subscriber("original_image", Image, self.callback, queue_size=1)
        self.pub_calib_rgb = rospy.Publisher("calib_image", Image, queue_size=1)

        self.bridge = CvBridge()

        camera_matrix, valid_roi = cv2.getOptimalNewCameraMatrix(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, 1, self.config.size)
        self.camera_matrix = camera_matrix
        self.valid_roi = valid_roi
        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.R, self.config.P, self.config.size, 5)

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # img = cv2.undistort(img, self.c_matrix, self.d_matrix, None, self.camera_matrix)

        img = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)

        x, y, w, h = self.valid_roi
        # img = img[y:y+h, x:x+w]
        # img = cv2.resize(img, self.config.size, cv2.INTER_CUBIC)
        self.pub_calib_rgb.publish(self.bridge.cv2_to_imgmsg(img))

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