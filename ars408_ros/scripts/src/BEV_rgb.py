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

class RGBBEV():
    def __init__(self, rgb_name):
        self.rgb_name = rgb_name
        self.config = rgb_config[self.rgb_name]
        self.bridge = CvBridge()

        # camera_matrix, valid_roi = cv2.getOptimalNewCameraMatrix(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, 1, self.config.size)
        # self.camera_matrix = camera_matrix
        # self.valid_roi = valid_roi

        # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.size, self.config.R, balance=1)
        # self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(self.config.intrinsic_matrix, self.config.distortion_matrix, self.config.R, new_K, self.config.size, 5)

        self.sub_rgb = rospy.Subscriber("calib_image", Image, self.callback, queue_size=1)
        self.pub_BEV_rgb = rospy.Publisher("BEV_image", Image, queue_size=1)

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        ################
        # OpenCV
        ################
        # Vertices coordinates in the source image
        # 1280 * 712
        img_Width = 1280
        img_High = 712
        # s = np.array([[830, 598],
        #           [868, 568],
        #           [1285, 598],
        #           [1248, 567]], dtype=np.float32)
        # s = np.array([[510, 420],
        #             [534, 380],
        #             [794, 420],
        #             [770, 380]], dtype=np.float32)

        # Vertices coordinates in the destination image
        # t = np.array([[177, 231],
        #             [213, 231],
        #             [178, 264],
        #             [216, 264]], dtype=np.float32)
        
        # add size to 1000 * 1000
        # t = np.array([[177*2, 231*2],  
        #             [213*2, 231*2],
        #             [178*2, 264*2],
        #             [216*2, 264*2]], dtype=np.float32)

        # 
        # t = np.array([[177*2, 264*2],
        #             [178*2, 231*2],
        #             [216*2, 264*2],
        #             [213*2, 231*2]], dtype=np.float32)

        # Warp the image
        # img = self.ipm_from_opencv(img, s, t)
        # height, width = img.shape[:2]
        # print(height, width)
        img = img[450:img_High - 20 ,0:img_Width]
        
        src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
        dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

        warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        msg = self.bridge.cv2_to_imgmsg(warped_img)
        # msg.header.stamp = rospy.Time.now()
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_BEV_rgb.publish(msg)
    
    # def ipm_from_opencv(self, image, source_points, target_points):
    #     # Compute projection matrix
    #     M = cv2.getPerspectiveTransform(source_points, target_points)
    #     # TARGET_H, TARGET_W = 500, 500
    #     TARGET_H, TARGET_W = 1000, 1000

    #     # Warp the image
    #     warped = cv2.warpPerspective(image, M, (TARGET_W, TARGET_H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
    #                                 borderValue=0)
    #     return warped
    


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            rospy.logerr("error getting rgb_name. got {}".format(",".join(sys.argv)))
            exit(-1)
        else:
            rgb_name = sys.argv[1]

        rospy.init_node("RGB BEV")

        
        rgb_to_ros = RGBBEV(rgb_name)
        rospy.spin()

    except rospy.ROSInternalException:
        pass
