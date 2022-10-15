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

        # 1280 * 712
        img_Width = 1280
        img_High = 712

        '''
            0: front_left
            1: front_center
            2: front_right
            3: rear_right
            4: rear_center
            5: rear_left
        '''
        # print(self.rgb_name)

        ''' #for Tasla camera
        if(self.rgb_name == 'front_left'):
            img = img[450:img_High - 20 ,350:img_Width]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width - 350, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'front_center'):
            img = img[430:img_High - 20 ,0:img_Width]
            # img = img[450:img_High - 20 ,0:img_Width]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 430 - 20)) # Image warping
            # warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'front_right'):
            img = img[450:img_High - 20 ,0:img_Width - 350]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width - 350, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'rear_right'):
            img = img[550:img_High - 20 ,350:img_Width]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width - 350, img_High - 550 - 20)) # Image warping

        elif(self.rgb_name == 'rear_center'):
            img = img[450:img_High - 20 ,0:img_Width]
            # img = img[550:img_High - 20 ,0:img_Width]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping
            # warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 550 - 20)) # Image warping

        elif(self.rgb_name == 'rear_left'):
            # img = img[550:img_High - 0 ,0:img_Width - 350]
            img = img[550:img_High - 20 ,0:img_Width - 350]
            
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            # warped_img = cv2.warpPerspective(img, M, (img_Width - 350, img_High - 550 - 0)) # Image warping
            warped_img = cv2.warpPerspective(img, M, (img_Width - 350, img_High - 550 - 20)) # Image warping
        '''

        # for Original camera
        if(self.rgb_name == 'front_left'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'front_center'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping


        elif(self.rgb_name == 'front_right'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'rear_right'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'rear_center'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        elif(self.rgb_name == 'rear_left'):
            img = img[450:img_High - 20 ,0:img_Width]
        
            src = np.float32([[0, img_High], [img_Width, img_High], [0, 0], [img_Width, 0]])
            dst = np.float32([[img_Width / 2 - 65, img_High], [img_Width / 2 + 65, img_High], [0, 0], [img_Width, 0]])
            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

            warped_img = cv2.warpPerspective(img, M, (img_Width, img_High - 450 - 20)) # Image warping

        msg = self.bridge.cv2_to_imgmsg(warped_img)
        # msg.header.stamp = rospy.Time.now()
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_BEV_rgb.publish(msg)

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
