#! /usr/bin/env python3
# coding=utf-8
import sys

from cv2 import ROTATE_180, ROTATE_90_CLOCKWISE, ROTATE_90_COUNTERCLOCKWISE

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

        def get_perspective_matrix():
            FL_src = np.float32([[200, 410], [900, 410], [-300, 700], [880, 700]])
            FL_dst = np.float32([[100, 200], [1280, 100], [500, 650], [880, 650]])

            FC_src = np.float32([[0, 712], [1280, 712], [0, 0], [1280, 0]])
            FC_dst = np.float32([[1280 / 2 - 65, 712], [1280 / 2 + 65, 712], [0, 0], [1280, 0]])

            FR_src = np.float32([[550, 410], [990, 410], [0, 650], [1540, 650]])
            FR_dst = np.float32([[0, 0], [1540, 0], [0, 650], [1540, 650]])

            RR_src = np.float32([[580, 400], [925, 400], [400, 700], [1105, 700]])
            RR_dst = np.float32([[400, 0], [1105, 0], [400, 700], [1105, 700]])

            RC_src = np.float32([[0, 712], [1280, 712], [0, 0], [1280, 0]])
            RC_dst = np.float32([[1280 / 2 - 65, 712], [1280 / 2 + 65, 712], [0, 0], [1280, 0]])

            RL_src = np.float32([[430, 430], [850, 430], [-230, 710], [1280, 710]])
            RL_dst = np.float32([[-200, 0], [1480, 0], [-230, 710], [1280,710]])

            return [cv2.getPerspectiveTransform(FL_src,FL_dst),
            cv2.getPerspectiveTransform(FC_src,FC_dst),
            cv2.getPerspectiveTransform(FR_src,FR_dst),
            cv2.getPerspectiveTransform(RR_src,RR_dst),
            cv2.getPerspectiveTransform(RC_src,RC_dst),
            cv2.getPerspectiveTransform(RL_src,RL_dst)]
            
        self.rgb_name = rgb_name
        self.config = rgb_config[self.rgb_name]
        self.bridge = CvBridge()
        self.perspective_matrix = get_perspective_matrix()
        # print(self.perspective_matrix)
        self.sub_rgb = rospy.Subscriber("calib_image", Image, self.callback, queue_size=1)
        self.pub_BEV_rgb = rospy.Publisher("BEV_image", Image, queue_size=1)

    

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        '''
            0: front_left
            1: front_center
            2: front_right
            3: rear_right
            4: rear_center
            5: rear_left
        '''
        
        # for Original camera
        if(self.rgb_name == 'front_left'):
            img = cv2.copyMakeBorder(img,0,0,0,1000,cv2.BORDER_CONSTANT,value=[0,0,0])
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[0], (1280,712)) 
            warped_img = cv2.rotate(warped_img,ROTATE_90_COUNTERCLOCKWISE)

        elif(self.rgb_name == 'front_center'):
            img = img[450:712 - 20 ,0:1280]
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[1], (1280, 712 - 450 - 20)) 

        elif(self.rgb_name == 'front_right'):
            img = cv2.copyMakeBorder(img,0,0,0,1000,cv2.BORDER_CONSTANT,value=[0,0,0])
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[2], (1280,712)) 
            warped_img = cv2.rotate(warped_img,ROTATE_90_CLOCKWISE)

        elif(self.rgb_name == 'rear_right'):
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[3], (1280,712)) 
            warped_img = cv2.rotate(warped_img,ROTATE_90_CLOCKWISE)

        elif(self.rgb_name == 'rear_center'):
            img = img[450:712 - 20 ,0:1280]
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[4], (1280, 712 - 450 - 20)) 
            warped_img = cv2.rotate(warped_img,ROTATE_180)

        elif(self.rgb_name == 'rear_left'):
            warped_img = cv2.warpPerspective(img, self.perspective_matrix[5], (1280,712)) 
            warped_img = cv2.rotate(warped_img,ROTATE_90_COUNTERCLOCKWISE)

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
