#! /usr/bin/env python3
# coding=utf-8
from typing import Tuple
import cv2
import rospy
import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class DualVision():
    def __init__(self):
        self.sub_image = message_filters.Subscriber("/rgb/front_center/original_image", Image)
        self.sub_thermal = message_filters.Subscriber("/rgb/thermal/original_image", Image)

        self.pub_image = rospy.Publisher("/rgb/front_center/dual_image", Image, queue_size=1)
        self.pub_thermal = rospy.Publisher("/rgb/thermal/dual_image", Image, queue_size=1)
        self.pub_dual_vision = rospy.Publisher("/rgb/dual/image", Image, queue_size=1)
        self.pub_dual_vision_crop = rospy.Publisher("/rgb/dual/crop_image", Image, queue_size=1)

        """
        # 裁切起始處 
        crop_x = 248
        crop_y = 102

        # 裁切區域的長度與寬度
        crop_w = 563
        crop_h = 440    #458
        """
        self.roi = [102, 440, 248, 563]
        self.h = self.find_homography()
        self.alpha = 0.6
        self.beta = 1 - self.alpha
        self.gamma = 0
        self.bridge = CvBridge()

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_thermal], 1, 0.5)
        self.synchronizer.registerCallback(self.callback)
    
    def find_homography(self):
        # RGB Four corners 左上 左下 右上 右下
        pts_rgb     = np.array([[251,107],[252,426],[539,119],[543,436]])
        # Thermal Four corners
        pts_thermal = np.array([[29,10],[11,476],[589,30],[577,502]])
        h, status = cv2.findHomography(pts_thermal, pts_rgb)
        return h
    
    def pre_process(self, image: Image, size: Tuple[int, int]) -> cv2.Mat:
        img = self.bridge.imgmsg_to_cv2(image)
        img = cv2.resize(img, size, cv2.INTER_CUBIC)
        return img
    
    def post_process(self, image: cv2.Mat, size: Tuple[int, int]) -> Image:
        img = cv2.resize(image, size, cv2.INTER_CUBIC)
        img = self.bridge.cv2_to_imgmsg(img)
        return img

    def callback(self, image, thermal):
        image = self.pre_process(image, (800, 600))
        thermal = self.pre_process(thermal, (640, 512))

        thermal_warped = cv2.warpPerspective(thermal, self.h, (image.shape[1], image.shape[0]))
        thermal_colored = cv2.applyColorMap(thermal_warped, cv2.COLORMAP_JET)

        output = cv2.addWeighted(image, self.alpha, thermal_colored, self.beta, self.gamma)

        output_croped = output[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
        fusion = output[0:600, :]
        image_croped = image[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]
        thermal_croped = thermal_warped[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]]

        self.pub_image.publish(self.post_process(image_croped, (640, 480)))
        self.pub_thermal.publish(self.post_process(thermal_croped, (640, 480)))
        self.pub_dual_vision.publish(self.post_process(output_croped, (640, 480)))
        self.pub_dual_vision_crop.publish(self.post_process(fusion, (800, 600)))


def main():
    rospy.init_node("Dual Vision")

    dual_vision = DualVision()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
