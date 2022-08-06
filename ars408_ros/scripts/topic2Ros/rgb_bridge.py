#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

sys.path.append('../../config')
from config import rgb_config, default_config


class RGBToRos():
    def __init__(self, rgb_name):
        self.rgb_name = rgb_name
        config = rgb_config[self.rgb_name]

        self.pub_rgb = rospy.Publisher("original_image", Image, queue_size=1)

        # camera config
        self.cam = cv2.VideoCapture(config.port)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, config.size[0])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, config.size[1])
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*list(config.codec)))
        self.cam.set(cv2.CAP_PROP_FPS, config.frame_rate)

        self.bridge = CvBridge()

    def loop(self):
        ret, image = self.cam.read()
        if not ret:
            # rospy.logerr("error capturing rgb cam" + self.rgb_name)
            return
        # convert to cv image
        img_message = self.bridge.cv2_to_imgmsg(image)
        # publish
        self.pub_rgb.publish(img_message)


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            rospy.logerr("error getting rgb_name. got {}".format(",".join(sys.argv)))
            exit(-1)
        else:
            rgb_name = sys.argv[1]

        rospy.init_node("RGB Bridge")

        
        rgb_to_ros = RGBToRos(rgb_name)
        rate = rospy.Rate(default_config.frame_rate)

        while not rospy.is_shutdown():
            rgb_to_ros.loop()
            rate.sleep()

    except rospy.ROSInternalException:
        pass
