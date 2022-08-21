#! /usr/bin/env python3
# coding=utf-8
import sys
import subprocess

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from config import rgb_config, default_config, CameraType


class RGBToRos():
    def __init__(self, rgb_name):
        self.rgb_name = rgb_name
        self.config = rgb_config[self.rgb_name]

        self.pub_rgb = rospy.Publisher("original_image", Image, queue_size=1)

        # getting real rgb port
        cmd = "readlink -f " + self.config.port
        process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        out = process.communicate()[0]
        rospy.loginfo(out)
        port = [int(x) for x in str(out) if x.isdigit()]
        port = int(''.join(map(str,port)))
        rospy.loginfo(port)
        rospy.loginfo(self.rgb_name + " linked to " + str(port))

        # camera config
        self.cam = cv2.VideoCapture(port)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.size[0])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.size[1])
        # self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*list(self.config.codec)))
        self.cam.set(cv2.CAP_PROP_FPS, self.config.frame_rate)
        if self.config.camera_type == CameraType.RGB:
            self.cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.bridge = CvBridge()

    def loop(self):
        ret, image = self.cam.read()
        if not ret:
            rospy.logerr("error capturing rgb cam" + self.rgb_name)
            return
        # convert to cv image
        if self.config.camera_type == CameraType.RGB:
            image = cv2.imdecode(image, cv2.IMREAD_UNCHANGED)
        
        # ORIGINAL
        img_message = self.bridge.cv2_to_imgmsg(image)

        # TEST
        # convert to gray scale
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # img_message = self.bridge.cv2_to_imgmsg(image, encoding="mono8")

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
