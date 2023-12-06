#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
count = 0

def callback(msg):
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    cv2.imwrite(os.path.join("/disk/bags/images", "frame%06i.png" % count), cv_img)
    if count % 50 == 0:
        print(os.path.join("/disk/bags/images/", "frame%06i.png" % count))
        print("Wrote image %i" % count)

    count += 1

def main():
    """Extract a folder of images from a rosbag.
    """

    rospy.init_node("export_image")
    sub = rospy.Subscriber("/cam/front_bottom_60", Image, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()
