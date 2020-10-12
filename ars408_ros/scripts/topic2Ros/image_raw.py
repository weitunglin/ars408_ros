#! /usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2

frameRate = 20
topic_RGB = "/rgbImg"
size_RGB = (640, 480)
global pub_RGB

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    pub_RGB.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

def listener():
    global pub_RGB
    rospy.init_node("image_raw")
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    pub_RGB = rospy.Publisher("/rgbImg/image_raw", Image, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
