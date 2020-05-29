#! /usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os

topic_RGB = "/rgbImg"
topic_TRM = "/thermalImg"

path_RGB = os.path.expanduser("~/rgb_")
path_TRM = os.path.expanduser("~/thermal_")

size_RGB = (800, 600)
size_TRM = (640, 512)

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global out_RGB
    out_RGB.write(img)

def callback_TRMImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global out_TRM
    out_TRM.write(img)
    
def listener():
    rospy.init_node("toVideo", anonymous=False)
    global out_RGB, out_TRM
    out_RGB = cv2.VideoWriter(path_RGB+str(rospy.Time.now())+".avi", cv2.VideoWriter_fourcc(*'DIVX'), 20, size_RGB, True)
    out_TRM = cv2.VideoWriter(path_TRM+str(rospy.Time.now())+".avi", cv2.VideoWriter_fourcc(*'DIVX'), 20, size_TRM, True)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg)
    sub_TRM = rospy.Subscriber(topic_TRM, Image, callback_TRMImg)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
    